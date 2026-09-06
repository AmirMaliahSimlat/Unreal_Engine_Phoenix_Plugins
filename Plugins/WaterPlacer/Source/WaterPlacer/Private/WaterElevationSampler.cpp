#include "WaterElevationSampler.h"
#include "WaterPlacerLog.h"

#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

THIRD_PARTY_INCLUDES_START
#include "zlib.h"
THIRD_PARTY_INCLUDES_END

namespace
{
		constexpr int32 QmCacheSize = 48;
		constexpr double Pi = 3.14159265358979323846;
		constexpr double DegToRad = Pi / 180.0;

		struct FQmTileId
		{
			int32 Z = 0;
			int32 X = 0;
			int32 Y = 0;

			friend bool operator==(const FQmTileId& A, const FQmTileId& B)
			{
				return A.Z == B.Z && A.X == B.X && A.Y == B.Y;
			}

			friend uint32 GetTypeHash(const FQmTileId& T)
			{
				return HashCombine(HashCombine(GetTypeHash(T.Z), GetTypeHash(T.X)), GetTypeHash(T.Y));
			}
		};

		struct FQmDecoded
		{
			TArray<double> U;
			TArray<double> V;
			TArray<double> HeightM;
			TArray<int32> Indices;
			double West = 0.0;
			double South = 0.0;
			double East = 0.0;
			double North = 0.0;
		};

		struct FRaster
		{
			int32 Width = 0;
			int32 Height = 0;
			double OriginX = 0.0;
			double OriginY = 0.0;
			double PixelW = 1.0;
			double PixelH = -1.0;
			bool bGeographic = true;
			int32 UtmZone = 0;
			bool bUtmNorth = true;
			float NoData = -9999.0f;
			TArray<float> Heights;
		};

		bool GunzipIfNeeded(const TArray<uint8>& In, TArray<uint8>& Out)
		{
			if (In.Num() < 2 || In[0] != 0x1f || In[1] != 0x8b)
			{
				Out = In;
				return true;
			}

			z_stream Stream;
			FMemory::Memzero(Stream);
			if (inflateInit2(&Stream, 16 + MAX_WBITS) != Z_OK)
			{
				return false;
			}

			Stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(In.GetData()));
			Stream.avail_in = static_cast<uInt>(In.Num());
			Out.Reset();
			uint8 Buf[1 << 15];
			int Ret = Z_OK;
			do
			{
				Stream.next_out = Buf;
				Stream.avail_out = sizeof(Buf);
				Ret = inflate(&Stream, Z_NO_FLUSH);
				const int32 Got = static_cast<int32>(sizeof(Buf) - Stream.avail_out);
				if (Got > 0)
				{
					Out.Append(Buf, Got);
				}
				if (Ret == Z_STREAM_ERROR || Ret == Z_DATA_ERROR || Ret == Z_MEM_ERROR)
				{
					inflateEnd(&Stream);
					return false;
				}
			}
			while (Ret != Z_STREAM_END);

			inflateEnd(&Stream);
			return true;
		}

		bool InflateZlib(const uint8* Src, int32 SrcLen, TArray<uint8>& Out, int32 HintSize)
		{
			auto TryWindow = [&](int WindowBits) -> bool
			{
				z_stream Stream;
				FMemory::Memzero(Stream);
				if (inflateInit2(&Stream, WindowBits) != Z_OK)
				{
					return false;
				}
				Stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(Src));
				Stream.avail_in = static_cast<uInt>(SrcLen);
				Out.Reset();
				Out.Reserve(HintSize > 0 ? HintSize : SrcLen * 4);
				uint8 Buf[1 << 15];
				int Ret = Z_OK;
				do
				{
					Stream.next_out = Buf;
					Stream.avail_out = sizeof(Buf);
					Ret = inflate(&Stream, Z_NO_FLUSH);
					const int32 Got = static_cast<int32>(sizeof(Buf) - Stream.avail_out);
					if (Got > 0)
					{
						Out.Append(Buf, Got);
					}
					if (Ret == Z_STREAM_ERROR || Ret == Z_DATA_ERROR || Ret == Z_MEM_ERROR)
					{
						inflateEnd(&Stream);
						return false;
					}
				}
				while (Ret != Z_STREAM_END);
				inflateEnd(&Stream);
				return true;
			};

			return TryWindow(MAX_WBITS) || TryWindow(-MAX_WBITS);
		}

		int32 ZigZagDecode(uint16 Encoded)
		{
			return static_cast<int32>(Encoded >> 1) ^ -static_cast<int32>(Encoded & 1);
		}

		int32 XTilesAtLevel(int32 Z)
		{
			return 2 << FMath::Clamp(Z, 0, 30);
		}

		int32 YTilesAtLevel(int32 Z)
		{
			return 1 << FMath::Clamp(Z, 0, 30);
		}

		void TileRectangleTms(int32 Z, int32 X, int32 Y, double& West, double& South, double& East, double& North)
		{
			const double XTiles = static_cast<double>(XTilesAtLevel(Z));
			const double YTiles = static_cast<double>(YTilesAtLevel(Z));
			West = -180.0 + (360.0 * static_cast<double>(X)) / XTiles;
			East = -180.0 + (360.0 * static_cast<double>(X + 1)) / XTiles;
			South = -90.0 + (180.0 * static_cast<double>(Y)) / YTiles;
			North = -90.0 + (180.0 * static_cast<double>(Y + 1)) / YTiles;
		}

		void TileRectangleCesium(int32 Z, int32 X, int32 Y, double& West, double& South, double& East, double& North)
		{
			const double XTiles = static_cast<double>(XTilesAtLevel(Z));
			const double YTiles = static_cast<double>(YTilesAtLevel(Z));
			West = -180.0 + (360.0 * static_cast<double>(X)) / XTiles;
			East = -180.0 + (360.0 * static_cast<double>(X + 1)) / XTiles;
			North = 90.0 - (180.0 * static_cast<double>(Y)) / YTiles;
			South = 90.0 - (180.0 * static_cast<double>(Y + 1)) / YTiles;
		}

		void LonLatToTileTms(int32 Z, double Lon, double Lat, int32& X, int32& Y)
		{
			const int32 XT = XTilesAtLevel(Z);
			const int32 YT = YTilesAtLevel(Z);
			X = FMath::Clamp(FMath::FloorToInt((Lon + 180.0) / 360.0 * XT), 0, XT - 1);
			Y = FMath::Clamp(FMath::FloorToInt((Lat + 90.0) / 180.0 * YT), 0, YT - 1);
		}

		void LonLatToTileCesium(int32 Z, double Lon, double Lat, int32& X, int32& Y)
		{
			const int32 XT = XTilesAtLevel(Z);
			const int32 YT = YTilesAtLevel(Z);
			X = FMath::Clamp(FMath::FloorToInt((Lon + 180.0) / 360.0 * XT), 0, XT - 1);
			Y = FMath::Clamp(FMath::FloorToInt((90.0 - Lat) / 180.0 * YT), 0, YT - 1);
		}

		bool LonLatToUtm(double LonDeg, double LatDeg, int32 Zone, bool bNorth, double& Easting, double& Northing)
		{
			if (Zone < 1 || Zone > 60)
			{
				return false;
			}
			const double A = 6378137.0;
			const double F = 1.0 / 298.257223563;
			const double K0 = 0.9996;
			const double E2 = F * (2.0 - F);
			const double EP2 = E2 / (1.0 - E2);
			const double Lon0 = static_cast<double>(Zone * 6 - 183) * DegToRad;
			const double Lat = LatDeg * DegToRad;
			const double Lon = LonDeg * DegToRad;
			const double N = A / FMath::Sqrt(1.0 - E2 * FMath::Sin(Lat) * FMath::Sin(Lat));
			const double T = FMath::Tan(Lat) * FMath::Tan(Lat);
			const double C = EP2 * FMath::Cos(Lat) * FMath::Cos(Lat);
			const double ALon = (Lon - Lon0) * FMath::Cos(Lat);
			const double M = A * ((1.0 - E2 / 4.0 - 3.0 * E2 * E2 / 64.0 - 5.0 * E2 * E2 * E2 / 256.0) * Lat
				- (3.0 * E2 / 8.0 + 3.0 * E2 * E2 / 32.0 + 45.0 * E2 * E2 * E2 / 1024.0) * FMath::Sin(2.0 * Lat)
				+ (15.0 * E2 * E2 / 256.0 + 45.0 * E2 * E2 * E2 / 1024.0) * FMath::Sin(4.0 * Lat)
				- (35.0 * E2 * E2 * E2 / 3072.0) * FMath::Sin(6.0 * Lat));
			Easting = K0 * N * (ALon + (1.0 - T + C) * ALon * ALon * ALon / 6.0
				+ (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * EP2) * ALon * ALon * ALon * ALon * ALon / 120.0)
				+ 500000.0;
			Northing = K0 * (M + N * FMath::Tan(Lat) * (ALon * ALon / 2.0
				+ (5.0 - T + 9.0 * C + 4.0 * C * C) * ALon * ALon * ALon * ALon / 24.0
				+ (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * EP2) * ALon * ALon * ALon * ALon * ALon * ALon / 720.0));
			if (!bNorth)
			{
				Northing += 10000000.0;
			}
			return true;
		}

		int32 UtmZoneFromEpsg(int32 Epsg, bool& bNorth)
		{
			if (Epsg >= 32601 && Epsg <= 32660)
			{
				bNorth = true;
				return Epsg - 32600;
			}
			if (Epsg >= 32701 && Epsg <= 32760)
			{
				bNorth = false;
				return Epsg - 32700;
			}
			return 0;
		}

		bool ParsePrjUtm(const FString& PrjPath, int32& OutZone, bool& bNorth)
		{
			FString Text;
			if (!FFileHelper::LoadFileToString(Text, *PrjPath))
			{
				return false;
			}
			int32 Epsg = 0;
			const int32 Auth = Text.Find(TEXT("EPSG"), ESearchCase::IgnoreCase);
			if (Auth != INDEX_NONE)
			{
				FString Tail = Text.Mid(Auth);
				FString Digits;
				for (const TCHAR C : Tail)
				{
					if (FChar::IsDigit(C))
					{
						Digits.AppendChar(C);
					}
					else if (Digits.Len() >= 4)
					{
						break;
					}
				}
				Epsg = FCString::Atoi(*Digits);
			}
			OutZone = UtmZoneFromEpsg(Epsg, bNorth);
			return OutZone > 0;
		}

		struct FBin
		{
			const uint8* D = nullptr;
			int32 N = 0;
			int32 I = 0;
			bool bOk = true;

			template <typename T>
			T Read()
			{
				if (I + static_cast<int32>(sizeof(T)) > N)
				{
					bOk = false;
					return T{};
				}
				T V;
				FMemory::Memcpy(&V, D + I, sizeof(T));
				I += sizeof(T);
				return V;
			}

			void Align(int32 Bytes)
			{
				if (Bytes <= 0)
				{
					return;
				}
				const int32 Mis = I % Bytes;
				if (Mis != 0)
				{
					I += Bytes - Mis;
					if (I > N)
					{
						bOk = false;
					}
				}
			}
		};

		bool DecodeQuantizedMesh(const TArray<uint8>& Raw, int32 Z, int32 X, int32 Y, bool bTms, FQmDecoded& Out)
		{
			TArray<uint8> Bytes;
			if (!GunzipIfNeeded(Raw, Bytes) || Bytes.Num() < 88)
			{
				return false;
			}

			FBin B{ Bytes.GetData(), Bytes.Num(), 0, true };
			B.Read<double>();
			B.Read<double>();
			B.Read<double>();
			const float MinH = B.Read<float>();
			const float MaxH = B.Read<float>();
			for (int32 I = 0; I < 7; ++I)
			{
				B.Read<double>();
			}
			const uint32 VertexCount = B.Read<uint32>();
			if (!B.bOk || VertexCount == 0 || VertexCount > 2000000u)
			{
				return false;
			}

			TArray<int32> UAbs, VAbs, HAbs;
			UAbs.SetNumUninitialized(VertexCount);
			VAbs.SetNumUninitialized(VertexCount);
			HAbs.SetNumUninitialized(VertexCount);
			int32 UAcc = 0, VAcc = 0, HAcc = 0;
			for (uint32 I = 0; I < VertexCount; ++I)
			{
				UAcc += ZigZagDecode(B.Read<uint16>());
				UAbs[I] = UAcc;
			}
			for (uint32 I = 0; I < VertexCount; ++I)
			{
				VAcc += ZigZagDecode(B.Read<uint16>());
				VAbs[I] = VAcc;
			}
			for (uint32 I = 0; I < VertexCount; ++I)
			{
				HAcc += ZigZagDecode(B.Read<uint16>());
				HAbs[I] = HAcc;
			}
			if (!B.bOk)
			{
				return false;
			}

			const bool b32 = VertexCount > 65536u;
			uint32 TriCount = 0;
			if (b32)
			{
				B.Align(4);
				TriCount = B.Read<uint32>();
			}
			else
			{
				B.Align(2);
				TriCount = B.Read<uint16>();
			}
			if (!B.bOk || TriCount > 4000000u)
			{
				return false;
			}

			Out.Indices.Reset();
			Out.Indices.Reserve(TriCount * 3);
			int32 Highest = 0;
			auto ReadIndex = [&]() -> int32
			{
				const uint32 Code = b32 ? B.Read<uint32>() : static_cast<uint32>(B.Read<uint16>());
				const int32 Result = Highest - static_cast<int32>(Code);
				if (Code == 0)
				{
					++Highest;
				}
				return Result;
			};
			for (uint32 T = 0; T < TriCount * 3u; ++T)
			{
				Out.Indices.Add(ReadIndex());
			}
			if (!B.bOk)
			{
				return false;
			}

			if (bTms)
			{
				TileRectangleTms(Z, X, Y, Out.West, Out.South, Out.East, Out.North);
			}
			else
			{
				TileRectangleCesium(Z, X, Y, Out.West, Out.South, Out.East, Out.North);
			}

			Out.U.SetNumUninitialized(VertexCount);
			Out.V.SetNumUninitialized(VertexCount);
			Out.HeightM.SetNumUninitialized(VertexCount);
			for (uint32 I = 0; I < VertexCount; ++I)
			{
				Out.U[I] = FMath::Clamp(static_cast<double>(UAbs[I]) / 32767.0, 0.0, 1.0);
				Out.V[I] = FMath::Clamp(static_cast<double>(VAbs[I]) / 32767.0, 0.0, 1.0);
				Out.HeightM[I] = static_cast<double>(MinH)
					+ (static_cast<double>(MaxH) - static_cast<double>(MinH))
					* FMath::Clamp(static_cast<double>(HAbs[I]) / 32767.0, 0.0, 1.0);
			}
			return Out.Indices.Num() >= 3;
		}

		bool PointInTri(double Px, double Py, double Ax, double Ay, double Bx, double By, double Cx, double Cy, double& W0, double& W1, double& W2)
		{
			const double Den = (By - Cy) * (Ax - Cx) + (Cx - Bx) * (Ay - Cy);
			if (FMath::Abs(Den) < 1.0e-18)
			{
				return false;
			}
			W0 = ((By - Cy) * (Px - Cx) + (Cx - Bx) * (Py - Cy)) / Den;
			W1 = ((Cy - Ay) * (Px - Cx) + (Ax - Cx) * (Py - Cy)) / Den;
			W2 = 1.0 - W0 - W1;
			return W0 >= -1.0e-8 && W1 >= -1.0e-8 && W2 >= -1.0e-8;
		}

		bool SampleDecodedQm(const FQmDecoded& Tile, double Lon, double Lat, double& OutHeightM)
		{
			const double SpanX = Tile.East - Tile.West;
			const double SpanY = Tile.North - Tile.South;
			if (FMath::Abs(SpanX) < 1.0e-18 || FMath::Abs(SpanY) < 1.0e-18)
			{
				return false;
			}
			const double Pu = (Lon - Tile.West) / SpanX;
			const double Pv = (Lat - Tile.South) / SpanY;
			double BestD = TNumericLimits<double>::Max();
			double BestH = 0.0;
			bool bInside = false;
			for (int32 T = 0; T + 2 < Tile.Indices.Num(); T += 3)
			{
				const int32 A = Tile.Indices[T];
				const int32 B = Tile.Indices[T + 1];
				const int32 C = Tile.Indices[T + 2];
				if (!Tile.U.IsValidIndex(A) || !Tile.U.IsValidIndex(B) || !Tile.U.IsValidIndex(C))
				{
					continue;
				}
				double W0, W1, W2;
				if (PointInTri(Pu, Pv, Tile.U[A], Tile.V[A], Tile.U[B], Tile.V[B], Tile.U[C], Tile.V[C], W0, W1, W2))
				{
					OutHeightM = W0 * Tile.HeightM[A] + W1 * Tile.HeightM[B] + W2 * Tile.HeightM[C];
					return true;
				}
				const double Mu = (Tile.U[A] + Tile.U[B] + Tile.U[C]) / 3.0;
				const double Mv = (Tile.V[A] + Tile.V[B] + Tile.V[C]) / 3.0;
				const double D = FMath::Square(Pu - Mu) + FMath::Square(Pv - Mv);
				if (D < BestD)
				{
					BestD = D;
					BestH = (Tile.HeightM[A] + Tile.HeightM[B] + Tile.HeightM[C]) / 3.0;
					bInside = true;
				}
			}
			if (bInside && BestD < 0.05)
			{
				OutHeightM = BestH;
				return true;
			}
			return false;
		}

		bool TryParseTileIdFromPath(const FString& Path, FQmTileId& OutId)
		{
			FString Norm = Path;
			Norm.ReplaceInline(TEXT("\\"), TEXT("/"));
			TArray<FString> Parts;
			Norm.ParseIntoArray(Parts, TEXT("/"), true);
			if (Parts.Num() < 3)
			{
				return false;
			}
			const FString File = FPaths::GetBaseFilename(Parts.Last());
			const FString XStr = Parts[Parts.Num() - 2];
			const FString ZStr = Parts[Parts.Num() - 3];
			if (!ZStr.IsNumeric() || !XStr.IsNumeric() || !File.IsNumeric())
			{
				return false;
			}
			OutId.Z = FCString::Atoi(*ZStr);
			OutId.X = FCString::Atoi(*XStr);
			OutId.Y = FCString::Atoi(*File);
			return OutId.Z >= 0 && OutId.Z <= 30;
		}

		bool LoadAsciiGrid(const FString& Path, FRaster& Out, FString& OutError)
		{
			TArray<FString> Lines;
			if (!FFileHelper::LoadFileToStringArray(Lines, *Path) || Lines.Num() < 7)
			{
				OutError = FString::Printf(TEXT("Could not read ASCII grid '%s'."), *Path);
				return false;
			}

			TMap<FString, FString> Header;
			int32 DataStart = 0;
			for (int32 I = 0; I < Lines.Num() && I < 12; ++I)
			{
				FString Key, Val;
				if (Lines[I].Split(TEXT(" "), &Key, &Val))
				{
					Key.TrimStartAndEndInline();
					Val.TrimStartAndEndInline();
					Header.Add(Key.ToLower(), Val);
					DataStart = I + 1;
				}
				else
				{
					break;
				}
			}

			Out.Width = FCString::Atoi(*Header.FindRef(TEXT("ncols")));
			Out.Height = FCString::Atoi(*Header.FindRef(TEXT("nrows")));
			Out.PixelW = FCString::Atod(*Header.FindRef(TEXT("cellsize")));
			Out.PixelH = -Out.PixelW;
			Out.NoData = Header.Contains(TEXT("nodata_value"))
				? FCString::Atof(*Header.FindRef(TEXT("nodata_value")))
				: -9999.0f;
			if (Header.Contains(TEXT("xllcorner")))
			{
				Out.OriginX = FCString::Atod(*Header.FindRef(TEXT("xllcorner")));
				Out.OriginY = FCString::Atod(*Header.FindRef(TEXT("yllcorner")))
					+ static_cast<double>(Out.Height) * Out.PixelW;
			}
			else
			{
				Out.OriginX = FCString::Atod(*Header.FindRef(TEXT("xllcenter"))) - Out.PixelW * 0.5;
				Out.OriginY = FCString::Atod(*Header.FindRef(TEXT("yllcenter")))
					+ (static_cast<double>(Out.Height) - 0.5) * Out.PixelW;
			}
			if (Out.Width <= 0 || Out.Height <= 0)
			{
				OutError = FString::Printf(TEXT("Invalid ASCII grid size in '%s'."), *Path);
				return false;
			}

			Out.bGeographic = FMath::Abs(Out.OriginX) <= 180.0 && FMath::Abs(Out.OriginY) <= 90.0;
			const FString Prj = FPaths::ChangeExtension(Path, TEXT("prj"));
			if (!Out.bGeographic)
			{
				ParsePrjUtm(Prj, Out.UtmZone, Out.bUtmNorth);
			}

			Out.Heights.SetNumUninitialized(Out.Width * Out.Height);
			int32 Count = 0;
			for (int32 I = DataStart; I < Lines.Num(); ++I)
			{
				TArray<FString> Vals;
				Lines[I].ParseIntoArrayWS(Vals);
				for (const FString& V : Vals)
				{
					if (Count >= Out.Heights.Num())
					{
						break;
					}
					Out.Heights[Count++] = FCString::Atof(*V);
				}
			}
			if (Count < Out.Heights.Num())
			{
				OutError = FString::Printf(TEXT("ASCII grid '%s' is truncated."), *Path);
				return false;
			}
			return true;
		}

		struct FTiffReader
		{
			TArray<uint8> Bytes;
			bool bBig = false;

			template <typename T>
			T Rd(int64 Off) const
			{
				if (Off < 0 || Off + static_cast<int64>(sizeof(T)) > Bytes.Num())
				{
					return T{};
				}
				T V;
				FMemory::Memcpy(&V, Bytes.GetData() + Off, sizeof(T));
				return V;
			}

			uint16 U16(int64 Off) const { return Rd<uint16>(Off); }
			uint32 U32(int64 Off) const { return Rd<uint32>(Off); }
			uint64 U64(int64 Off) const { return Rd<uint64>(Off); }
		};

		bool LoadGeoTiff(const FString& Path, FRaster& Out, FString& OutError)
		{
			FTiffReader T;
			if (!FFileHelper::LoadFileToArray(T.Bytes, *Path) || T.Bytes.Num() < 16)
			{
				OutError = FString::Printf(TEXT("Could not read GeoTIFF '%s'."), *Path);
				return false;
			}
			if (T.Bytes[0] == 'M' && T.Bytes[1] == 'M')
			{
				OutError = FString::Printf(TEXT("Big-endian GeoTIFF is not supported: '%s'."), *Path);
				return false;
			}
			const uint16 Magic = T.U16(2);
			T.bBig = (Magic == 43);
			if (Magic != 42 && Magic != 43)
			{
				OutError = FString::Printf(TEXT("Not a TIFF: '%s'."), *Path);
				return false;
			}

			const int32 EntrySize = T.bBig ? 20 : 12;
			int64 Ifd = T.bBig ? static_cast<int64>(T.U64(8)) : static_cast<int64>(T.U32(4));
			const int64 NumEnt = T.bBig ? static_cast<int64>(T.U64(Ifd)) : static_cast<int64>(T.U16(Ifd));
			Ifd += T.bBig ? 8 : 2;

			auto Entry = [&](int32 I, uint16& Tag, uint16& Type, uint64& Count, uint64& Val)
			{
				const int64 E = Ifd + static_cast<int64>(I) * EntrySize;
				Tag = T.U16(E);
				Type = T.U16(E + 2);
				if (T.bBig)
				{
					Count = T.U64(E + 4);
					Val = T.U64(E + 12);
				}
				else
				{
					Count = T.U32(E + 4);
					Val = T.U32(E + 8);
				}
			};

			auto TypeSize = [](uint16 Type) -> int32
			{
				switch (Type)
				{
				case 1: case 2: case 6: case 7: return 1;
				case 3: case 8: return 2;
				case 4: case 9: case 11: return 4;
				case 5: case 10: case 12: return 8;
				case 16: case 17: return 8;
				default: return 1;
				}
			};

			auto PayloadOff = [&](uint16 Type, uint64 Count, uint64 Val) -> int64
			{
				const int64 BytesN = static_cast<int64>(TypeSize(Type)) * static_cast<int64>(Count);
				const int32 Inline = T.bBig ? 8 : 4;
				return BytesN <= Inline ? -1 : static_cast<int64>(Val);
			};

			auto ReadU32s = [&](uint16 Type, uint64 Count, uint64 Val, TArray<uint64>& OutA)
			{
				OutA.Reset();
				OutA.Reserve(static_cast<int32>(Count));
				const int64 Off = PayloadOff(Type, Count, Val);
				for (uint64 I = 0; I < Count; ++I)
				{
					if (Off < 0)
					{
						OutA.Add(Val);
						break;
					}
					const int32 Sz = TypeSize(Type);
					if (Sz == 2)
					{
						OutA.Add(T.U16(Off + static_cast<int64>(I) * 2));
					}
					else if (Sz == 8)
					{
						OutA.Add(T.U64(Off + static_cast<int64>(I) * 8));
					}
					else
					{
						OutA.Add(T.U32(Off + static_cast<int64>(I) * 4));
					}
				}
			};

			auto ReadDoubles = [&](uint16 Type, uint64 Count, uint64 Val, TArray<double>& OutA)
			{
				OutA.Reset();
				int64 Off = PayloadOff(Type, Count, Val);
				if (Off < 0)
				{
					Off = static_cast<int64>(Val);
				}
				for (uint64 I = 0; I < Count; ++I)
				{
					if (Type == 12)
					{
						double D = 0.0;
						if (Off + static_cast<int64>(I) * 8 + 8 <= T.Bytes.Num())
						{
							FMemory::Memcpy(&D, T.Bytes.GetData() + Off + static_cast<int64>(I) * 8, 8);
						}
						OutA.Add(D);
					}
					else if (Type == 5)
					{
						const int64 P = Off + static_cast<int64>(I) * 8;
						const uint32 N = T.U32(P);
						const uint32 Den = T.U32(P + 4);
						OutA.Add(Den ? static_cast<double>(N) / static_cast<double>(Den) : 0.0);
					}
				}
			};

			uint32 Width = 0, Height = 0, Compression = 1, Samples = 1, Predictor = 1, SampleFormat = 1;
			uint16 Bits = 16;
			uint32 TileW = 0, TileH = 0, RowsPerStrip = 0;
			TArray<uint64> StripOff, StripBytes, TileOff, TileBytes;
			TArray<double> Scales, Tiepoints, Transform;
			TArray<uint16> GeoKeys;

			for (int32 I = 0; I < NumEnt; ++I)
			{
				uint16 Tag = 0, Type = 0;
				uint64 Count = 0, Val = 0;
				Entry(I, Tag, Type, Count, Val);
				switch (Tag)
				{
				case 256: Width = static_cast<uint32>(Val); break;
				case 257: Height = static_cast<uint32>(Val); break;
				case 258: Bits = static_cast<uint16>(Val); break;
				case 259: Compression = static_cast<uint32>(Val); break;
				case 277: Samples = static_cast<uint32>(Val); break;
				case 278: RowsPerStrip = static_cast<uint32>(Val); break;
				case 273: ReadU32s(Type, Count, Val, StripOff); break;
				case 279: ReadU32s(Type, Count, Val, StripBytes); break;
				case 322: TileW = static_cast<uint32>(Val); break;
				case 323: TileH = static_cast<uint32>(Val); break;
				case 324: ReadU32s(Type, Count, Val, TileOff); break;
				case 325: ReadU32s(Type, Count, Val, TileBytes); break;
				case 317: Predictor = static_cast<uint32>(Val); break;
				case 339: SampleFormat = static_cast<uint32>(Val); break;
				case 33550: ReadDoubles(Type, Count, Val, Scales); break;
				case 33922: ReadDoubles(Type, Count, Val, Tiepoints); break;
				case 34264: ReadDoubles(Type, Count, Val, Transform); break;
				case 34735:
				{
					const int64 Off = PayloadOff(Type, Count, Val);
					if (Off >= 0)
					{
						for (uint64 K = 0; K + 3 < Count; K += 4)
						{
							GeoKeys.Add(T.U16(Off + static_cast<int64>(K) * 2));
							GeoKeys.Add(T.U16(Off + static_cast<int64>(K) * 2 + 2));
							GeoKeys.Add(T.U16(Off + static_cast<int64>(K) * 2 + 4));
							GeoKeys.Add(T.U16(Off + static_cast<int64>(K) * 2 + 6));
						}
					}
					break;
				}
				default:
					break;
				}
			}

			if (Width == 0 || Height == 0 || Width > 200000 || Height > 200000)
			{
				OutError = FString::Printf(TEXT("GeoTIFF '%s' has unsupported size."), *Path);
				return false;
			}
			if (Samples != 1 || (Bits != 16 && Bits != 32 && Bits != 64) || (Compression != 1 && Compression != 8 && Compression != 32946))
			{
				OutError = FString::Printf(
					TEXT("GeoTIFF '%s' must be single-band 16/32/64-bit uncompressed or Deflate."),
					*Path);
				return false;
			}

			if (Scales.Num() >= 2 && Tiepoints.Num() >= 6)
			{
				Out.PixelW = Scales[0];
				Out.PixelH = -FMath::Abs(Scales[1]);
				Out.OriginX = Tiepoints[3] - Tiepoints[0] * Out.PixelW;
				Out.OriginY = Tiepoints[4] - Tiepoints[1] * Out.PixelH;
			}
			else if (Transform.Num() >= 6)
			{
				Out.OriginX = Transform[3];
				Out.OriginY = Transform[5];
				Out.PixelW = Transform[0];
				Out.PixelH = Transform[4];
			}
			else
			{
				const FString Tfw = FPaths::ChangeExtension(Path, TEXT("tfw"));
				TArray<FString> W;
				if (!FFileHelper::LoadFileToStringArray(W, *Tfw) || W.Num() < 6)
				{
					OutError = FString::Printf(TEXT("GeoTIFF '%s' has no georeference (tags or .tfw)."), *Path);
					return false;
				}
				Out.PixelW = FCString::Atod(*W[0]);
				Out.PixelH = FCString::Atod(*W[3]);
				Out.OriginX = FCString::Atod(*W[4]) - Out.PixelW * 0.5;
				Out.OriginY = FCString::Atod(*W[5]) - Out.PixelH * 0.5;
			}

			Out.Width = static_cast<int32>(Width);
			Out.Height = static_cast<int32>(Height);
			Out.bGeographic = FMath::Abs(Out.OriginX) <= 180.0 && FMath::Abs(Out.OriginY) <= 90.0;
			Out.NoData = -9999.0f;

			int32 Epsg = 0;
			for (int32 K = 4; K + 3 < GeoKeys.Num(); K += 4)
			{
				const uint16 Key = GeoKeys[K];
				const uint16 Value = GeoKeys[K + 3];
				if (Key == 2048 || Key == 3072)
				{
					Epsg = Value;
				}
			}
			if (Epsg == 4326)
			{
				Out.bGeographic = true;
			}
			else
			{
				const int32 Zone = UtmZoneFromEpsg(Epsg, Out.bUtmNorth);
				if (Zone > 0)
				{
					Out.bGeographic = false;
					Out.UtmZone = Zone;
				}
			}
			if (!Out.bGeographic && Out.UtmZone == 0)
			{
				ParsePrjUtm(FPaths::ChangeExtension(Path, TEXT("prj")), Out.UtmZone, Out.bUtmNorth);
			}

			const int32 BytesPerSample = Bits / 8;
			Out.Heights.SetNumZeroed(Out.Width * Out.Height);

			auto StoreRow = [&](int32 Y, const TArray<uint8>& RowBytes)
			{
				if (Y < 0 || Y >= Out.Height)
				{
					return;
				}
				TArray<uint8> Work = RowBytes;
				if (Predictor == 2 && Work.Num() >= BytesPerSample * Out.Width)
				{
					for (int32 X = 1; X < Out.Width; ++X)
					{
						for (int32 B = 0; B < BytesPerSample; ++B)
						{
							Work[X * BytesPerSample + B] += Work[(X - 1) * BytesPerSample + B];
						}
					}
				}
				for (int32 X = 0; X < Out.Width; ++X)
				{
					const int32 Off = X * BytesPerSample;
					if (Off + BytesPerSample > Work.Num())
					{
						break;
					}
					float H = 0.0f;
					if (Bits == 16 && SampleFormat == 2)
					{
						int16 S;
						FMemory::Memcpy(&S, Work.GetData() + Off, 2);
						H = static_cast<float>(S);
					}
					else if (Bits == 16)
					{
						uint16 S;
						FMemory::Memcpy(&S, Work.GetData() + Off, 2);
						H = static_cast<float>(S);
					}
					else if (Bits == 32 && SampleFormat == 3)
					{
						FMemory::Memcpy(&H, Work.GetData() + Off, 4);
					}
					else if (Bits == 32 && SampleFormat == 2)
					{
						int32 S;
						FMemory::Memcpy(&S, Work.GetData() + Off, 4);
						H = static_cast<float>(S);
					}
					else if (Bits == 64 && SampleFormat == 3)
					{
						double D;
						FMemory::Memcpy(&D, Work.GetData() + Off, 8);
						H = static_cast<float>(D);
					}
					else
					{
						uint32 S;
						FMemory::Memcpy(&S, Work.GetData() + Off, FMath::Min(BytesPerSample, 4));
						H = static_cast<float>(S);
					}
					Out.Heights[Y * Out.Width + X] = H;
				}
			};

			auto DecodeBlock = [&](int64 Off, int64 Len, int32 Hint, TArray<uint8>& Decoded) -> bool
			{
				if (Off < 0 || Off + Len > T.Bytes.Num())
				{
					return false;
				}
				if (Compression == 1)
				{
					Decoded.SetNumUninitialized(static_cast<int32>(Len));
					FMemory::Memcpy(Decoded.GetData(), T.Bytes.GetData() + Off, static_cast<SIZE_T>(Len));
					return true;
				}
				return InflateZlib(T.Bytes.GetData() + Off, static_cast<int32>(Len), Decoded, Hint);
			};

			if (TileW > 0 && TileH > 0 && TileOff.Num() > 0)
			{
				const int32 TilesX = (Out.Width + static_cast<int32>(TileW) - 1) / static_cast<int32>(TileW);
				for (int32 Ti = 0; Ti < TileOff.Num(); ++Ti)
				{
					TArray<uint8> Dec;
					const int64 Len = TileBytes.IsValidIndex(Ti) ? static_cast<int64>(TileBytes[Ti]) : 0;
					if (!DecodeBlock(static_cast<int64>(TileOff[Ti]), Len, static_cast<int32>(TileW * TileH * BytesPerSample), Dec))
					{
						continue;
					}
					const int32 TileX = (Ti % TilesX) * static_cast<int32>(TileW);
					const int32 TileY = (Ti / TilesX) * static_cast<int32>(TileH);
					for (int32 Ry = 0; Ry < static_cast<int32>(TileH); ++Ry)
					{
						const int32 Y = TileY + Ry;
						if (Y >= Out.Height)
						{
							break;
						}
						const int32 SrcOff = Ry * static_cast<int32>(TileW) * BytesPerSample;
						const int32 CopyW = FMath::Min(static_cast<int32>(TileW), Out.Width - TileX);
						TArray<uint8> TileRow;
						TileRow.SetNumUninitialized(static_cast<int32>(TileW) * BytesPerSample);
						if (SrcOff + TileRow.Num() <= Dec.Num())
						{
							FMemory::Memcpy(TileRow.GetData(), Dec.GetData() + SrcOff, TileRow.Num());
						}
						else
						{
							continue;
						}
						if (Predictor == 2)
						{
							for (int32 X = 1; X < static_cast<int32>(TileW); ++X)
							{
								for (int32 B = 0; B < BytesPerSample; ++B)
								{
									TileRow[X * BytesPerSample + B] += TileRow[(X - 1) * BytesPerSample + B];
								}
							}
						}
						for (int32 X = 0; X < CopyW; ++X)
						{
							TArray<uint8> DummyRow;
							DummyRow.SetNumZeroed(Out.Width * BytesPerSample);
							FMemory::Memcpy(
								DummyRow.GetData() + (TileX + X) * BytesPerSample,
								TileRow.GetData() + X * BytesPerSample,
								BytesPerSample);
							// Fall through to pixel decode for this single pixel via StoreRow would overwrite the row.
							const int32 Off = X * BytesPerSample;
							float H = 0.0f;
							if (Bits == 16 && SampleFormat == 2)
							{
								int16 S;
								FMemory::Memcpy(&S, TileRow.GetData() + Off, 2);
								H = static_cast<float>(S);
							}
							else if (Bits == 16)
							{
								uint16 S;
								FMemory::Memcpy(&S, TileRow.GetData() + Off, 2);
								H = static_cast<float>(S);
							}
							else if (Bits == 32 && SampleFormat == 3)
							{
								FMemory::Memcpy(&H, TileRow.GetData() + Off, 4);
							}
							else if (Bits == 32 && SampleFormat == 2)
							{
								int32 S;
								FMemory::Memcpy(&S, TileRow.GetData() + Off, 4);
								H = static_cast<float>(S);
							}
							else if (Bits == 64 && SampleFormat == 3)
							{
								double D;
								FMemory::Memcpy(&D, TileRow.GetData() + Off, 8);
								H = static_cast<float>(D);
							}
							Out.Heights[Y * Out.Width + (TileX + X)] = H;
						}
					}
				}
			}
			else
			{
				if (RowsPerStrip == 0)
				{
					RowsPerStrip = Height;
				}
				for (int32 S = 0; S < StripOff.Num(); ++S)
				{
					TArray<uint8> Dec;
					const int64 Len = StripBytes.IsValidIndex(S) ? static_cast<int64>(StripBytes[S]) : 0;
					if (!DecodeBlock(static_cast<int64>(StripOff[S]), Len, Out.Width * static_cast<int32>(RowsPerStrip) * BytesPerSample, Dec))
					{
						continue;
					}
					const int32 Y0 = S * static_cast<int32>(RowsPerStrip);
					const int32 RowBytes = Out.Width * BytesPerSample;
					const int32 Rows = FMath::Min(static_cast<int32>(RowsPerStrip), Out.Height - Y0);
					for (int32 R = 0; R < Rows; ++R)
					{
						TArray<uint8> Row;
						Row.SetNumUninitialized(RowBytes);
						if ((R + 1) * RowBytes <= Dec.Num())
						{
							FMemory::Memcpy(Row.GetData(), Dec.GetData() + R * RowBytes, RowBytes);
							StoreRow(Y0 + R, Row);
						}
					}
				}
			}

			return Out.Heights.Num() == Out.Width * Out.Height;
		}

		bool RasterContains(const FRaster& R, double X, double Y)
		{
			const double MaxX = R.OriginX + R.PixelW * R.Width;
			const double MaxY = R.OriginY + R.PixelH * R.Height;
			const double MinX = FMath::Min(R.OriginX, MaxX);
			const double MaxXe = FMath::Max(R.OriginX, MaxX);
			const double MinY = FMath::Min(R.OriginY, MaxY);
			const double MaxYe = FMath::Max(R.OriginY, MaxY);
			return X >= MinX && X <= MaxXe && Y >= MinY && Y <= MaxYe;
		}

		bool SampleRaster(const FRaster& R, double Lon, double Lat, double& OutHeightM)
		{
			double X = Lon;
			double Y = Lat;
			if (!R.bGeographic)
			{
				if (R.UtmZone <= 0 || !LonLatToUtm(Lon, Lat, R.UtmZone, R.bUtmNorth, X, Y))
				{
					return false;
				}
			}
			if (!RasterContains(R, X, Y))
			{
				return false;
			}
			const double Col = (X - R.OriginX) / R.PixelW - 0.5;
			const double Row = (Y - R.OriginY) / R.PixelH - 0.5;
			const int32 C0 = FMath::Clamp(FMath::FloorToInt(Col), 0, R.Width - 1);
			const int32 R0 = FMath::Clamp(FMath::FloorToInt(Row), 0, R.Height - 1);
			const int32 C1 = FMath::Min(C0 + 1, R.Width - 1);
			const int32 R1 = FMath::Min(R0 + 1, R.Height - 1);
			const double Tx = FMath::Clamp(Col - C0, 0.0, 1.0);
			const double Ty = FMath::Clamp(Row - R0, 0.0, 1.0);
			auto At = [&](int32 RR, int32 CC) -> float
			{
				return R.Heights[RR * R.Width + CC];
			};
			const float A = At(R0, C0);
			const float B = At(R0, C1);
			const float C = At(R1, C0);
			const float D = At(R1, C1);
			auto Valid = [&](float H)
			{
				return H != R.NoData && FMath::IsFinite(H);
			};
			if (!Valid(A) || !Valid(B) || !Valid(C) || !Valid(D))
			{
				if (Valid(A))
				{
					OutHeightM = A;
					return true;
				}
				return false;
			}
			const double Top = FMath::Lerp(static_cast<double>(A), static_cast<double>(B), Tx);
			const double Bot = FMath::Lerp(static_cast<double>(C), static_cast<double>(D), Tx);
			OutHeightM = FMath::Lerp(Top, Bot, Ty);
			return true;
		}

		TMap<FQmTileId, FString> GQmFiles;
		TMap<FQmTileId, TSharedPtr<FQmDecoded>> GQmCache;
		TArray<FQmTileId> GQmLru;
		TArray<FRaster> GRasters;
		bool GQmTms = true;
		int32 GQmMaxZ = -1;
		FString GDescription;
		bool GHasQm = false;
		bool GHasRaster = false;

		void CacheQm(const FQmTileId& Id, const TSharedPtr<FQmDecoded>& Tile)
		{
			GQmCache.Add(Id, Tile);
			GQmLru.Remove(Id);
			GQmLru.Add(Id);
			while (GQmLru.Num() > QmCacheSize)
			{
				GQmCache.Remove(GQmLru[0]);
				GQmLru.RemoveAt(0);
			}
		}

		TSharedPtr<FQmDecoded> GetQmTile(const FQmTileId& Id)
		{
			if (TSharedPtr<FQmDecoded>* Found = GQmCache.Find(Id))
			{
				GQmLru.Remove(Id);
				GQmLru.Add(Id);
				return *Found;
			}
			const FString* Path = GQmFiles.Find(Id);
			if (!Path)
			{
				return nullptr;
			}
			TArray<uint8> Raw;
			if (!FFileHelper::LoadFileToArray(Raw, **Path))
			{
				return nullptr;
			}
			TSharedPtr<FQmDecoded> Dec = MakeShared<FQmDecoded>();
			if (!DecodeQuantizedMesh(Raw, Id.Z, Id.X, Id.Y, GQmTms, *Dec))
			{
				return nullptr;
			}
			CacheQm(Id, Dec);
			return Dec;
		}
}

namespace WaterElevation
{
	bool FSampler::Load(const FString& FolderOrFile, FString& OutError)
	{
		bLoaded = false;
		GQmFiles.Reset();
		GQmCache.Reset();
		GQmLru.Reset();
		GRasters.Reset();
		GHasQm = false;
		GHasRaster = false;
		GQmMaxZ = -1;
		GQmTms = true;
		SourceDescription.Reset();

		FString Path = FolderOrFile;
		Path.TrimStartAndEndInline();
		while ((Path.StartsWith(TEXT("\"")) && Path.EndsWith(TEXT("\"")) && Path.Len() >= 2)
			|| (Path.StartsWith(TEXT("'")) && Path.EndsWith(TEXT("'")) && Path.Len() >= 2))
		{
			Path = Path.Mid(1, Path.Len() - 2).TrimStartAndEnd();
		}
		if (Path.IsEmpty())
		{
			OutError = TEXT("Elevation folder path is empty.");
			return false;
		}

		IFileManager& FM = IFileManager::Get();
		TArray<FString> TerrainFiles;
		TArray<FString> RasterFiles;
		if (FM.DirectoryExists(*Path))
		{
			FM.FindFilesRecursive(TerrainFiles, *Path, TEXT("*.terrain"), true, false);
			TArray<FString> Tif, Tiff, Asc;
			FM.FindFilesRecursive(Tif, *Path, TEXT("*.tif"), true, false);
			FM.FindFilesRecursive(Tiff, *Path, TEXT("*.tiff"), true, false);
			FM.FindFilesRecursive(Asc, *Path, TEXT("*.asc"), true, false);
			RasterFiles = Tif;
			RasterFiles.Append(Tiff);
			RasterFiles.Append(Asc);
			const FString Layer = FPaths::Combine(Path, TEXT("layer.json"));
			if (FM.FileExists(*Layer))
			{
				FString Json;
				if (FFileHelper::LoadFileToString(Json, *Layer))
				{
					TSharedPtr<FJsonObject> Obj;
					const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
					if (FJsonSerializer::Deserialize(Reader, Obj) && Obj.IsValid())
					{
						FString Scheme;
						if (Obj->TryGetStringField(TEXT("scheme"), Scheme))
						{
							GQmTms = Scheme.Equals(TEXT("tms"), ESearchCase::IgnoreCase);
						}
					}
				}
			}
		}
		else if (FM.FileExists(*Path))
		{
			const FString Ext = FPaths::GetExtension(Path, false).ToLower();
			if (Ext == TEXT("terrain"))
			{
				TerrainFiles.Add(Path);
			}
			else
			{
				RasterFiles.Add(Path);
			}
		}
		else
		{
			OutError = FString::Printf(TEXT("Elevation path not found: %s"), *Path);
			return false;
		}

		FString Dummy;
		if (TerrainFiles.Num() > 0)
		{
			LoadQuantizedMeshFolder(Path, TerrainFiles, Dummy);
		}
		if (RasterFiles.Num() > 0)
		{
			LoadRasters(RasterFiles, Dummy);
		}

		if (!GHasQm && !GHasRaster)
		{
			OutError = FString::Printf(
				TEXT("No readable elevation tiles in '%s'. Expected .terrain (Cesium quantized-mesh), .tif GeoTIFF, or .asc."),
				*Path);
			return false;
		}

		bLoaded = true;
		SourceDescription = FString::Printf(
			TEXT("%s (%d quantized-mesh tile(s) maxZ=%d, %d raster(s))"),
			*Path,
			GQmFiles.Num(),
			GQmMaxZ,
			GRasters.Num());
		UE_LOG(LogWaterPlacer, Display, TEXT("Loaded local elevation: %s"), *SourceDescription);
		return true;
	}

	bool FSampler::LoadQuantizedMeshFolder(const FString& Folder, const TArray<FString>& TerrainFiles, FString& OutError)
	{
		(void)Folder;
		(void)OutError;
		for (const FString& File : TerrainFiles)
		{
			FQmTileId Id;
			if (!TryParseTileIdFromPath(File, Id))
			{
				continue;
			}
			GQmFiles.Add(Id, File);
			GQmMaxZ = FMath::Max(GQmMaxZ, Id.Z);
		}
		GHasQm = GQmFiles.Num() > 0;
		return GHasQm;
	}

	bool FSampler::LoadRasters(const TArray<FString>& RasterFiles, FString& OutError)
	{
		(void)OutError;
		for (const FString& File : RasterFiles)
		{
			FRaster R;
			FString Err;
			const FString Ext = FPaths::GetExtension(File, false).ToLower();
			const bool bOk = (Ext == TEXT("asc")) ? LoadAsciiGrid(File, R, Err) : LoadGeoTiff(File, R, Err);
			if (bOk)
			{
				GRasters.Add(MoveTemp(R));
			}
			else
			{
				UE_LOG(LogWaterPlacer, Warning, TEXT("%s"), *Err);
			}
		}
		GHasRaster = GRasters.Num() > 0;
		return GHasRaster;
	}

	bool FSampler::SampleQuantizedMesh(double LonDeg, double LatDeg, double& OutHeightM) const
	{
		if (!GHasQm || GQmMaxZ < 0)
		{
			return false;
		}
		for (int32 Z = GQmMaxZ; Z >= 0; --Z)
		{
			int32 X = 0, Y = 0;
			if (GQmTms)
			{
				LonLatToTileTms(Z, LonDeg, LatDeg, X, Y);
			}
			else
			{
				LonLatToTileCesium(Z, LonDeg, LatDeg, X, Y);
			}
			const FQmTileId Id{ Z, X, Y };
			if (!GQmFiles.Contains(Id))
			{
				continue;
			}
			TSharedPtr<FQmDecoded> Tile = GetQmTile(Id);
			if (Tile.IsValid() && SampleDecodedQm(*Tile, LonDeg, LatDeg, OutHeightM))
			{
				return true;
			}
		}
		return false;
	}

	bool FSampler::SampleRasters(double LonDeg, double LatDeg, double& OutHeightM) const
	{
		for (const FRaster& R : GRasters)
		{
			if (SampleRaster(R, LonDeg, LatDeg, OutHeightM))
			{
				return true;
			}
		}
		return false;
	}

	bool FSampler::SampleHeightM(double LonDeg, double LatDeg, double& OutHeightM) const
	{
		if (!bLoaded)
		{
			return false;
		}
		if (GHasQm && SampleQuantizedMesh(LonDeg, LatDeg, OutHeightM))
		{
			return true;
		}
		if (GHasRaster && SampleRasters(LonDeg, LatDeg, OutHeightM))
		{
			return true;
		}
		return false;
	}

	FString FSampler::Describe() const
	{
		return SourceDescription;
	}
}
