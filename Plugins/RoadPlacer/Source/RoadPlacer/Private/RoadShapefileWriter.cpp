#include "RoadPlacerPrivatePCH.h"
#include "RoadShapefileWriter.h"
#include "RoadPlacerLog.h"

#include "Algo/Reverse.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
	FString NormalizeShpPath(const FString& InPath)
	{
		FString Path = InPath.TrimStartAndEnd();
		while ((Path.StartsWith(TEXT("\"")) && Path.EndsWith(TEXT("\"")) && Path.Len() >= 2)
			|| (Path.StartsWith(TEXT("'")) && Path.EndsWith(TEXT("'")) && Path.Len() >= 2))
		{
			Path = Path.Mid(1, Path.Len() - 2).TrimStartAndEnd();
		}
		const FString Ext = FPaths::GetExtension(Path);
		if (Ext.Equals(TEXT("shp"), ESearchCase::IgnoreCase)
			|| Ext.Equals(TEXT("shx"), ESearchCase::IgnoreCase)
			|| Ext.Equals(TEXT("dbf"), ESearchCase::IgnoreCase)
			|| Ext.Equals(TEXT("prj"), ESearchCase::IgnoreCase))
		{
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path));
		}
		return Path;
	}

	void AppendInt32LE(TArray<uint8>& Out, int32 Value)
	{
		const uint32 U = static_cast<uint32>(Value);
		Out.Add(static_cast<uint8>(U));
		Out.Add(static_cast<uint8>(U >> 8));
		Out.Add(static_cast<uint8>(U >> 16));
		Out.Add(static_cast<uint8>(U >> 24));
	}

	void AppendInt32BE(TArray<uint8>& Out, int32 Value)
	{
		const uint32 U = static_cast<uint32>(Value);
		Out.Add(static_cast<uint8>(U >> 24));
		Out.Add(static_cast<uint8>(U >> 16));
		Out.Add(static_cast<uint8>(U >> 8));
		Out.Add(static_cast<uint8>(U));
	}

	void AppendDoubleLE(TArray<uint8>& Out, double Value)
	{
		uint64 Bits = 0;
		FMemory::Memcpy(&Bits, &Value, sizeof(double));
		for (int32 I = 0; I < 8; ++I)
		{
			Out.Add(static_cast<uint8>(Bits >> (8 * I)));
		}
	}

	void PatchInt32BE(TArray<uint8>& Out, int32 Offset, int32 Value)
	{
		const uint32 U = static_cast<uint32>(Value);
		Out[Offset] = static_cast<uint8>(U >> 24);
		Out[Offset + 1] = static_cast<uint8>(U >> 16);
		Out[Offset + 2] = static_cast<uint8>(U >> 8);
		Out[Offset + 3] = static_cast<uint8>(U);
	}

	void WriteShpHeader(TArray<uint8>& Out, int32 ShapeType)
	{
		Out.Reserve(100);
		AppendInt32BE(Out, 9994);
		for (int32 I = 0; I < 5; ++I)
		{
			AppendInt32BE(Out, 0);
		}
		AppendInt32BE(Out, 50);
		AppendInt32LE(Out, 1000);
		AppendInt32LE(Out, ShapeType);
		for (int32 I = 0; I < 8; ++I)
		{
			AppendDoubleLE(Out, 0.0);
		}
	}

	void PatchBoundingBox(
		TArray<uint8>& Out,
		double XMin,
		double YMin,
		double XMax,
		double YMax,
		double ZMin,
		double ZMax)
	{
		auto PatchDouble = [&](int32 Offset, double Value)
		{
			uint64 Bits = 0;
			FMemory::Memcpy(&Bits, &Value, sizeof(double));
			for (int32 I = 0; I < 8; ++I)
			{
				Out[Offset + I] = static_cast<uint8>(Bits >> (8 * I));
			}
		};
		PatchDouble(36, XMin);
		PatchDouble(44, YMin);
		PatchDouble(52, XMax);
		PatchDouble(60, YMax);
		PatchDouble(68, ZMin);
		PatchDouble(76, ZMax);
		PatchDouble(84, 0.0);
		PatchDouble(92, 0.0);
	}

	double SignedArea(const TArray<FVector2D>& Ring)
	{
		double A = 0.0;
		const int32 N = Ring.Num();
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D& P = Ring[I];
			const FVector2D& Q = Ring[(I + 1) % N];
			A += P.X * Q.Y - Q.X * P.Y;
		}
		return A;
	}

	void CloseClockwise(FRoadShapefileRing& Ring)
	{
		if (Ring.LonLat.Num() < 3)
		{
			return;
		}
		if (SignedArea(Ring.LonLat) > 0.0)
		{
			Algo::Reverse(Ring.LonLat);
			if (Ring.HeightM.Num() == Ring.LonLat.Num())
			{
				Algo::Reverse(Ring.HeightM);
			}
		}
		if (!Ring.LonLat[0].Equals(Ring.LonLat.Last(), 1.0e-12))
		{
			const FVector2D FirstLonLat = Ring.LonLat[0];
			const double FirstZ = Ring.HeightM.Num() > 0 ? Ring.HeightM[0] : 0.0;
			const bool bCopyZ = Ring.HeightM.Num() == Ring.LonLat.Num();
			Ring.LonLat.Add(FirstLonLat);
			if (bCopyZ)
			{
				Ring.HeightM.Add(FirstZ);
			}
			else if (Ring.HeightM.Num() != Ring.LonLat.Num())
			{
				Ring.HeightM.SetNum(Ring.LonLat.Num());
			}
		}
	}

	bool WriteDbf(const FString& DbfPath, int32 NumRecords, FString& OutError)
	{
		const int32 FieldLen = 10;
		const int32 RecordLen = 1 + FieldLen;
		const int32 HeaderLen = 32 + 32 + 1;
		TArray<uint8> Data;
		Data.Add(0x03);
		Data.Add(25);
		Data.Add(1);
		Data.Add(1);
		AppendInt32LE(Data, NumRecords);
		Data.Add(static_cast<uint8>(HeaderLen));
		Data.Add(static_cast<uint8>(HeaderLen >> 8));
		Data.Add(static_cast<uint8>(RecordLen));
		Data.Add(static_cast<uint8>(RecordLen >> 8));
		for (int32 I = 0; I < 20; ++I)
		{
			Data.Add(0);
		}

		const ANSICHAR* Name = "ID";
		for (int32 I = 0; I < 11; ++I)
		{
			Data.Add(I < 2 ? static_cast<uint8>(Name[I]) : 0);
		}
		Data.Add(static_cast<uint8>('N'));
		AppendInt32LE(Data, 0);
		Data.Add(static_cast<uint8>(FieldLen));
		Data.Add(0);
		for (int32 I = 0; I < 14; ++I)
		{
			Data.Add(0);
		}
		Data.Add(0x0D);

		for (int32 Rec = 1; Rec <= NumRecords; ++Rec)
		{
			Data.Add(0x20);
			const FString Num = FString::Printf(TEXT("%10d"), Rec);
			for (int32 I = 0; I < FieldLen; ++I)
			{
				const TCHAR Ch = Num.IsValidIndex(I) ? Num[I] : TCHAR(' ');
				Data.Add(static_cast<uint8>(Ch < 128 ? Ch : ' '));
			}
		}
		Data.Add(0x1A);
		if (!FFileHelper::SaveArrayToFile(Data, *DbfPath))
		{
			OutError = FString::Printf(TEXT("Failed to write DBF: %s"), *DbfPath);
			return false;
		}
		return true;
	}

	void AppendPolygonZRecord(
		TArray<uint8>& Shp,
		TArray<uint8>& Shx,
		int32 RecordNumber,
		const double* X,
		const double* Y,
		const double* Z,
		int32 NumPoints)
	{
		double RMinX = X[0], RMaxX = X[0];
		double RMinY = Y[0], RMaxY = Y[0];
		double RMinZ = Z[0], RMaxZ = Z[0];
		for (int32 I = 1; I < NumPoints; ++I)
		{
			RMinX = FMath::Min(RMinX, X[I]);
			RMaxX = FMath::Max(RMaxX, X[I]);
			RMinY = FMath::Min(RMinY, Y[I]);
			RMaxY = FMath::Max(RMaxY, Y[I]);
			RMinZ = FMath::Min(RMinZ, Z[I]);
			RMaxZ = FMath::Max(RMaxZ, Z[I]);
		}

		constexpr int32 ShapeType = 15;
		const int32 ContentBytes = 4 + 32 + 4 + 4 + 4 + NumPoints * 16 + 16 + NumPoints * 8;
		const int32 OffsetWords = Shp.Num() / 2;
		AppendInt32BE(Shp, RecordNumber);
		AppendInt32BE(Shp, ContentBytes / 2);
		AppendInt32LE(Shp, ShapeType);
		AppendDoubleLE(Shp, RMinX);
		AppendDoubleLE(Shp, RMinY);
		AppendDoubleLE(Shp, RMaxX);
		AppendDoubleLE(Shp, RMaxY);
		AppendInt32LE(Shp, 1);
		AppendInt32LE(Shp, NumPoints);
		AppendInt32LE(Shp, 0);
		for (int32 I = 0; I < NumPoints; ++I)
		{
			AppendDoubleLE(Shp, X[I]);
			AppendDoubleLE(Shp, Y[I]);
		}
		AppendDoubleLE(Shp, RMinZ);
		AppendDoubleLE(Shp, RMaxZ);
		for (int32 I = 0; I < NumPoints; ++I)
		{
			AppendDoubleLE(Shp, Z[I]);
		}
		AppendInt32BE(Shx, OffsetWords);
		AppendInt32BE(Shx, ContentBytes / 2);
	}

	bool WriteShapefileSidecars(
		const FString& Base,
		int32 NumRecords,
		TArray<uint8>& Shp,
		TArray<uint8>& Shx,
		double XMin,
		double YMin,
		double XMax,
		double YMax,
		double ZMin,
		double ZMax,
		FString& OutError)
	{
		PatchInt32BE(Shp, 24, Shp.Num() / 2);
		PatchInt32BE(Shx, 24, Shx.Num() / 2);
		PatchBoundingBox(Shp, XMin, YMin, XMax, YMax, ZMin, ZMax);
		PatchBoundingBox(Shx, XMin, YMin, XMax, YMax, ZMin, ZMax);

		const FString ShpPath = Base + TEXT(".shp");
		const FString ShxPath = Base + TEXT(".shx");
		const FString DbfPath = Base + TEXT(".dbf");
		const FString PrjPath = Base + TEXT(".prj");
		const FString CpgPath = Base + TEXT(".cpg");
		if (!FFileHelper::SaveArrayToFile(Shp, *ShpPath)
			|| !FFileHelper::SaveArrayToFile(Shx, *ShxPath))
		{
			OutError = FString::Printf(TEXT("Failed to write SHP/SHX: %s"), *ShpPath);
			return false;
		}
		if (!WriteDbf(DbfPath, NumRecords, OutError))
		{
			return false;
		}
		const FString Prj =
			TEXT("GEOGCS[\"GCS_WGS_1984\",DATUM[\"D_WGS_1984\",SPHEROID[\"WGS_1984\",6378137.0,298.257223563]],")
			TEXT("PRIMEM[\"Greenwich\",0.0],UNIT[\"Degree\",0.0174532925199433]]");
		if (!FFileHelper::SaveStringToFile(Prj, *PrjPath, FFileHelper::EEncodingOptions::ForceAnsi)
			|| !FFileHelper::SaveStringToFile(TEXT("UTF-8"), *CpgPath, FFileHelper::EEncodingOptions::ForceAnsi))
		{
			OutError = FString::Printf(TEXT("Failed to write PRJ/CPG next to %s"), *ShpPath);
			return false;
		}
		return true;
	}
}

bool RoadShapefileWriter::WritePolygonZRings(
	const FString& ShapefilePath,
	const TArray<FRoadShapefileRing>& InRings,
	FString& OutError,
	int32& OutFeatureCount)
{
	OutError.Reset();
	OutFeatureCount = 0;
	const FString Base = NormalizeShpPath(ShapefilePath);
	if (Base.IsEmpty())
	{
		OutError = TEXT("Export shapefile path is empty.");
		return false;
	}

	TArray<FRoadShapefileRing> Rings;
	Rings.Reserve(InRings.Num());
	for (const FRoadShapefileRing& In : InRings)
	{
		if (In.LonLat.Num() < 3)
		{
			continue;
		}
		FRoadShapefileRing Copy = In;
		CloseClockwise(Copy);
		if (Copy.LonLat.Num() >= 4)
		{
			Rings.Add(MoveTemp(Copy));
		}
	}
	if (Rings.Num() == 0)
	{
		OutError = TEXT("No road outline rings to export.");
		return false;
	}
	OutFeatureCount = Rings.Num();

	const FString Dir = FPaths::GetPath(Base);
	if (!Dir.IsEmpty() && !IFileManager::Get().DirectoryExists(*Dir))
	{
		if (!IFileManager::Get().MakeDirectory(*Dir, true))
		{
			OutError = FString::Printf(TEXT("Could not create folder: %s"), *Dir);
			return false;
		}
	}

	double XMin = Rings[0].LonLat[0].X;
	double XMax = XMin;
	double YMin = Rings[0].LonLat[0].Y;
	double YMax = YMin;
	double ZMin = 0.0;
	double ZMax = 0.0;
	bool bHaveZ = false;
	for (const FRoadShapefileRing& Ring : Rings)
	{
		for (int32 I = 0; I < Ring.LonLat.Num(); ++I)
		{
			XMin = FMath::Min(XMin, Ring.LonLat[I].X);
			XMax = FMath::Max(XMax, Ring.LonLat[I].X);
			YMin = FMath::Min(YMin, Ring.LonLat[I].Y);
			YMax = FMath::Max(YMax, Ring.LonLat[I].Y);
			const double Z = Ring.HeightM.IsValidIndex(I) ? Ring.HeightM[I] : 0.0;
			if (!bHaveZ)
			{
				ZMin = ZMax = Z;
				bHaveZ = true;
			}
			else
			{
				ZMin = FMath::Min(ZMin, Z);
				ZMax = FMath::Max(ZMax, Z);
			}
		}
	}

	constexpr int32 ShapeType = 15;
	TArray<uint8> Shp;
	TArray<uint8> Shx;
	WriteShpHeader(Shp, ShapeType);
	WriteShpHeader(Shx, ShapeType);

	for (int32 Ri = 0; Ri < Rings.Num(); ++Ri)
	{
		const FRoadShapefileRing& Ring = Rings[Ri];
		const int32 NumPoints = Ring.LonLat.Num();
		TArray<double> X, Y, Z;
		X.SetNumUninitialized(NumPoints);
		Y.SetNumUninitialized(NumPoints);
		Z.SetNumUninitialized(NumPoints);
		for (int32 I = 0; I < NumPoints; ++I)
		{
			X[I] = Ring.LonLat[I].X;
			Y[I] = Ring.LonLat[I].Y;
			Z[I] = Ring.HeightM.IsValidIndex(I) ? Ring.HeightM[I] : 0.0;
		}
		AppendPolygonZRecord(Shp, Shx, Ri + 1, X.GetData(), Y.GetData(), Z.GetData(), NumPoints);
	}

	return WriteShapefileSidecars(
		Base, Rings.Num(), Shp, Shx, XMin, YMin, XMax, YMax, ZMin, ZMax, OutError);
}

bool RoadShapefileWriter::WritePolygonZTriangles(
	const FString& ShapefilePath,
	const TArray<FRoadShapefileVertex>& Vertices,
	const TArray<int32>& Triangles,
	FString& OutError,
	int32& OutFeatureCount)
{
	OutError.Reset();
	OutFeatureCount = 0;
	const FString Base = NormalizeShpPath(ShapefilePath);
	if (Base.IsEmpty())
	{
		OutError = TEXT("Export shapefile path is empty.");
		return false;
	}

	const FString Dir = FPaths::GetPath(Base);
	if (!Dir.IsEmpty() && !IFileManager::Get().DirectoryExists(*Dir))
	{
		if (!IFileManager::Get().MakeDirectory(*Dir, true))
		{
			OutError = FString::Printf(TEXT("Could not create folder: %s"), *Dir);
			return false;
		}
	}

	constexpr int32 ShapeType = 15;
	constexpr int32 NumPoints = 4;
	constexpr int32 ContentBytes = 4 + 32 + 4 + 4 + 4 + NumPoints * 16 + 16 + NumPoints * 8;
	constexpr int32 MaxShpBytes = 2000000000;
	TArray<uint8> Shp;
	TArray<uint8> Shx;
	WriteShpHeader(Shp, ShapeType);
	WriteShpHeader(Shx, ShapeType);
	const int32 Estimated = FMath::Max(Triangles.Num() / 3, 0);
	Shp.Reserve(100 + Estimated * (8 + ContentBytes));
	Shx.Reserve(100 + Estimated * 8);

	double XMin = 0.0, XMax = 0.0, YMin = 0.0, YMax = 0.0, ZMin = 0.0, ZMax = 0.0;
	bool bHaveBBox = false;
	int32 Written = 0;

	for (int32 T = 0; T + 2 < Triangles.Num(); T += 3)
	{
		const int32 IA = Triangles[T];
		const int32 IB = Triangles[T + 1];
		const int32 IC = Triangles[T + 2];
		if (!Vertices.IsValidIndex(IA) || !Vertices.IsValidIndex(IB) || !Vertices.IsValidIndex(IC))
		{
			continue;
		}
		const FRoadShapefileVertex& A = Vertices[IA];
		const FRoadShapefileVertex& B = Vertices[IB];
		const FRoadShapefileVertex& C = Vertices[IC];
		double X[4] = { A.Lon, B.Lon, C.Lon, A.Lon };
		double Y[4] = { A.Lat, B.Lat, C.Lat, A.Lat };
		double Z[4] = { A.HeightM, B.HeightM, C.HeightM, A.HeightM };
		const double Area =
			X[0] * Y[1] - X[1] * Y[0]
			+ X[1] * Y[2] - X[2] * Y[1]
			+ X[2] * Y[0] - X[0] * Y[2];
		if (FMath::Abs(Area) < 1.0e-20)
		{
			continue;
		}
		if (Area > 0.0)
		{
			Swap(X[1], X[2]);
			Swap(Y[1], Y[2]);
			Swap(Z[1], Z[2]);
		}
		X[3] = X[0];
		Y[3] = Y[0];
		Z[3] = Z[0];

		if (Shp.Num() + 8 + ContentBytes > MaxShpBytes)
		{
			OutError = TEXT("Export shapefile would exceed the 2 GB shapefile limit.");
			return false;
		}

		for (int32 I = 0; I < NumPoints; ++I)
		{
			if (!bHaveBBox)
			{
				XMin = XMax = X[I];
				YMin = YMax = Y[I];
				ZMin = ZMax = Z[I];
				bHaveBBox = true;
			}
			else
			{
				XMin = FMath::Min(XMin, X[I]);
				XMax = FMath::Max(XMax, X[I]);
				YMin = FMath::Min(YMin, Y[I]);
				YMax = FMath::Max(YMax, Y[I]);
				ZMin = FMath::Min(ZMin, Z[I]);
				ZMax = FMath::Max(ZMax, Z[I]);
			}
		}

		++Written;
		AppendPolygonZRecord(Shp, Shx, Written, X, Y, Z, NumPoints);
	}

	if (Written == 0)
	{
		OutError = TEXT("No road TIN triangles to export.");
		return false;
	}
	OutFeatureCount = Written;
	UE_LOG(
		LogRoadPlacer,
		Display,
		TEXT("PolygonZ TIN export: %d triangles, Z %.3f .. %.3f m (ellipsoid), lon[%.6f, %.6f] lat[%.6f, %.6f]."),
		Written,
		ZMin,
		ZMax,
		XMin,
		XMax,
		YMin,
		YMax);
	return WriteShapefileSidecars(
		Base, Written, Shp, Shx, XMin, YMin, XMax, YMax, ZMin, ZMax, OutError);
}
