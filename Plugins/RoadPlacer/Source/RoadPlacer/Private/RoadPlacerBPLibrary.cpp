#include "RoadPlacerPrivatePCH.h"
#include "RoadPlacerBPLibrary.h"

#include "RoadCesiumPlacement.h"
#include "RoadPlacerLog.h"
#include "RoadShapefileReader.h"
#include "RoadStaticMesh.h"
#include "RoadTriangulate.h"

#include "Algo/Reverse.h"
#include "Cesium3DTileset.h"
#include "CesiumCartographicPolygon.h"
#include "CesiumGeoreference.h"
#include "CesiumPolygonRasterOverlay.h"
#include "Components/SplineComponent.h"
#include "Editor.h"
#include "Engine/Engine.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Internationalization/Internationalization.h"
#include "Materials/MaterialInterface.h"
#include "Misc/PackageName.h"
#include "Misc/ScopedSlowTask.h"
#include "Templates/Function.h"
#include "UObject/SoftObjectPath.h"

namespace
{
	const FName RoadPlacerTag(TEXT("RoadPlacer"));
	const FName RoadPlacerOverlayName(TEXT("RoadPlacerClip"));
	constexpr double DuplicateEpsDeg = 1.0e-10;
	constexpr int32 ClipOutlineMaxVertices = 8192;
	constexpr double ClipSimplifyMeters = 0.05;
	constexpr double ClipInflateMeters = 0.05;
	constexpr double ClipBridgeWidthMeters = 0.15;

	FString SanitizeFilePath(const FString& InPath)
	{
		FString Path = InPath.TrimStartAndEnd();
		while ((Path.StartsWith(TEXT("\"")) && Path.EndsWith(TEXT("\"")) && Path.Len() >= 2)
			|| (Path.StartsWith(TEXT("'")) && Path.EndsWith(TEXT("'")) && Path.Len() >= 2))
		{
			Path = Path.Mid(1, Path.Len() - 2).TrimStartAndEnd();
		}
		return Path;
	}

	UWorld* ResolveEditorWorld(UObject* WorldContextObject)
	{
		UWorld* World = nullptr;
		if (WorldContextObject)
		{
			World = GEngine ? GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::ReturnNull) : nullptr;
		}
		if (!World && GEditor)
		{
			World = GEditor->GetEditorWorldContext().World();
		}
		return World;
	}

	UMaterialInterface* LoadMaterial(const FString& InPath)
	{
		FString Path = SanitizeFilePath(InPath);
		int32 Quote = INDEX_NONE;
		if (Path.FindChar(TCHAR('\''), Quote))
		{
			const int32 Last = Path.Find(TEXT("'"), ESearchCase::IgnoreCase, ESearchDir::FromEnd);
			if (Last > Quote)
			{
				Path = Path.Mid(Quote + 1, Last - Quote - 1);
			}
		}
		if (Path.IsEmpty())
		{
			return LoadObject<UMaterialInterface>(
				nullptr, TEXT("/Engine/EngineMaterials/DefaultMaterial.DefaultMaterial"));
		}
		if (UMaterialInterface* Mat = LoadObject<UMaterialInterface>(nullptr, *Path))
		{
			return Mat;
		}
		return Cast<UMaterialInterface>(FSoftObjectPath(Path).TryLoad());
	}

	void RemovePrevious(UWorld& World)
	{
		for (TActorIterator<ACesium3DTileset> It(&World); It; ++It)
		{
			ACesium3DTileset* Tileset = *It;
			if (!Tileset)
			{
				continue;
			}
			TArray<UCesiumPolygonRasterOverlay*> Overlays;
			Tileset->GetComponents<UCesiumPolygonRasterOverlay>(Overlays);
			for (UCesiumPolygonRasterOverlay* Overlay : Overlays)
			{
				if (!Overlay)
				{
					continue;
				}
				if (Overlay->GetFName().ToString().StartsWith(RoadPlacerOverlayName.ToString()))
				{
					Overlay->RemoveFromTileset();
					Overlay->DestroyComponent();
					Tileset->RefreshTileset();
				}
			}
		}

		TArray<AActor*> ToDestroy;
		for (TActorIterator<AActor> It(&World); It; ++It)
		{
			if (*It && (*It)->ActorHasTag(RoadPlacerTag))
			{
				ToDestroy.Add(*It);
			}
		}
		for (AActor* Actor : ToDestroy)
		{
			World.DestroyActor(Actor);
		}
	}

	struct FRoadLonLatRect
	{
		double MinLon = 0.0;
		double MaxLon = 0.0;
		double MinLat = 0.0;
		double MaxLat = 0.0;
		bool bValid = false;

		bool Contains(double Lon, double Lat) const
		{
			return !bValid
				|| (Lon >= MinLon && Lon <= MaxLon && Lat >= MinLat && Lat <= MaxLat);
		}
	};

	bool MaskLonLatBounds(
		const TArray<FRoadShapefileMask>& Masks,
		double& MinLon,
		double& MaxLon,
		double& MinLat,
		double& MaxLat)
	{
		bool bAny = false;
		auto Acc = [&](const FVector2D& P)
		{
			if (!bAny)
			{
				MinLon = MaxLon = P.X;
				MinLat = MaxLat = P.Y;
				bAny = true;
			}
			else
			{
				MinLon = FMath::Min(MinLon, P.X);
				MaxLon = FMath::Max(MaxLon, P.X);
				MinLat = FMath::Min(MinLat, P.Y);
				MaxLat = FMath::Max(MaxLat, P.Y);
			}
		};
		for (const FRoadShapefileMask& Mask : Masks)
		{
			for (const FVector2D& P : Mask.Outer.LonLat)
			{
				Acc(P);
			}
			for (const FRoadShapefileRing& Hole : Mask.Holes)
			{
				for (const FVector2D& P : Hole.LonLat)
				{
					Acc(P);
				}
			}
		}
		return bAny;
	}

	void ChooseSquareTileGrid(
		double MinLon,
		double MaxLon,
		double MinLat,
		double MaxLat,
		int32 TargetTileCount,
		int32& OutTilesX,
		int32& OutTilesY)
	{
		const int32 Target = FMath::Clamp(TargetTileCount <= 0 ? 16 : TargetTileCount, 1, 4096);
		const double LonSpan = FMath::Max(MaxLon - MinLon, 1.0e-9);
		const double LatSpan = FMath::Max(MaxLat - MinLat, 1.0e-9);
		const double MidLatRad = FMath::DegreesToRadians(0.5 * (MinLat + MaxLat));
		const double WidthM = LonSpan * FMath::Max(111320.0 * FMath::Cos(MidLatRad), 1.0);
		const double HeightM = LatSpan * 110540.0;

		int32 BestX = Target;
		int32 BestY = 1;
		double BestErr = TNumericLimits<double>::Max();
		for (int32 Y = 1; Y <= Target; ++Y)
		{
			if (Target % Y != 0)
			{
				continue;
			}
			const int32 X = Target / Y;
			const double Aspect = (WidthM / X) / FMath::Max(HeightM / Y, 1.0);
			const double Err = FMath::Abs(FMath::Loge(Aspect));
			if (Err < BestErr)
			{
				BestErr = Err;
				BestX = X;
				BestY = Y;
			}
		}
		OutTilesX = BestX;
		OutTilesY = BestY;
	}

	bool HasMissingSampleHeights(const TArray<FRoadSample>& Samples)
	{
		double MaxAbsH = 0.0;
		for (const FRoadSample& S : Samples)
		{
			if (S.HeightM > -1.0e20)
			{
				MaxAbsH = FMath::Max(MaxAbsH, FMath::Abs(S.HeightM));
			}
		}
		const bool bZeroIsMissing = MaxAbsH > 10.0;
		for (const FRoadSample& S : Samples)
		{
			const bool bKnown = S.HeightM > -1.0e20
				&& !(bZeroIsMissing && FMath::Abs(S.HeightM) < 1.0e-9);
			if (!bKnown)
			{
				return true;
			}
		}
		return false;
	}

	bool FillMissingHeights(
		TArray<FRoadSample>& Samples,
		const TFunctionRef<bool(int32, int32)> OnProgress)
	{
		TArray<int32> Known;
		TArray<int32> Unknown;
		Known.Reserve(Samples.Num());
		double MaxAbsH = 0.0;
		for (const FRoadSample& S : Samples)
		{
			if (S.HeightM > -1.0e20)
			{
				MaxAbsH = FMath::Max(MaxAbsH, FMath::Abs(S.HeightM));
			}
		}
		const bool bZeroIsMissing = MaxAbsH > 10.0;
		for (int32 I = 0; I < Samples.Num(); ++I)
		{
			const bool bKnown = Samples[I].HeightM > -1.0e20
				&& !(bZeroIsMissing && FMath::Abs(Samples[I].HeightM) < 1.0e-9);
			if (bKnown)
			{
				Known.Add(I);
			}
			else
			{
				Unknown.Add(I);
			}
		}
		if (Known.Num() == 0 || Unknown.Num() == 0)
		{
			OnProgress(1, 1);
			return true;
		}

		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Fill missing heights: %d known, %d unknown."),
			Known.Num(),
			Unknown.Num());

		auto Report = [&](int32 Done) -> bool
		{
			return OnProgress(Done, Unknown.Num());
		};
		if (!Report(0))
		{
			return false;
		}

		int32 Dropped = 0;
		auto AssignNearestNaive = [&](int32 I)
		{
			double BestD = TNumericLimits<double>::Max();
			double BestH = Samples[Known[0]].HeightM;
			for (const int32 K : Known)
			{
				const double D = FMath::Square(Samples[I].Lon - Samples[K].Lon)
					+ FMath::Square(Samples[I].Lat - Samples[K].Lat);
				if (D < BestD)
				{
					BestD = D;
					BestH = Samples[K].HeightM;
				}
			}
			Samples[I].HeightM = BestH;
		};

		if (static_cast<int64>(Unknown.Num()) * static_cast<int64>(Known.Num()) < 250000)
		{
			for (int32 Ui = 0; Ui < Unknown.Num(); ++Ui)
			{
				if ((Ui & 255) == 0 && !Report(Ui))
				{
					return false;
				}
				AssignNearestNaive(Unknown[Ui]);
			}
			return Report(Unknown.Num());
		}

		const double MidLat = Samples[Known[0]].Lat;
		const double MetersLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(MidLat)), 0.05);
		constexpr double CellM = 10.0;
		constexpr double MaxSearchM = 50.0;
		const double CellDeg = CellM / FMath::Min(MetersLon, 110540.0);
		const int32 MaxR = FMath::Max(FMath::CeilToInt(MaxSearchM / CellM), 1);
		TMap<uint64, TArray<int32>> Grid;
		auto Pack = [](int32 X, int32 Y) -> uint64
		{
			return (static_cast<uint64>(static_cast<uint32>(X)) << 32) | static_cast<uint32>(Y);
		};
		for (const int32 K : Known)
		{
			const int32 CX = FMath::FloorToInt(Samples[K].Lon / CellDeg);
			const int32 CY = FMath::FloorToInt(Samples[K].Lat / CellDeg);
			Grid.FindOrAdd(Pack(CX, CY)).Add(K);
		}
		for (int32 Ui = 0; Ui < Unknown.Num(); ++Ui)
		{
			if ((Ui & 255) == 0 && !Report(Ui))
			{
				return false;
			}
			const int32 I = Unknown[Ui];
			const int32 CX = FMath::FloorToInt(Samples[I].Lon / CellDeg);
			const int32 CY = FMath::FloorToInt(Samples[I].Lat / CellDeg);
			double BestD = TNumericLimits<double>::Max();
			double BestH = Samples[Known[0]].HeightM;
			bool bHit = false;
			for (int32 R = 0; R <= MaxR; ++R)
			{
				for (int32 DY = -R; DY <= R; ++DY)
				{
					for (int32 DX = -R; DX <= R; ++DX)
					{
						if (R > 0 && FMath::Abs(DX) != R && FMath::Abs(DY) != R)
						{
							continue;
						}
						if (const TArray<int32>* Cell = Grid.Find(Pack(CX + DX, CY + DY)))
						{
							for (const int32 K : *Cell)
							{
								const double D = FMath::Square(Samples[I].Lon - Samples[K].Lon)
									+ FMath::Square(Samples[I].Lat - Samples[K].Lat);
								if (D < BestD)
								{
									BestD = D;
									BestH = Samples[K].HeightM;
									bHit = true;
								}
							}
						}
					}
				}
				if (bHit)
				{
					break;
				}
			}
			if (bHit)
			{
				Samples[I].HeightM = BestH;
			}
			else
			{
				Samples[I].HeightM = TNumericLimits<double>::Lowest();
				++Dropped;
			}
		}

		if (Dropped > 0)
		{
			Samples.RemoveAll([](const FRoadSample& S)
			{
				return S.HeightM <= -1.0e20;
			});
			UE_LOG(
				LogRoadPlacer,
				Display,
				TEXT("Fill missing heights: dropped %d sample(s) with no PointZ within %.0fm."),
				Dropped,
				MaxSearchM);
		}
		return Report(Unknown.Num());
	}

	struct FOutlineIndex
	{
		struct FEdge
		{
			FVector2D A;
			FVector2D B;
		};

		TArray<FEdge> Edges;
		TMap<uint64, TArray<int32>> Cells;
		double CellDeg = 0.00002;

		static uint64 Pack(int32 X, int32 Y)
		{
			return (static_cast<uint64>(static_cast<uint32>(X)) << 32) | static_cast<uint32>(Y);
		}

		void AddRing(const TArray<FVector2D>& Ring)
		{
			const int32 N = Ring.Num();
			if (N < 2)
			{
				return;
			}
			for (int32 I = 0; I < N; ++I)
			{
				const FVector2D& A = Ring[I];
				const FVector2D& B = Ring[(I + 1) % N];
				const int32 EdgeIndex = Edges.Num();
				Edges.Add({ A, B });
				const int32 MinX = FMath::FloorToInt(FMath::Min(A.X, B.X) / CellDeg);
				const int32 MaxX = FMath::FloorToInt(FMath::Max(A.X, B.X) / CellDeg);
				const int32 MinY = FMath::FloorToInt(FMath::Min(A.Y, B.Y) / CellDeg);
				const int32 MaxY = FMath::FloorToInt(FMath::Max(A.Y, B.Y) / CellDeg);
				for (int32 Y = MinY; Y <= MaxY; ++Y)
				{
					for (int32 X = MinX; X <= MaxX; ++X)
					{
						Cells.FindOrAdd(Pack(X, Y)).Add(EdgeIndex);
					}
				}
			}
		}

		void Build(const TArray<FRoadShapefileMask>& Masks)
		{
			Edges.Reset();
			Cells.Reset();
			double SumLat = 0.0;
			int32 Count = 0;
			for (const FRoadShapefileMask& Mask : Masks)
			{
				for (const FVector2D& P : Mask.Outer.LonLat)
				{
					SumLat += P.Y;
					++Count;
				}
			}
			const double MidLat = (Count > 0) ? (SumLat / static_cast<double>(Count)) : 0.0;
			const double MetersLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(MidLat)), 0.05);
			CellDeg = 2.0 / FMath::Min(MetersLon, 110540.0);

			for (const FRoadShapefileMask& Mask : Masks)
			{
				AddRing(Mask.Outer.LonLat);
				for (const FRoadShapefileRing& Hole : Mask.Holes)
				{
					AddRing(Hole.LonLat);
				}
			}
		}

		double DistPointSegM(const FVector2D& P, const FVector2D& A, const FVector2D& B) const
		{
			const double MLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(P.Y)), 0.05);
			const double MLat = 110540.0;
			const double Px = (P.X - A.X) * MLon;
			const double Py = (P.Y - A.Y) * MLat;
			const double Bx = (B.X - A.X) * MLon;
			const double By = (B.Y - A.Y) * MLat;
			const double LenSq = Bx * Bx + By * By;
			double T = (LenSq > 1.0e-12) ? ((Px * Bx + Py * By) / LenSq) : 0.0;
			T = FMath::Clamp(T, 0.0, 1.0);
			const double Dx = Px - T * Bx;
			const double Dy = Py - T * By;
			return FMath::Sqrt(Dx * Dx + Dy * Dy);
		}

		bool IsNear(const FVector2D& P, double TolM) const
		{
			if (Edges.Num() == 0)
			{
				return false;
			}
			const int32 CX = FMath::FloorToInt(P.X / CellDeg);
			const int32 CY = FMath::FloorToInt(P.Y / CellDeg);
			// Cells are ~2 m. Search far enough to honor TolM (default 15 m).
			const int32 CellR = FMath::Clamp(FMath::CeilToInt(TolM / 2.0) + 1, 2, 16);
			for (int32 DY = -CellR; DY <= CellR; ++DY)
			{
				for (int32 DX = -CellR; DX <= CellR; ++DX)
				{
					if (const TArray<int32>* Hits = Cells.Find(Pack(CX + DX, CY + DY)))
					{
						for (const int32 EdgeIndex : *Hits)
						{
							if (DistPointSegM(P, Edges[EdgeIndex].A, Edges[EdgeIndex].B) <= TolM)
							{
								return true;
							}
						}
					}
				}
			}
			return false;
		}
	};

	double EvalLinearZ(
		double SQuery,
		const TArray<double>& AS,
		const TArray<double>& AZ,
		bool bClosed,
		double RingLen)
	{
		const int32 N = AZ.Num();
		if (N <= 0)
		{
			return TNumericLimits<double>::Lowest();
		}
		if (N == 1)
		{
			return AZ[0];
		}

		auto LerpAB = [&](int32 I0, int32 I1, double A, double B, double Q) -> double
		{
			const double T = FMath::Clamp((Q - A) / FMath::Max(B - A, 1.0e-9), 0.0, 1.0);
			return AZ[I0] + T * (AZ[I1] - AZ[I0]);
		};

		if (!bClosed)
		{
			if (SQuery <= AS[0])
			{
				return AZ[0];
			}
			if (SQuery >= AS.Last())
			{
				return AZ.Last();
			}
			for (int32 I = 0; I + 1 < N; ++I)
			{
				if (SQuery <= AS[I + 1] + 1.0e-9)
				{
					return LerpAB(I, I + 1, AS[I], AS[I + 1], SQuery);
				}
			}
			return AZ.Last();
		}

		double Q = SQuery;
		if (Q + 1.0e-9 < AS[0])
		{
			Q += RingLen;
		}
		for (int32 I = 0; I + 1 < N; ++I)
		{
			if (Q + 1.0e-9 >= AS[I] && Q <= AS[I + 1] + 1.0e-9)
			{
				return LerpAB(I, I + 1, AS[I], AS[I + 1], Q);
			}
		}
		const double WrapEnd = AS[0] + RingLen;
		if (Q + 1.0e-9 >= AS.Last() && Q <= WrapEnd + 1.0e-9)
		{
			return LerpAB(N - 1, 0, AS.Last(), WrapEnd, Q);
		}
		return AZ[0];
	}

	void LogSampleHeights(const TCHAR* Tag, const TArray<FRoadSample>& Samples)
	{
		double ZMin = TNumericLimits<double>::Max();
		double ZMax = TNumericLimits<double>::Lowest();
		double ZSum = 0.0;
		int32 Count = 0;
		for (const FRoadSample& S : Samples)
		{
			if (S.HeightM <= -1.0e20)
			{
				continue;
			}
			ZMin = FMath::Min(ZMin, S.HeightM);
			ZMax = FMath::Max(ZMax, S.HeightM);
			ZSum += S.HeightM;
			++Count;
		}
		if (Count == 0)
		{
			UE_LOG(LogRoadPlacer, Warning, TEXT("%s: no valid PointZ heights."), Tag);
			return;
		}
		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("%s: Z min/mean/max=%.2f/%.2f/%.2f m  n=%d"),
			Tag,
			ZMin,
			ZSum / static_cast<double>(Count),
			ZMax,
			Count);
	}

	void ResampleAltitudeAlongChains(TArray<FRoadSample>& Samples, double SpacingM)
	{
		const int32 N = Samples.Num();
		if (SpacingM <= 1.0e-6 || N < 3)
		{
			return;
		}

		const double MidLat = Samples[0].Lat;
		const double MLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(MidLat)), 0.05);
		const double MLat = 110540.0;
		auto DistM = [&](int32 A, int32 B) -> double
		{
			const double Dx = (Samples[A].Lon - Samples[B].Lon) * MLon;
			const double Dy = (Samples[A].Lat - Samples[B].Lat) * MLat;
			return FMath::Sqrt(Dx * Dx + Dy * Dy);
		};

		// Outline PointZ is stored in curb-walk order (~0.3 m). Split on jumps; do not
		// snap to the mask ring or a 2D nearest-neighbor graph (those flatten a city
		// to one height, often far below the DTM).
		TArray<double> StepM;
		StepM.Reserve(N - 1);
		for (int32 I = 1; I < N; ++I)
		{
			StepM.Add(DistM(I - 1, I));
		}
		TArray<double> SortedStep = StepM;
		SortedStep.Sort();
		const double MedianStep = SortedStep[SortedStep.Num() / 2];
		const double BreakM = FMath::Clamp(MedianStep * 8.0, 1.5, 6.0);

		TArray<TArray<int32>> Chains;
		TArray<int32> Current;
		Current.Add(0);
		int32 Isolated = 0;
		for (int32 I = 1; I < N; ++I)
		{
			if (StepM[I - 1] <= BreakM)
			{
				Current.Add(I);
			}
			else
			{
				if (Current.Num() >= 2)
				{
					Chains.Add(MoveTemp(Current));
				}
				else
				{
					++Isolated;
				}
				Current.Reset();
				Current.Add(I);
			}
		}
		if (Current.Num() >= 2)
		{
			Chains.Add(MoveTemp(Current));
		}
		else if (Current.Num() == 1)
		{
			++Isolated;
		}

		int32 ChainCount = 0;
		int32 AnchorCount = 0;
		int32 SkippedShort = 0;
		constexpr double MaxDeltaM = 20.0;

		for (const TArray<int32>& Chain : Chains)
		{
			TArray<double> S;
			S.SetNum(Chain.Num());
			S[0] = 0.0;
			for (int32 I = 1; I < Chain.Num(); ++I)
			{
				S[I] = S[I - 1] + DistM(Chain[I - 1], Chain[I]);
			}
			const bool bClosed =
				Chain.Num() > 3
				&& DistM(Chain[0], Chain.Last()) <= BreakM
				&& S.Last() >= SpacingM * 2.0;
			const double RingLen = bClosed
				? (S.Last() + DistM(Chain.Last(), Chain[0]))
				: S.Last();

			TArray<int32> Anchors;
			Anchors.Add(0);
			double LastS = 0.0;
			for (int32 I = 1; I < Chain.Num(); ++I)
			{
				if ((S[I] - LastS) >= SpacingM - 1.0e-6)
				{
					Anchors.Add(I);
					LastS = S[I];
				}
			}
			if (!bClosed && Anchors.Last() != Chain.Num() - 1)
			{
				Anchors.Add(Chain.Num() - 1);
			}
			if (Anchors.Num() < 2)
			{
				++SkippedShort;
				continue;
			}

			TArray<double> AS;
			TArray<double> AZ;
			AS.Reserve(Anchors.Num());
			AZ.Reserve(Anchors.Num());
			for (const int32 U : Anchors)
			{
				AS.Add(S[U]);
				AZ.Add(Samples[Chain[U]].HeightM);
			}
			++ChainCount;
			AnchorCount += AS.Num();

			for (int32 I = 0; I < Chain.Num(); ++I)
			{
				const double OldZ = Samples[Chain[I]].HeightM;
				const double NewZ = EvalLinearZ(S[I], AS, AZ, bClosed, RingLen);
				if (NewZ <= -1.0e20)
				{
					continue;
				}
				Samples[Chain[I]].HeightM = FMath::Clamp(NewZ, OldZ - MaxDeltaM, OldZ + MaxDeltaM);
			}
		}

		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Altitude resample: spacing=%.1fm break=%.2fm (median step=%.3fm) chains=%d anchors=%d isolated=%d skippedShort=%d."),
			SpacingM,
			BreakM,
			MedianStep,
			ChainCount,
			AnchorCount,
			Isolated,
			SkippedShort);
	}

	uint64 UndirectedEdge(int32 A, int32 B)
	{
		const int32 Lo = FMath::Min(A, B);
		const int32 Hi = FMath::Max(A, B);
		return (static_cast<uint64>(static_cast<uint32>(Lo)) << 32) | static_cast<uint32>(Hi);
	}

	void BuildSlabWorld(
		ACesiumGeoreference& Georeference,
		const TArray<FRoadSample>& Samples,
		const TArray<int32>& TopTriangles,
		double TopOffsetM,
		double ThicknessM,
		TArray<FVector>& OutWorld,
		TArray<int32>& OutTriangles)
	{
		OutWorld.Reset();
		OutTriangles.Reset();
		const int32 N = Samples.Num();
		OutWorld.Reserve(N * (ThicknessM > 1.0e-6 ? 2 : 1));
		for (const FRoadSample& S : Samples)
		{
			OutWorld.Add(RoadCesiumPlacement::LonLatHeightToUnreal(
				Georeference, S.Lon, S.Lat, S.HeightM + TopOffsetM));
		}
		OutTriangles = TopTriangles;
		if (ThicknessM <= 1.0e-6)
		{
			return;
		}

		TMap<uint64, int32> EdgeCount;
		TMap<uint64, TPair<int32, int32>> EdgeDir;
		for (int32 T = 0; T + 2 < TopTriangles.Num(); T += 3)
		{
			const int32 I[3] = { TopTriangles[T], TopTriangles[T + 1], TopTriangles[T + 2] };
			for (int32 E = 0; E < 3; ++E)
			{
				const int32 A = I[E];
				const int32 B = I[(E + 1) % 3];
				const uint64 Key = UndirectedEdge(A, B);
				++EdgeCount.FindOrAdd(Key);
				if (!EdgeDir.Contains(Key))
				{
					EdgeDir.Add(Key, TPair<int32, int32>(A, B));
				}
			}
		}

		TArray<int32> BottomOf;
		BottomOf.Init(INDEX_NONE, N);
		auto EnsureBottom = [&](int32 I) -> int32
		{
			if (BottomOf[I] == INDEX_NONE)
			{
				const FRoadSample& S = Samples[I];
				BottomOf[I] = OutWorld.Num();
				OutWorld.Add(RoadCesiumPlacement::LonLatHeightToUnreal(
					Georeference, S.Lon, S.Lat, S.HeightM + TopOffsetM - ThicknessM));
			}
			return BottomOf[I];
		};

		for (const TPair<uint64, int32>& Pair : EdgeCount)
		{
			if (Pair.Value != 1)
			{
				continue;
			}
			const TPair<int32, int32>& Dir = EdgeDir.FindChecked(Pair.Key);
			const int32 A = Dir.Key;
			const int32 B = Dir.Value;
			const int32 ABot = EnsureBottom(A);
			const int32 BBot = EnsureBottom(B);
			OutTriangles.Add(A);
			OutTriangles.Add(BBot);
			OutTriangles.Add(B);
			OutTriangles.Add(A);
			OutTriangles.Add(ABot);
			OutTriangles.Add(BBot);
		}
	}

	void CompactUsedSamples(
		const TArray<FRoadSample>& InVerts,
		const TArray<int32>& InTris,
		TArray<FRoadSample>& OutVerts,
		TArray<int32>& OutTris)
	{
		TArray<int32> Remap;
		Remap.Init(INDEX_NONE, InVerts.Num());
		OutVerts.Reset();
		OutTris.Reset();
		OutTris.Reserve(InTris.Num());
		for (int32 I = 0; I + 2 < InTris.Num(); I += 3)
		{
			for (int32 K = 0; K < 3; ++K)
			{
				const int32 Old = InTris[I + K];
				if (!InVerts.IsValidIndex(Old))
				{
					continue;
				}
				if (Remap[Old] == INDEX_NONE)
				{
					Remap[Old] = OutVerts.Num();
					OutVerts.Add(InVerts[Old]);
				}
				OutTris.Add(Remap[Old]);
			}
		}
	}

	double PerpDistSqLonLat(const FVector2D& Point, const FVector2D& A, const FVector2D& B)
	{
		const FVector2D AB = B - A;
		const double LenSq = AB.SizeSquared();
		if (LenSq < 1.0e-30)
		{
			return FVector2D::DistSquared(Point, A);
		}
		const double T = FMath::Clamp(FVector2D::DotProduct(Point - A, AB) / LenSq, 0.0, 1.0);
		return FVector2D::DistSquared(Point, A + AB * T);
	}

	void RdpKeepClip(const TArray<FVector2D>& Pts, int32 Start, int32 End, double EpsSq, TArray<uint8>& Keep)
	{
		double MaxD = -1.0;
		int32 MaxI = Start;
		for (int32 I = Start + 1; I < End; ++I)
		{
			const double D = PerpDistSqLonLat(Pts[I], Pts[Start], Pts[End]);
			if (D > MaxD)
			{
				MaxD = D;
				MaxI = I;
			}
		}
		if (MaxD > EpsSq && MaxI > Start && MaxI < End)
		{
			RdpKeepClip(Pts, Start, MaxI, EpsSq, Keep);
			RdpKeepClip(Pts, MaxI, End, EpsSq, Keep);
		}
		else
		{
			Keep[Start] = 1;
			Keep[End] = 1;
		}
	}

	void DecimateClipRing(const TArray<FVector2D>& In, TArray<FVector2D>& Out)
	{
		Out.Reset();
		TArray<FVector2D> Unique;
		Unique.Reserve(In.Num());
		for (const FVector2D& P : In)
		{
			if (Unique.Num() == 0 || !Unique.Last().Equals(P, DuplicateEpsDeg))
			{
				Unique.Add(P);
			}
		}
		if (Unique.Num() >= 2 && Unique[0].Equals(Unique.Last(), DuplicateEpsDeg))
		{
			Unique.Pop();
		}
		if (Unique.Num() < 3)
		{
			Out = Unique;
			return;
		}

		const double EpsDeg = ClipSimplifyMeters / 111320.0;
		TArray<uint8> Keep;
		Keep.Init(0, Unique.Num());
		RdpKeepClip(Unique, 0, Unique.Num() - 1, EpsDeg * EpsDeg, Keep);
		Keep[0] = 1;
		Keep.Last() = 1;
		Out.Reserve(Unique.Num());
		for (int32 I = 0; I < Unique.Num(); ++I)
		{
			if (Keep[I])
			{
				Out.Add(Unique[I]);
			}
		}
		if (Out.Num() < 3)
		{
			Out = Unique;
		}
		if (Out.Num() > ClipOutlineMaxVertices)
		{
			TArray<FVector2D> Sampled;
			Sampled.Reserve(ClipOutlineMaxVertices);
			for (int32 I = 0; I < ClipOutlineMaxVertices; ++I)
			{
				const int32 Src = (I * (Out.Num() - 1)) / (ClipOutlineMaxVertices - 1);
				Sampled.Add(Out[Src]);
			}
			Out = MoveTemp(Sampled);
		}
	}

	void InflateLonLatRing(TArray<FVector2D>& Ring, double MetersOut)
	{
		const int32 N = Ring.Num();
		if (N < 3 || FMath::Abs(MetersOut) <= 1.0e-6)
		{
			return;
		}
		double SumLat = 0.0;
		for (const FVector2D& P : Ring)
		{
			SumLat += P.Y;
		}
		const double MidLat = SumLat / static_cast<double>(N);
		const double MLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(MidLat)), 0.05);
		const double MLat = 110540.0;

		double Area2 = 0.0;
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D& A = Ring[I];
			const FVector2D& B = Ring[(I + 1) % N];
			Area2 += A.X * B.Y - B.X * A.Y;
		}
		const double Sign = (Area2 >= 0.0) ? 1.0 : -1.0;

		TArray<FVector2D> Out;
		Out.SetNum(N);
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D& Prev = Ring[(I + N - 1) % N];
			const FVector2D& Cur = Ring[I];
			const FVector2D& Next = Ring[(I + 1) % N];
			FVector2D E0((Cur.X - Prev.X) * MLon, (Cur.Y - Prev.Y) * MLat);
			FVector2D E1((Next.X - Cur.X) * MLon, (Next.Y - Cur.Y) * MLat);
			FVector2D N0(-E0.Y, E0.X);
			FVector2D N1(-E1.Y, E1.X);
			N0.Normalize();
			N1.Normalize();
			FVector2D Avg = (N0 + N1) * Sign;
			if (Avg.SizeSquared() < 1.0e-12)
			{
				Avg = N0 * Sign;
			}
			Avg.Normalize();
			Out[I] = FVector2D(
				Cur.X + (Avg.X * MetersOut) / MLon,
				Cur.Y + (Avg.Y * MetersOut) / MLat);
		}
		Ring = MoveTemp(Out);
	}

	double ClipRingArea2(const TArray<FVector2D>& Ring)
	{
		double Area2 = 0.0;
		const int32 N = Ring.Num();
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D& A = Ring[I];
			const FVector2D& B = Ring[(I + 1) % N];
			Area2 += A.X * B.Y - B.X * A.Y;
		}
		return Area2;
	}

	void EnsureClipWinding(TArray<FVector2D>& Ring, bool bCCW)
	{
		if (Ring.Num() < 3)
		{
			return;
		}
		const bool bIsCCW = ClipRingArea2(Ring) >= 0.0;
		if (bIsCCW != bCCW)
		{
			Algo::Reverse(Ring);
		}
	}

	void StripClosedDuplicate(TArray<FVector2D>& Ring)
	{
		if (Ring.Num() >= 2 && Ring[0].Equals(Ring.Last(), DuplicateEpsDeg))
		{
			Ring.Pop();
		}
	}

	TArray<FVector2D> BridgeJoinOuterAndHole(
		const TArray<FVector2D>& Outer,
		const TArray<FVector2D>& Hole)
	{
		TArray<FVector2D> Out;
		if (Outer.Num() < 3 || Hole.Num() < 3)
		{
			return Out;
		}
		int32 BestO = 0;
		int32 BestH = 0;
		double BestD = TNumericLimits<double>::Max();
		for (int32 O = 0; O < Outer.Num(); ++O)
		{
			for (int32 H = 0; H < Hole.Num(); ++H)
			{
				const double Dx = Outer[O].X - Hole[H].X;
				const double Dy = Outer[O].Y - Hole[H].Y;
				const double D = Dx * Dx + Dy * Dy;
				if (D < BestD)
				{
					BestD = D;
					BestO = O;
					BestH = H;
				}
			}
		}
		Out.Reserve(Outer.Num() + Hole.Num() + 6);
		const FVector2D A = Outer[BestO];
		const FVector2D B = Hole[BestH];
		const double MidLat = 0.5 * (A.Y + B.Y);
		const double MLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(MidLat)), 0.05);
		const double MLat = 110540.0;
		FVector2D DirM((B.X - A.X) * MLon, (B.Y - A.Y) * MLat);
		const double LenM = DirM.Size();
		FVector2D Off(0.0, 0.0);
		if (LenM > 1.0e-3)
		{
			const FVector2D PerpM(-DirM.Y / LenM, DirM.X / LenM);
			Off = FVector2D(
				(PerpM.X * ClipBridgeWidthMeters) / MLon,
				(PerpM.Y * ClipBridgeWidthMeters) / MLat);
		}
		for (int32 I = 0; I <= BestO; ++I)
		{
			Out.Add(Outer[I]);
		}
		if (Off.SizeSquared() > 0.0)
		{
			Out.Add(A + Off);
			Out.Add(B + Off);
		}
		const int32 HN = Hole.Num();
		for (int32 K = 0; K < HN; ++K)
		{
			Out.Add(Hole[(BestH + K) % HN]);
		}
		Out.Add(Hole[BestH]);
		Out.Add(Outer[BestO]);
		for (int32 I = BestO + 1; I < Outer.Num(); ++I)
		{
			Out.Add(Outer[I]);
		}
		return Out;
	}

	bool ClipPointInRing(const FVector2D& P, const TArray<FVector2D>& Ring)
	{
		const int32 N = Ring.Num();
		if (N < 3)
		{
			return false;
		}
		bool bInside = false;
		for (int32 I = 0, J = N - 1; I < N; J = I++)
		{
			const FVector2D& A = Ring[I];
			const FVector2D& B = Ring[J];
			const bool bIntersect = ((A.Y > P.Y) != (B.Y > P.Y))
				&& (P.X < (B.X - A.X) * (P.Y - A.Y) / (B.Y - A.Y + 1.0e-30) + A.X);
			if (bIntersect)
			{
				bInside = !bInside;
			}
		}
		return bInside;
	}

	void ExtractBoundaryRings(
		const TArray<FRoadSample>& Verts,
		const TArray<int32>& Tris,
		TArray<TArray<FVector2D>>& OutRings)
	{
		OutRings.Reset();
		const int32 N = Verts.Num();
		if (N < 3 || Tris.Num() < 3)
		{
			return;
		}

		TMap<uint64, int32> EdgeCount;
		TMap<uint64, TPair<int32, int32>> EdgeDir;
		for (int32 T = 0; T + 2 < Tris.Num(); T += 3)
		{
			const int32 I[3] = { Tris[T], Tris[T + 1], Tris[T + 2] };
			for (int32 E = 0; E < 3; ++E)
			{
				const int32 A = I[E];
				const int32 B = I[(E + 1) % 3];
				if (!Verts.IsValidIndex(A) || !Verts.IsValidIndex(B))
				{
					continue;
				}
				const uint64 Key = UndirectedEdge(A, B);
				++EdgeCount.FindOrAdd(Key);
				if (!EdgeDir.Contains(Key))
				{
					EdgeDir.Add(Key, TPair<int32, int32>(A, B));
				}
			}
		}

		TArray<TArray<int32>> Nbr;
		Nbr.SetNum(N);
		TSet<uint64> Unused;
		for (const TPair<uint64, int32>& Pair : EdgeCount)
		{
			if (Pair.Value != 1)
			{
				continue;
			}
			const TPair<int32, int32>& Dir = EdgeDir.FindChecked(Pair.Key);
			Nbr[Dir.Key].Add(Dir.Value);
			Nbr[Dir.Value].Add(Dir.Key);
			Unused.Add(Pair.Key);
		}

		auto PopUnusedNbr = [&](int32 Cur, int32 Prev) -> int32
		{
			int32 Best = INDEX_NONE;
			for (const int32 J : Nbr[Cur])
			{
				if (J == Prev)
				{
					continue;
				}
				const uint64 Key = UndirectedEdge(Cur, J);
				if (Unused.Contains(Key))
				{
					return J;
				}
				if (Best == INDEX_NONE)
				{
					Best = J;
				}
			}
			return Best;
		};

		TArray<uint8> VertUsed;
		VertUsed.Init(0, N);
		for (int32 Start = 0; Start < N; ++Start)
		{
			if (VertUsed[Start] || Nbr[Start].Num() == 0)
			{
				continue;
			}
			int32 Cur = Start;
			int32 Prev = INDEX_NONE;
			TArray<FVector2D> Ring;
			Ring.Reserve(64);
			for (int32 Guard = 0; Guard < N + 2; ++Guard)
			{
				VertUsed[Cur] = 1;
				Ring.Add(FVector2D(Verts[Cur].Lon, Verts[Cur].Lat));
				const int32 Next = PopUnusedNbr(Cur, Prev);
				if (Next == INDEX_NONE)
				{
					break;
				}
				Unused.Remove(UndirectedEdge(Cur, Next));
				if (Next == Start)
				{
					break;
				}
				Prev = Cur;
				Cur = Next;
			}
			if (Ring.Num() >= 3)
			{
				OutRings.Add(MoveTemp(Ring));
			}
		}
	}

	void ConvertBoundaryRingsToClipRings(
		const TArray<TArray<FVector2D>>& InRings,
		TArray<TArray<FVector2D>>& OutRings)
	{
		OutRings.Reset();
		TArray<TArray<FVector2D>> Rings;
		Rings.Reserve(InRings.Num());
		for (const TArray<FVector2D>& In : InRings)
		{
			TArray<FVector2D> Decimated;
			DecimateClipRing(In, Decimated);
			StripClosedDuplicate(Decimated);
			if (Decimated.Num() >= 3)
			{
				Rings.Add(MoveTemp(Decimated));
			}
		}

		const int32 N = Rings.Num();
		if (N == 0)
		{
			return;
		}

		TArray<double> AbsArea;
		TArray<FVector2D> Centroid;
		AbsArea.SetNum(N);
		Centroid.SetNum(N);
		for (int32 I = 0; I < N; ++I)
		{
			AbsArea[I] = FMath::Abs(ClipRingArea2(Rings[I]));
			FVector2D C(0.0, 0.0);
			for (const FVector2D& P : Rings[I])
			{
				C += P;
			}
			Centroid[I] = C / static_cast<double>(Rings[I].Num());
		}

		TArray<int32> Parent;
		Parent.Init(INDEX_NONE, N);
		for (int32 I = 0; I < N; ++I)
		{
			for (int32 J = 0; J < N; ++J)
			{
				if (I == J || AbsArea[J] <= AbsArea[I])
				{
					continue;
				}
				if (!ClipPointInRing(Centroid[I], Rings[J]))
				{
					continue;
				}
				if (Parent[I] == INDEX_NONE || AbsArea[J] < AbsArea[Parent[I]])
				{
					Parent[I] = J;
				}
			}
		}

		TArray<TArray<int32>> Children;
		Children.SetNum(N);
		TArray<uint8> bHole;
		bHole.Init(0, N);
		for (int32 I = 0; I < N; ++I)
		{
			if (Parent[I] != INDEX_NONE)
			{
				Children[Parent[I]].Add(I);
				bHole[I] = 1;
			}
		}

		TArray<TArray<FVector2D>> Inflated = Rings;
		for (int32 I = 0; I < N; ++I)
		{
			if (bHole[I])
			{
				EnsureClipWinding(Inflated[I], false);
				InflateLonLatRing(Inflated[I], -ClipInflateMeters);
				EnsureClipWinding(Inflated[I], false);
			}
			else
			{
				EnsureClipWinding(Inflated[I], true);
				InflateLonLatRing(Inflated[I], ClipInflateMeters);
			}
		}

		for (int32 I = 0; I < N; ++I)
		{
			if (Parent[I] != INDEX_NONE)
			{
				continue;
			}
			TArray<FVector2D> Combined = Inflated[I];
			EnsureClipWinding(Combined, true);
			for (const int32 Child : Children[I])
			{
				Combined = BridgeJoinOuterAndHole(Combined, Inflated[Child]);
			}
			if (Combined.Num() >= 3)
			{
				OutRings.Add(MoveTemp(Combined));
			}
		}
	}

	void ApplyWorldClosedSpline(USplineComponent& Spline, const TArray<FVector>& WorldPoints)
	{
		Spline.ClearSplinePoints(false);
		Spline.SetClosedLoop(false, false);
		for (const FVector& P : WorldPoints)
		{
			Spline.AddSplinePoint(P, ESplineCoordinateSpace::World, false);
		}
		Spline.SetClosedLoop(true, false);
		const int32 Num = Spline.GetNumberOfSplinePoints();
		for (int32 I = 0; I < Num; ++I)
		{
			Spline.SetSplinePointType(I, ESplinePointType::Linear, false);
		}
		Spline.UpdateSpline();
	}

	ACesiumCartographicPolygon* SpawnClipPolygon(
		UWorld& World,
		const TArray<FVector>& WorldPoints,
		const FString& Label,
		const FString& FolderPath)
	{
		if (WorldPoints.Num() < 3)
		{
			return nullptr;
		}

		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		ACesiumCartographicPolygon* Poly = World.SpawnActor<ACesiumCartographicPolygon>(
			WorldPoints[0],
			FRotator::ZeroRotator,
			Params);
		if (!Poly || !Poly->Polygon)
		{
			return nullptr;
		}

		ApplyWorldClosedSpline(*Poly->Polygon, WorldPoints);
		Poly->SetActorLabel(Label);
		Poly->Tags.AddUnique(RoadPlacerTag);
		if (!FolderPath.IsEmpty())
		{
			Poly->SetFolderPath(FName(*FolderPath));
		}
		Poly->Modify();
		return Poly;
	}

	void ApplyClipOverlayToTileset(
		ACesium3DTileset& Tileset,
		const TArray<ACesiumCartographicPolygon*>& ClipPolygons)
	{
		UCesiumPolygonRasterOverlay* Overlay = NewObject<UCesiumPolygonRasterOverlay>(
			&Tileset,
			RoadPlacerOverlayName,
			RF_Transactional);
		if (!Overlay)
		{
			return;
		}

		Overlay->bAutoActivate = false;
		Tileset.AddInstanceComponent(Overlay);
		Overlay->RegisterComponent();
		Overlay->Polygons.Reset();
		for (ACesiumCartographicPolygon* Poly : ClipPolygons)
		{
			if (Poly)
			{
				Overlay->Polygons.Add(Poly);
			}
		}
		Overlay->InvertSelection = false;
		Overlay->ExcludeSelectedTiles = true;
		Tileset.Modify();
		Overlay->Activate(true);
		Overlay->Refresh();
		Tileset.RefreshTileset();
	}
}

FRoadPlaceResult URoadPlacerBPLibrary::PlaceRoadsFromShapefiles(
	UObject* WorldContextObject,
	const FString& MaskShapefilePath,
	const FString& ElevationPointsPath,
	const FString& OptionalAltitudeFieldName,
	const FString& RoadMaterialPath,
	const FString& MeshContentFolder,
	int32 TargetTileCount,
	float MaxEdgeMeters,
	float HeightOffsetMeters,
	float ThicknessMeters,
	float AltitudeSampleMeters,
	bool bSoftenEdges,
	float MetersPerUv,
	bool bEnableCollision,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath,
	bool bClipGroundUnderRoads,
	int32 OnlyTileIndex)
{
	FRoadPlaceResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString MaskPath = SanitizeFilePath(MaskShapefilePath);
	const FString PointsPath = SanitizeFilePath(ElevationPointsPath);
	const FString MeshFolder = MeshContentFolder.IsEmpty() ? TEXT("/Game/RoadPlacer/Meshes") : MeshContentFolder;
	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("Road") : ActorLabelPrefix;
	double MaxEdge = static_cast<double>(MaxEdgeMeters);
	if (MaxEdge > 0.0 && MaxEdge < 100.0)
	{
		UE_LOG(
			LogRoadPlacer,
			Warning,
			TEXT("Max Edge Meters is %.1f (too small for curb-to-curb fill). Ignoring so the pavement is not shredded into slivers. Set 0, or >= 100 to cap triangle length."),
			MaxEdge);
		MaxEdge = 0.0;
	}
	else if (MaxEdge < 0.0)
	{
		MaxEdge = 0.0;
	}
	const double HeightOff = static_cast<double>(HeightOffsetMeters);
	const double Thickness = FMath::Max(static_cast<double>(ThicknessMeters), 0.0);
	const double AltSample = FMath::Max(static_cast<double>(AltitudeSampleMeters), 0.0);
	const double UvMeters = FMath::Max(static_cast<double>(MetersPerUv), 0.1);

	UE_LOG(LogRoadPlacer, Display, TEXT("========== Road Place START =========="));
	UE_LOG(
		LogRoadPlacer,
		Display,
		TEXT("mask='%s' points='%s' tiles=%d onlyTile=%d maxEdgeM=%.2f heightOffM=%.3f thicknessM=%.3f altSampleM=%.2f soften=%s clipGround=%s"),
		*MaskPath,
		*PointsPath,
		TargetTileCount,
		OnlyTileIndex,
		MaxEdge,
		HeightOff,
		Thickness,
		AltSample,
		bSoftenEdges ? TEXT("on") : TEXT("off"),
		bClipGroundUnderRoads ? TEXT("on") : TEXT("off"));

	UWorld* World = ResolveEditorWorld(WorldContextObject);
	if (!World)
	{
		Result.Message = TEXT("Could not resolve an editor world. Open a map first.");
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	ACesiumGeoreference* Georeference = RoadCesiumPlacement::FindGeoreference(World);
	if (!Georeference)
	{
		Result.Message = TEXT("No ACesiumGeoreference found in the level.");
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (MaskPath.IsEmpty() || PointsPath.IsEmpty())
	{
		Result.Message = TEXT("MaskShapefilePath and ElevationPointsPath are required.");
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	constexpr int32 SampleProgressChunks = 50;
	FScopedSlowTask LoadTask(
		3.0f + static_cast<float>(SampleProgressChunks) + 1.0f,
		NSLOCTEXT("RoadPlacer", "LoadProgress", "Loading road shapefiles..."));
	LoadTask.MakeDialog(true);

	LoadTask.EnterProgressFrame(1.0f, NSLOCTEXT("RoadPlacer", "ReadMask", "Reading road mask..."));
	TArray<FRoadShapefileMask> Masks;
	FString ReadError;
	if (!RoadShapefileReader::ReadMaskPolygons(MaskPath, Masks, ReadError))
	{
		Result.Message = ReadError;
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}
	Result.MaskPolygonsRead = Masks.Num();
	if (LoadTask.ShouldCancel())
	{
		Result.bCancelled = true;
		Result.Message = TEXT("Cancelled while reading the road mask.");
		return Result;
	}

	LoadTask.EnterProgressFrame(1.0f, NSLOCTEXT("RoadPlacer", "ReadPoints", "Reading outline elevation points..."));
	TArray<FRoadShapefilePoint> Points;
	if (!RoadShapefileReader::ReadElevationPoints(PointsPath, OptionalAltitudeFieldName, Points, ReadError))
	{
		Result.Message = ReadError;
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}
	Result.ElevationPointsRead = Points.Num();
	if (LoadTask.ShouldCancel())
	{
		Result.bCancelled = true;
		Result.Message = TEXT("Cancelled while reading elevation points.");
		return Result;
	}

	LoadTask.EnterProgressFrame(1.0f, NSLOCTEXT("RoadPlacer", "IndexOutline", "Indexing mask outline..."));
	FOutlineIndex Outline;
	Outline.Build(Masks);
	constexpr double OutlineSnapM = 15.0;

	// Freeze the tile grid from the mask before filtering samples. A one-tile run
	// must not recompute the grid from the remaining points or indices shift.
	double MinLon = 0.0, MaxLon = 0.0, MinLat = 0.0, MaxLat = 0.0;
	if (!MaskLonLatBounds(Masks, MinLon, MaxLon, MinLat, MaxLat))
	{
		Result.Message = TEXT("Road mask has no vertices.");
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}
	{
		const double SnapMidLat = 0.5 * (MinLat + MaxLat);
		const double SnapLon = OutlineSnapM
			/ FMath::Max(111320.0 * FMath::Cos(FMath::DegreesToRadians(SnapMidLat)), 1.0);
		const double SnapLat = OutlineSnapM / 110540.0;
		MinLon -= SnapLon;
		MaxLon += SnapLon;
		MinLat -= SnapLat;
		MaxLat += SnapLat;
	}

	int32 TilesX = 1, TilesY = 1;
	ChooseSquareTileGrid(MinLon, MaxLon, MinLat, MaxLat, TargetTileCount, TilesX, TilesY);
	const double LonSpan = FMath::Max(MaxLon - MinLon, 1.0e-9);
	const double LatSpan = FMath::Max(MaxLat - MinLat, 1.0e-9);
	const double MidLat = 0.5 * (MinLat + MaxLat);
	const double PadM = FMath::Max(MaxEdge, 80.0);
	const double PadLon = PadM / FMath::Max(111320.0 * FMath::Cos(FMath::DegreesToRadians(MidLat)), 1.0);
	const double PadLat = PadM / 110540.0;
	const int32 NumTiles = TilesX * TilesY;
	const bool bOneTile = OnlyTileIndex > 0;
	int32 WantedTile = 0;
	FRoadLonLatRect SampleClip;
	if (bOneTile)
	{
		WantedTile = FMath::Clamp(OnlyTileIndex, 1, NumTiles);
		if (OnlyTileIndex != WantedTile)
		{
			UE_LOG(
				LogRoadPlacer,
				Warning,
				TEXT("Only Tile Index %d is outside 1-%d; using %d."),
				OnlyTileIndex,
				NumTiles,
				WantedTile);
		}
		const int32 WantedTX = (WantedTile - 1) % TilesX;
		const int32 WantedTY = (WantedTile - 1) / TilesX;
		SampleClip.MinLon = MinLon + LonSpan * (static_cast<double>(WantedTX) / TilesX) - PadLon;
		SampleClip.MaxLon = MinLon + LonSpan * (static_cast<double>(WantedTX + 1) / TilesX) + PadLon;
		SampleClip.MinLat = MinLat + LatSpan * (static_cast<double>(WantedTY) / TilesY) - PadLat;
		SampleClip.MaxLat = MinLat + LatSpan * (static_cast<double>(WantedTY + 1) / TilesY) + PadLat;
		SampleClip.bValid = true;
		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Only Tile Index %d of %d (grid %dx%d, TY then TX, cell %d,%d). Other tiles are skipped."),
			WantedTile,
			NumTiles,
			TilesX,
			TilesY,
			WantedTX,
			WantedTY);
	}
	else
	{
		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Tile grid %dx%d from mask bounds (Target Tile Count %d)."),
			TilesX,
			TilesY,
			TargetTileCount);
	}

	TArray<FRoadSample> AllSamples;
	AllSamples.Reserve(bOneTile ? 8192 : Points.Num() + 256);
	int32 ZeroZ = 0;
	int32 UsedPoints = 0;
	int32 SkippedOutsideTile = 0;
	int32 SkippedNotOnMask = 0;
	int32 SampleChunksEntered = 0;
	const int32 SampleChunkSize = FMath::Max((Points.Num() + SampleProgressChunks - 1) / SampleProgressChunks, 1);
	int32 NextSampleChunkAt = 0;
	for (int32 Pi = 0; Pi < Points.Num(); ++Pi)
	{
		if (Pi >= NextSampleChunkAt && SampleChunksEntered < SampleProgressChunks)
		{
			LoadTask.EnterProgressFrame(
				1.0f,
				FText::FromString(FString::Printf(
					TEXT("Selecting outline elevation samples (%d / %d)..."),
					Pi,
					Points.Num())));
			++SampleChunksEntered;
			NextSampleChunkAt += SampleChunkSize;
			if (LoadTask.ShouldCancel())
			{
				Result.bCancelled = true;
				Result.Message = TEXT("Cancelled while selecting elevation samples.");
				return Result;
			}
		}
		if ((Pi & 8191) == 0 && LoadTask.ShouldCancel())
		{
			Result.bCancelled = true;
			Result.Message = TEXT("Cancelled while selecting elevation samples.");
			return Result;
		}
		const FRoadShapefilePoint& P = Points[Pi];
		if (!SampleClip.Contains(P.LonDeg, P.LatDeg))
		{
			++SkippedOutsideTile;
			continue;
		}
		const FVector2D LonLat(P.LonDeg, P.LatDeg);
		// Outline PointZ often sits on the ring and fails a strict point-in-polygon test.
		if (!Outline.IsNear(LonLat, OutlineSnapM) && !RoadTriangulate::PointInMask(LonLat, Masks))
		{
			++SkippedNotOnMask;
			continue;
		}
		FRoadSample S;
		S.Lon = P.LonDeg;
		S.Lat = P.LatDeg;
		S.HeightM = P.HeightM;
		++UsedPoints;
		if (FMath::Abs(S.HeightM) < 1.0e-9)
		{
			++ZeroZ;
		}
		AllSamples.Add(S);
	}
	if (SampleChunksEntered < SampleProgressChunks)
	{
		LoadTask.EnterProgressFrame(
			static_cast<float>(SampleProgressChunks - SampleChunksEntered),
			NSLOCTEXT("RoadPlacer", "PickSamplesDone", "Selecting outline elevation samples..."));
	}

	LogSampleHeights(TEXT("PointZ"), AllSamples);
	LoadTask.EnterProgressFrame(
		1.0f,
		AltSample > 1.0e-6
			? NSLOCTEXT("RoadPlacer", "ResampleAlt", "Resampling altitude along curb walks...")
			: NSLOCTEXT("RoadPlacer", "SkipResampleAlt", "Keeping every PointZ height..."));
	if (LoadTask.ShouldCancel())
	{
		Result.bCancelled = true;
		Result.Message = TEXT("Cancelled while resampling altitude.");
		return Result;
	}
	if (AltSample > 1.0e-6)
	{
		ResampleAltitudeAlongChains(AllSamples, AltSample);
		LogSampleHeights(TEXT("After altitude resample"), AllSamples);
	}

	if (HasMissingSampleHeights(AllSamples))
	{
		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Some outline PointZ heights are missing or 0; copying Z from nearest neighbors."));
		FScopedSlowTask FillTask(
			20.0f,
			NSLOCTEXT("RoadPlacer", "FillHeights", "Filling missing PointZ heights..."));
		FillTask.MakeDialog(true);
		int32 FillChunksEntered = 0;
		auto OnFillProgress = [&](int32 Done, int32 Total) -> bool
		{
			const int32 SafeTotal = FMath::Max(Total, 1);
			int32 Want = FMath::Clamp((Done * 20) / SafeTotal, 0, 20);
			if (FillChunksEntered == 0)
			{
				Want = FMath::Max(Want, 1);
			}
			while (FillChunksEntered < Want)
			{
				FillTask.EnterProgressFrame(
					1.0f,
					FText::FromString(FString::Printf(
						TEXT("Filling missing PointZ heights (%d / %d)..."),
						Done,
						Total)));
				++FillChunksEntered;
				if (FillTask.ShouldCancel() || LoadTask.ShouldCancel())
				{
					return false;
				}
			}
			return !(FillTask.ShouldCancel() || LoadTask.ShouldCancel());
		};
		if (!FillMissingHeights(AllSamples, OnFillProgress))
		{
			Result.bCancelled = true;
			Result.Message = TEXT("Cancelled while filling sample heights.");
			return Result;
		}
		LogSampleHeights(TEXT("After PointZ height fill"), AllSamples);
	}
	else
	{
		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Using outline PointZ only (no mask ring samples, no height fill)."));
	}

	if (bOneTile)
	{
		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Using %d of %d elevation point(s); skipped %d outside tile %d pad, %d not on/near mask (mask has %d polygon(s))."),
			UsedPoints,
			Points.Num(),
			SkippedOutsideTile,
			WantedTile,
			SkippedNotOnMask,
			Masks.Num());
	}
	else
	{
		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Using %d of %d elevation point(s); %d not on/near mask (mask has %d polygon(s))."),
			UsedPoints,
			Points.Num(),
			SkippedNotOnMask,
			Masks.Num());
	}

	if (AllSamples.Num() < 3)
	{
		if (bOneTile)
		{
			Result.Message = FString::Printf(
				TEXT("Only Tile Index %d has %d PointZ on/near the mask (need >= 3). Read %d, skipped %d outside this tile pad, %d not on/near the mask. Try another tile index, or 0 for all tiles."),
				WantedTile,
				UsedPoints,
				Points.Num(),
				SkippedOutsideTile,
				SkippedNotOnMask);
		}
		else
		{
			Result.Message = FString::Printf(
				TEXT("Only %d of %d PointZ sit on or near the road mask (need >= 3). %d were not within %.0f m of the mask outline."),
				UsedPoints,
				Points.Num(),
				SkippedNotOnMask,
				OutlineSnapM);
		}
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}
	if (UsedPoints > 0 && ZeroZ == UsedPoints)
	{
		UE_LOG(
			LogRoadPlacer,
			Warning,
			TEXT("Every PointZ height is 0. Drape the points in QGIS from the DTM before placing roads."));
	}

	RemovePrevious(*World);
	UMaterialInterface* Material = LoadMaterial(RoadMaterialPath);
	if (!Material)
	{
		UE_LOG(LogRoadPlacer, Warning, TEXT("Road material did not load; meshes will use an empty slot."));
	}

	FScopedSlowTask SlowTask(
		static_cast<float>(bOneTile ? 1 : NumTiles),
		NSLOCTEXT("RoadPlacer", "PlaceProgress", "Placing road tiles..."));
	SlowTask.MakeDialog(true);

	int32 TileIndex = 0;
	bool bStopTiles = false;
	TArray<TArray<FVector2D>> PendingClipRings;
	for (int32 TY = 0; TY < TilesY && !bStopTiles; ++TY)
	{
		for (int32 TX = 0; TX < TilesX; ++TX)
		{
			++TileIndex;
			if (bOneTile && TileIndex != WantedTile)
			{
				continue;
			}

			const double TMinLon = MinLon + LonSpan * (static_cast<double>(TX) / TilesX);
			const double TMaxLon = MinLon + LonSpan * (static_cast<double>(TX + 1) / TilesX);
			const double TMinLat = MinLat + LatSpan * (static_cast<double>(TY) / TilesY);
			const double TMaxLat = MinLat + LatSpan * (static_cast<double>(TY + 1) / TilesY);

			TArray<FRoadSample> TileSamples;
			for (const FRoadSample& S : AllSamples)
			{
				if (S.Lon >= TMinLon - PadLon && S.Lon <= TMaxLon + PadLon
					&& S.Lat >= TMinLat - PadLat && S.Lat <= TMaxLat + PadLat)
				{
					TileSamples.Add(S);
				}
			}
			if (TileSamples.Num() < 3)
			{
				++Result.TilesSkipped;
				if (bOneTile)
				{
					UE_LOG(
						LogRoadPlacer,
						Warning,
						TEXT("Only Tile Index %d (grid %d,%d) has no road samples."),
						WantedTile,
						TX,
						TY);
					bStopTiles = true;
					break;
				}
				continue;
			}

			const FText TileLabel = FText::FromString(FString::Printf(
				TEXT("Road tile %d / %d (grid %d,%d)"), TileIndex, NumTiles, TX, TY));
			SlowTask.EnterProgressFrame(1.0f, TileLabel);
			if (SlowTask.ShouldCancel())
			{
				Result.bCancelled = true;
				bStopTiles = true;
				break;
			}
			FScopedSlowTask TileTask(100.0f, TileLabel);

			const double TileStart = FPlatformTime::Seconds();
			UE_LOG(
				LogRoadPlacer,
				Display,
				TEXT("Tile %d / %d (grid %d,%d): %d sample(s)."),
				TileIndex,
				NumTiles,
				TX,
				TY,
				TileSamples.Num());
			if (TileSamples.Num() > 25000 && NumTiles == 1)
			{
				UE_LOG(
					LogRoadPlacer,
					Warning,
					TEXT("One tile has %d samples. Prefer Target Tile Count 16-64; a single city mesh is slow to save even after triangulation."),
					TileSamples.Num());
			}

			float LastTinFrac = 0.0f;
			auto OnTinProgress = [&](float Fraction01, const TCHAR* Stage) -> bool
			{
				if (SlowTask.ShouldCancel())
				{
					return false;
				}
				const float Delta = FMath::Max(Fraction01 - LastTinFrac, 0.0f);
				LastTinFrac = Fraction01;
				TileTask.EnterProgressFrame(
					88.0f * Delta,
					FText::FromString(FString::Printf(
						TEXT("Tile %d / %d - %s"),
						TileIndex,
						NumTiles,
						Stage)));
				return true;
			};

			FRoadTin Tin;
			FString TinError;
			if (!RoadTriangulate::BuildTin(TileSamples, Masks, MaxEdge, Tin, TinError, OnTinProgress))
			{
				++Result.TilesSkipped;
				if (TinError == TEXT("Cancelled."))
				{
					Result.bCancelled = true;
					bStopTiles = true;
					break;
				}
				UE_LOG(LogRoadPlacer, Verbose, TEXT("Tile %d,%d: %s"), TX, TY, *TinError);
				continue;
			}
			if (LastTinFrac < 1.0f)
			{
				TileTask.EnterProgressFrame(88.0f * (1.0f - LastTinFrac));
			}
			UE_LOG(
				LogRoadPlacer,
				Display,
				TEXT("Tile %d / %d: triangulated %d tri(s) in %.1fs."),
				TileIndex,
				NumTiles,
				Tin.Triangles.Num() / 3,
				FPlatformTime::Seconds() - TileStart);

			// Keep triangles whose centroid is in this tile (not only the pad).
			TArray<int32> Kept;
			Kept.Reserve(Tin.Triangles.Num());
			for (int32 T = 0; T + 2 < Tin.Triangles.Num(); T += 3)
			{
				const FRoadSample& A = Tin.Vertices[Tin.Triangles[T]];
				const FRoadSample& B = Tin.Vertices[Tin.Triangles[T + 1]];
				const FRoadSample& C = Tin.Vertices[Tin.Triangles[T + 2]];
				const double CLon = (A.Lon + B.Lon + C.Lon) / 3.0;
				const double CLat = (A.Lat + B.Lat + C.Lat) / 3.0;
				const bool bLastX = (TX == TilesX - 1);
				const bool bLastY = (TY == TilesY - 1);
				const bool bInX = CLon >= TMinLon && (bLastX ? CLon <= TMaxLon : CLon < TMaxLon);
				const bool bInY = CLat >= TMinLat && (bLastY ? CLat <= TMaxLat : CLat < TMaxLat);
				if (bInX && bInY)
				{
					Kept.Add(Tin.Triangles[T]);
					Kept.Add(Tin.Triangles[T + 1]);
					Kept.Add(Tin.Triangles[T + 2]);
				}
			}
			if (Kept.Num() < 3)
			{
				++Result.TilesSkipped;
				continue;
			}

			TArray<FRoadSample> UsedVerts;
			TArray<int32> UsedTris;
			CompactUsedSamples(Tin.Vertices, Kept, UsedVerts, UsedTris);

			if (bClipGroundUnderRoads)
			{
				TArray<TArray<FVector2D>> TileRings;
				ExtractBoundaryRings(UsedVerts, UsedTris, TileRings);
				TArray<TArray<FVector2D>> ClipRings;
				ConvertBoundaryRingsToClipRings(TileRings, ClipRings);
				PendingClipRings.Append(MoveTemp(ClipRings));
			}

			TileTask.EnterProgressFrame(
				6.0f,
				FText::FromString(FString::Printf(TEXT("Tile %d / %d - building slab mesh"), TileIndex, NumTiles)));
			TArray<FVector> WorldPts;
			TArray<int32> SlabTris;
			BuildSlabWorld(*Georeference, UsedVerts, UsedTris, HeightOff, Thickness, WorldPts, SlabTris);
			if (WorldPts.Num() < 3 || SlabTris.Num() < 3)
			{
				++Result.TilesSkipped;
				continue;
			}

			FVector Origin = FVector::ZeroVector;
			for (const FVector& P : WorldPts)
			{
				Origin += P;
			}
			Origin /= static_cast<double>(WorldPts.Num());
			TArray<FVector> LocalPts;
			LocalPts.Reserve(WorldPts.Num());
			for (const FVector& P : WorldPts)
			{
				LocalPts.Add(P - Origin);
			}

			const FString MeshLabel = FString::Printf(TEXT("%s_Tile_%d_%d"), *LabelPrefix, TX, TY);
			FString MeshError;
			TileTask.EnterProgressFrame(
				6.0f,
				FText::FromString(FString::Printf(TEXT("Tile %d / %d - saving mesh"), TileIndex, NumTiles)));
			UStaticMesh* Mesh = RoadStaticMesh::CreatePersistentStaticMesh(
				MeshFolder, MeshLabel, LocalPts, SlabTris, Material, UvMeters, bSoftenEdges, MeshError);
			if (!Mesh)
			{
				++Result.TilesSkipped;
				UE_LOG(LogRoadPlacer, Warning, TEXT("Tile %d,%d mesh failed: %s"), TX, TY, *MeshError);
				continue;
			}

			if (RoadStaticMesh::SpawnMeshActor(
					*World, Origin, Mesh, Material, MeshLabel, EditorFolderPath, RoadPlacerTag, bEnableCollision))
			{
				++Result.TilesSpawned;
				Result.TrianglesBuilt += SlabTris.Num() / 3;
			}
			else
			{
				++Result.TilesSkipped;
			}
			if (bOneTile)
			{
				bStopTiles = true;
				break;
			}
		}
		if (Result.bCancelled)
		{
			break;
		}
	}

	if (bClipGroundUnderRoads && !Result.bCancelled && Result.TilesSpawned > 0)
	{
		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Clip rings from road mesh: %d polygon(s)."),
			PendingClipRings.Num());

		TArray<ACesium3DTileset*> Tilesets;
		for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
		{
			if (*It)
			{
				Tilesets.Add(*It);
			}
		}
		if (Tilesets.Num() == 0)
		{
			UE_LOG(
				LogRoadPlacer,
				Warning,
				TEXT("Clip Ground Under Roads is on but no ACesium3DTileset is in the level. Road meshes will still spawn."));
		}

		TArray<ACesiumCartographicPolygon*> ClipActors;
		ClipActors.Reserve(PendingClipRings.Num());
		for (int32 Ri = 0; Ri < PendingClipRings.Num(); ++Ri)
		{
			const TArray<FVector2D>& Ring = PendingClipRings[Ri];
			if (Ring.Num() < 3)
			{
				continue;
			}
			TArray<FVector> WorldPts;
			WorldPts.Reserve(Ring.Num());
			for (const FVector2D& LonLat : Ring)
			{
				WorldPts.Add(RoadCesiumPlacement::LonLatHeightToUnreal(
					*Georeference, LonLat.X, LonLat.Y, 0.0));
			}
			const FString ClipLabel = FString::Printf(TEXT("%s_Clip_%d"), *LabelPrefix, Ri);
			if (ACesiumCartographicPolygon* ClipActor = SpawnClipPolygon(
					*World, WorldPts, ClipLabel, EditorFolderPath))
			{
				ClipActors.Add(ClipActor);
				++Result.ClipPolygonsSpawned;
			}
		}
		for (ACesium3DTileset* Tileset : Tilesets)
		{
			if (!Tileset || ClipActors.Num() == 0)
			{
				continue;
			}
			ApplyClipOverlayToTileset(*Tileset, ClipActors);
			++Result.TilesetsClipped;
		}
		UE_LOG(
			LogRoadPlacer,
			Display,
			TEXT("Clipped ground under roads on %d tileset(s) (%d clip polygons)."),
			Result.TilesetsClipped,
			Result.ClipPolygonsSpawned);
	}

	World->MarkPackageDirty();
	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	Result.bSuccess = Result.TilesSpawned > 0;
	if (!Result.bSuccess)
	{
		Result.Message = Result.bCancelled
			? TEXT("Cancelled before any road tiles were spawned.")
			: TEXT("No road StaticMeshActors were spawned. Outline points must sit on the mask.");
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	Result.Message = FString::Printf(
		TEXT("Spawned %d road tile(s) (%d triangles) from %d mask polygon(s) and %d points. OnlyTile=%s. Clip polygons=%d tilesets=%d. Elapsed: %.2fs."),
		Result.TilesSpawned,
		Result.TrianglesBuilt,
		Result.MaskPolygonsRead,
		Result.ElevationPointsRead,
		bOneTile ? *FString::Printf(TEXT("%d/%d"), WantedTile, NumTiles) : TEXT("all"),
		Result.ClipPolygonsSpawned,
		Result.TilesetsClipped,
		Result.ElapsedSeconds);
	UE_LOG(LogRoadPlacer, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogRoadPlacer, Display, TEXT("========== Road Place END =========="));
	return Result;
}
