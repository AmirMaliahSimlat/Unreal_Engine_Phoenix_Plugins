#include "RoadPlacerBPLibrary.h"

#include "RoadCesiumPlacement.h"
#include "RoadPlacerLog.h"
#include "RoadShapefileReader.h"
#include "RoadStaticMesh.h"
#include "RoadTriangulate.h"

#include "CesiumGeoreference.h"
#include "Editor.h"
#include "Engine/Engine.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/PlatformTime.h"
#include "Internationalization/Internationalization.h"
#include "Materials/MaterialInterface.h"
#include "Misc/PackageName.h"
#include "Misc/ScopedSlowTask.h"
#include "UObject/SoftObjectPath.h"

namespace
{
	const FName RoadPlacerTag(TEXT("RoadPlacer"));

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

	void CollectMaskSamples(const TArray<FRoadShapefileMask>& Masks, TArray<FRoadSample>& Out)
	{
		auto AddRing = [&](const FRoadShapefileRing& Ring)
		{
			for (int32 I = 0; I < Ring.LonLat.Num(); ++I)
			{
				FRoadSample S;
				S.Lon = Ring.LonLat[I].X;
				S.Lat = Ring.LonLat[I].Y;
				S.HeightM = Ring.HeightM.IsValidIndex(I) ? Ring.HeightM[I] : TNumericLimits<double>::Lowest();
				Out.Add(S);
			}
		};
		for (const FRoadShapefileMask& Mask : Masks)
		{
			AddRing(Mask.Outer);
			for (const FRoadShapefileRing& Hole : Mask.Holes)
			{
				AddRing(Hole);
			}
		}
	}

	void FillMissingHeights(TArray<FRoadSample>& Samples)
	{
		TArray<int32> Known;
		Known.Reserve(Samples.Num());
		for (int32 I = 0; I < Samples.Num(); ++I)
		{
			if (Samples[I].HeightM > -1.0e20)
			{
				Known.Add(I);
			}
		}
		if (Known.Num() == 0)
		{
			return;
		}
		for (int32 I = 0; I < Samples.Num(); ++I)
		{
			if (Samples[I].HeightM > -1.0e20)
			{
				continue;
			}
			double BestD = TNumericLimits<double>::Max();
			double BestH = 0.0;
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
		}
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

		for (const FRoadSample& S : Samples)
		{
			OutWorld.Add(RoadCesiumPlacement::LonLatHeightToUnreal(
				Georeference, S.Lon, S.Lat, S.HeightM + TopOffsetM - ThicknessM));
		}

		for (int32 T = 0; T + 2 < TopTriangles.Num(); T += 3)
		{
			OutTriangles.Add(TopTriangles[T] + N);
			OutTriangles.Add(TopTriangles[T + 2] + N);
			OutTriangles.Add(TopTriangles[T + 1] + N);
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
		for (const TPair<uint64, int32>& Pair : EdgeCount)
		{
			if (Pair.Value != 1)
			{
				continue;
			}
			const TPair<int32, int32>& Dir = EdgeDir.FindChecked(Pair.Key);
			const int32 A = Dir.Key;
			const int32 B = Dir.Value;
			OutTriangles.Add(A);
			OutTriangles.Add(B);
			OutTriangles.Add(B + N);
			OutTriangles.Add(A);
			OutTriangles.Add(B + N);
			OutTriangles.Add(A + N);
		}
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
	float MetersPerUv,
	bool bEnableCollision,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath)
{
	FRoadPlaceResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString MaskPath = SanitizeFilePath(MaskShapefilePath);
	const FString PointsPath = SanitizeFilePath(ElevationPointsPath);
	const FString MeshFolder = MeshContentFolder.IsEmpty() ? TEXT("/Game/RoadPlacer/Meshes") : MeshContentFolder;
	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("Road") : ActorLabelPrefix;
	const double MaxEdge = FMath::Max(static_cast<double>(MaxEdgeMeters), 0.25);
	const double HeightOff = static_cast<double>(HeightOffsetMeters);
	const double Thickness = FMath::Max(static_cast<double>(ThicknessMeters), 0.0);
	const double UvMeters = FMath::Max(static_cast<double>(MetersPerUv), 0.1);

	UE_LOG(LogRoadPlacer, Display, TEXT("========== Road Place START =========="));
	UE_LOG(
		LogRoadPlacer,
		Display,
		TEXT("mask='%s' points='%s' tiles=%d maxEdgeM=%.2f heightOffM=%.3f thicknessM=%.3f"),
		*MaskPath,
		*PointsPath,
		TargetTileCount,
		MaxEdge,
		HeightOff,
		Thickness);

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

	TArray<FRoadShapefileMask> Masks;
	FString ReadError;
	if (!RoadShapefileReader::ReadMaskPolygons(MaskPath, Masks, ReadError))
	{
		Result.Message = ReadError;
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}
	Result.MaskPolygonsRead = Masks.Num();

	TArray<FRoadShapefilePoint> Points;
	if (!RoadShapefileReader::ReadElevationPoints(PointsPath, OptionalAltitudeFieldName, Points, ReadError))
	{
		Result.Message = ReadError;
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}
	Result.ElevationPointsRead = Points.Num();

	TArray<FRoadSample> AllSamples;
	AllSamples.Reserve(Points.Num() + 256);
	int32 ZeroZ = 0;
	int32 UsedPoints = 0;
	for (const FRoadShapefilePoint& P : Points)
	{
		if (!RoadTriangulate::PointInMask(FVector2D(P.LonDeg, P.LatDeg), Masks))
		{
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

	CollectMaskSamples(Masks, AllSamples);
	FillMissingHeights(AllSamples);

	if (AllSamples.Num() < 3)
	{
		Result.Message = TEXT("Fewer than 3 elevation samples fall inside the road mask.");
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

	double MinLon = AllSamples[0].Lon, MaxLon = AllSamples[0].Lon;
	double MinLat = AllSamples[0].Lat, MaxLat = AllSamples[0].Lat;
	for (const FRoadSample& S : AllSamples)
	{
		MinLon = FMath::Min(MinLon, S.Lon);
		MaxLon = FMath::Max(MaxLon, S.Lon);
		MinLat = FMath::Min(MinLat, S.Lat);
		MaxLat = FMath::Max(MaxLat, S.Lat);
	}

	int32 TilesX = 1, TilesY = 1;
	ChooseSquareTileGrid(MinLon, MaxLon, MinLat, MaxLat, TargetTileCount, TilesX, TilesY);
	const double LonSpan = FMath::Max(MaxLon - MinLon, 1.0e-9);
	const double LatSpan = FMath::Max(MaxLat - MinLat, 1.0e-9);
	const double MidLat = 0.5 * (MinLat + MaxLat);
	const double PadLon = MaxEdge / FMath::Max(111320.0 * FMath::Cos(FMath::DegreesToRadians(MidLat)), 1.0);
	const double PadLat = MaxEdge / 110540.0;

	RemovePrevious(*World);
	UMaterialInterface* Material = LoadMaterial(RoadMaterialPath);
	if (!Material)
	{
		UE_LOG(LogRoadPlacer, Warning, TEXT("Road material did not load; meshes will use an empty slot."));
	}

	FScopedSlowTask SlowTask(
		static_cast<float>(TilesX * TilesY),
		NSLOCTEXT("RoadPlacer", "PlaceProgress", "Placing road meshes..."));
	SlowTask.MakeDialog(true);

	for (int32 TY = 0; TY < TilesY; ++TY)
	{
		for (int32 TX = 0; TX < TilesX; ++TX)
		{
			SlowTask.EnterProgressFrame(1.0f);
			if (SlowTask.ShouldCancel())
			{
				Result.bCancelled = true;
				break;
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
				continue;
			}

			FRoadTin Tin;
			FString TinError;
			if (!RoadTriangulate::BuildTin(TileSamples, Masks, MaxEdge, Tin, TinError))
			{
				++Result.TilesSkipped;
				UE_LOG(LogRoadPlacer, Verbose, TEXT("Tile %d,%d: %s"), TX, TY, *TinError);
				continue;
			}

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

			TArray<FVector> WorldPts;
			TArray<int32> SlabTris;
			BuildSlabWorld(*Georeference, Tin.Vertices, Kept, HeightOff, Thickness, WorldPts, SlabTris);
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
			UStaticMesh* Mesh = RoadStaticMesh::CreatePersistentStaticMesh(
				MeshFolder, MeshLabel, LocalPts, SlabTris, Material, UvMeters, MeshError);
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
		}
		if (Result.bCancelled)
		{
			break;
		}
	}

	World->MarkPackageDirty();
	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	Result.bSuccess = Result.TilesSpawned > 0;
	if (!Result.bSuccess)
	{
		Result.Message = Result.bCancelled
			? TEXT("Cancelled before any road tiles were spawned.")
			: TEXT("No road StaticMeshActors were spawned. Check Max Edge Meters and that points sit inside the mask.");
		UE_LOG(LogRoadPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	Result.Message = FString::Printf(
		TEXT("Spawned %d road tile(s) (%d triangles) from %d mask polygon(s) and %d points. Elapsed: %.2fs."),
		Result.TilesSpawned,
		Result.TrianglesBuilt,
		Result.MaskPolygonsRead,
		Result.ElevationPointsRead,
		Result.ElapsedSeconds);
	UE_LOG(LogRoadPlacer, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogRoadPlacer, Display, TEXT("========== Road Place END =========="));
	return Result;
}
