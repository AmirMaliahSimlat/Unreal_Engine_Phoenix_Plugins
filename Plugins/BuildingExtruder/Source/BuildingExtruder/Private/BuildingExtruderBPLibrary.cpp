#include "BuildingExtruderBPLibrary.h"

#include "BuildingCesiumPlacement.h"
#include "BuildingCesiumTerrain.h"
#include "BuildingExtrudeUtils.h"
#include "BuildingExtruderLog.h"
#include "BuildingFbxExporter.h"
#include "BuildingShapefileReader.h"
#include "BuildingStaticMeshUtils.h"

#include "Cesium3DTileset.h"
#include "CesiumGeoreference.h"
#include "Editor.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "Internationalization/Internationalization.h"
#include "Materials/MaterialInterface.h"
#include "HAL/IConsoleManager.h"
#include "Misc/Paths.h"
#include "Misc/ScopedSlowTask.h"

static TAutoConsoleVariable<int32> CVarBuildingExtruderDiagnoseDtmLoad(
	TEXT("BuildingExtruder.DiagnoseDtmLoadConsistency"),
	0,
	TEXT("If 1, after normal DTM sampling re-sample with a longer refine (no timeout, 99%) and log ")
	TEXT("buildings whose floor min changes. Placement still uses the first sample. Default 0."),
	ECVF_Default);

namespace
{
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

	FVector2D FeatureCentroidLonLat(const FBuildingShapefileFeature& Feature)
	{
		FVector2D Sum(0, 0);
		for (const FVector2D& P : Feature.OuterRingLonLat)
		{
			Sum += P;
		}
		return Sum / static_cast<double>(FMath::Max(Feature.OuterRingLonLat.Num(), 1));
	}

	/**
	 * Chooses TX×TY with TX*TY == TargetTileCount (exact factor pair).
	 * Picks the pair whose cells are closest to square in meters.
	 */
	void ChooseSquareTileGrid(
		double MinLon,
		double MaxLon,
		double MinLat,
		double MaxLat,
		int32 TargetTileCount,
		int32& OutTilesX,
		int32& OutTilesY)
	{
		const int32 Target = FMath::Clamp(TargetTileCount <= 0 ? 64 : TargetTileCount, 1, 4096);
		const double LonSpan = FMath::Max(MaxLon - MinLon, 1.0e-9);
		const double LatSpan = FMath::Max(MaxLat - MinLat, 1.0e-9);
		const double MidLatRad = FMath::DegreesToRadians(0.5 * (MinLat + MaxLat));
		constexpr double MetersPerDegLat = 110540.0;
		const double MetersPerDegLon = FMath::Max(111320.0 * FMath::Cos(MidLatRad), 1.0);
		const double WidthM = LonSpan * MetersPerDegLon;
		const double HeightM = LatSpan * MetersPerDegLat;

		int32 BestX = Target;
		int32 BestY = 1;
		double BestAspectError = TNumericLimits<double>::Max();

		// Exact factor pairs only: for 24 → 1×24, 2×12, 3×8, 4×6, 6×4, 8×3, 12×2, 24×1.
		for (int32 Y = 1; Y <= Target; ++Y)
		{
			if (Target % Y != 0)
			{
				continue;
			}
			const int32 X = Target / Y;
			const double CellW = WidthM / static_cast<double>(X);
			const double CellH = HeightM / static_cast<double>(Y);
			const double CellAspect = CellW / FMath::Max(CellH, 1.0);
			const double AspectError = FMath::Abs(FMath::Loge(CellAspect)); // 0 when square
			if (AspectError < BestAspectError)
			{
				BestAspectError = AspectError;
				BestX = X;
				BestY = Y;
			}
		}

		OutTilesX = BestX;
		OutTilesY = BestY;
	}

	/** Parses "0,6,12" into a set of linear tile indices. Empty string → empty set (meaning all). */
	bool ParseTileIndicesFilter(const FString& TileIndices, TSet<int32>& OutIndices, FString& OutError)
	{
		OutIndices.Reset();
		FString Trimmed = TileIndices.TrimStartAndEnd();
		if (Trimmed.IsEmpty())
		{
			return true;
		}

		TArray<FString> Parts;
		Trimmed.ParseIntoArray(Parts, TEXT(","), /*bCullEmpty*/ true);
		for (FString& Part : Parts)
		{
			Part.TrimStartAndEndInline();
			if (Part.IsEmpty())
			{
				continue;
			}
			int32 Index = INDEX_NONE;
			if (!LexTryParseString(Index, *Part) || Index < 0)
			{
				OutError = FString::Printf(
					TEXT("TileIndices entry '%s' is not a non-negative integer. Use e.g. \"0,6,12\"."),
					*Part);
				return false;
			}
			OutIndices.Add(Index);
		}
		return true;
	}

	void AppendWorldMesh(
		FExtrudedPrismMesh& Combined,
		const FExtrudedPrismMesh& LocalMesh,
		const FVector& WorldOrigin)
	{
		const int32 VertexOffset = Combined.Vertices.Num();
		Combined.Vertices.Reserve(Combined.Vertices.Num() + LocalMesh.Vertices.Num());
		Combined.Normals.Reserve(Combined.Normals.Num() + LocalMesh.Normals.Num());
		Combined.UVs.Reserve(Combined.UVs.Num() + LocalMesh.UVs.Num());
		Combined.Triangles.Reserve(Combined.Triangles.Num() + LocalMesh.Triangles.Num());

		for (const FVector& V : LocalMesh.Vertices)
		{
			Combined.Vertices.Add(V + WorldOrigin);
		}
		Combined.Normals.Append(LocalMesh.Normals);
		Combined.UVs.Append(LocalMesh.UVs);
		for (const int32 Index : LocalMesh.Triangles)
		{
			Combined.Triangles.Add(Index + VertexOffset);
		}
	}

	bool BuildFeaturePrismWorld(
		ACesiumGeoreference& Georeference,
		const FBuildingShapefileFeature& Feature,
		double BaseAltitudeM,
		FExtrudedPrismMesh& OutWorldMesh,
		FVector& OutCentroid,
		FString& OutError)
	{
		const TArray<FVector2D>& Ring = Feature.OuterRingLonLat;
		if (Ring.Num() < 3)
		{
			OutError = TEXT("Ring too small.");
			return false;
		}

		const double TopHeightM = BaseAltitudeM + Feature.HeightM;

		TArray<FVector> BaseWorld;
		TArray<FVector> TopWorld;
		BaseWorld.Reserve(Ring.Num());
		TopWorld.Reserve(Ring.Num());

		FVector Origin(0, 0, 0);
		for (const FVector2D& LonLat : Ring)
		{
			const FVector BasePos = BuildingCesiumPlacement::LonLatHeightToUnreal(
				Georeference, LonLat.X, LonLat.Y, BaseAltitudeM);
			const FVector TopPos = BuildingCesiumPlacement::LonLatHeightToUnreal(
				Georeference, LonLat.X, LonLat.Y, TopHeightM);
			BaseWorld.Add(BasePos);
			TopWorld.Add(TopPos);
			Origin += BasePos;
		}
		Origin /= static_cast<double>(Ring.Num());
		OutCentroid = Origin;

		TArray<FVector> BaseLocal;
		TArray<FVector> TopLocal;
		BaseLocal.Reserve(Ring.Num());
		TopLocal.Reserve(Ring.Num());
		for (int32 I = 0; I < Ring.Num(); ++I)
		{
			BaseLocal.Add(BaseWorld[I] - Origin);
			TopLocal.Add(TopWorld[I] - Origin);
		}

		FExtrudedPrismMesh LocalMesh;
		if (!BuildingExtrudeUtils::BuildPrismFromRings(BaseLocal, TopLocal, LocalMesh, OutError))
		{
			return false;
		}

		OutWorldMesh = FExtrudedPrismMesh();
		AppendWorldMesh(OutWorldMesh, LocalMesh, Origin);
		return true;
	}

	bool SpawnTileStaticMeshActor(
		UWorld& World,
		const FExtrudedPrismMesh& WorldMesh,
		const FString& ActorLabel,
		const FString& EditorFolderPath,
		UMaterialInterface* Material,
		const TArray<FName>& ExtraTags,
		AStaticMeshActor*& OutActor,
		FString& OutError)
	{
		OutActor = nullptr;
		if (WorldMesh.Vertices.Num() < 3)
		{
			OutError = TEXT("Tile mesh empty.");
			return false;
		}

		FVector Origin(0, 0, 0);
		for (const FVector& V : WorldMesh.Vertices)
		{
			Origin += V;
		}
		Origin /= static_cast<double>(WorldMesh.Vertices.Num());

		FExtrudedPrismMesh LocalMesh;
		LocalMesh.Normals = WorldMesh.Normals;
		LocalMesh.UVs = WorldMesh.UVs;
		LocalMesh.Triangles = WorldMesh.Triangles;
		LocalMesh.Vertices.Reserve(WorldMesh.Vertices.Num());
		for (const FVector& V : WorldMesh.Vertices)
		{
			LocalMesh.Vertices.Add(V - Origin);
		}

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		SpawnParams.ObjectFlags |= RF_Transactional;

		AStaticMeshActor* Actor = World.SpawnActor<AStaticMeshActor>(Origin, FRotator::ZeroRotator, SpawnParams);
		if (!Actor)
		{
			OutError = TEXT("Failed to spawn StaticMeshActor.");
			return false;
		}

		Actor->SetActorLabel(ActorLabel);
		Actor->Tags.Add(FName(TEXT("BuildingExtruder")));
		for (const FName& Tag : ExtraTags)
		{
			Actor->Tags.AddUnique(Tag);
		}
		if (!EditorFolderPath.IsEmpty())
		{
			Actor->SetFolderPath(FName(*EditorFolderPath));
		}

		UStaticMesh* StaticMesh = BuildingStaticMeshUtils::CreateTransientStaticMesh(
			Actor,
			FName(*ActorLabel),
			LocalMesh,
			Material,
			OutError);
		if (!StaticMesh)
		{
			Actor->Destroy();
			return false;
		}

		UStaticMeshComponent* Comp = Actor->GetStaticMeshComponent();
		Comp->SetMobility(EComponentMobility::Static);
		Comp->SetStaticMesh(StaticMesh);
		Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Actor->MarkPackageDirty();
		OutActor = Actor;
		return true;
	}
}

FBuildingExtrudeResult UBuildingExtruderBPLibrary::ImportAndExtrudeBuildingsFromShapefile(
	UObject* WorldContextObject,
	const FString& ShapefilePath,
	const FString& FbxOutputPath,
	const FString& HeightFieldName,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath,
	int32 TargetTileCount,
	const FString& TileIndices,
	bool bEnableDtmLoadTimeout)
{
	FBuildingExtrudeResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const bool bDiagnoseDtmLoadConsistency = CVarBuildingExtruderDiagnoseDtmLoad.GetValueOnGameThread() != 0;

	const FString CleanInputPath = SanitizeFilePath(ShapefilePath);
	const FString CleanFbxPath = SanitizeFilePath(FbxOutputPath);

	UE_LOG(LogBuildingExtruder, Display, TEXT("========== Extrude START =========="));
	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("shp='%s' fbx='%s' heightField='%s' targetTiles=%d tileFilter='%s' dtmTimeout=%s diagnoseDtmLoad=%s (base Z from DTM min)"),
		*CleanInputPath,
		*CleanFbxPath,
		*HeightFieldName,
		TargetTileCount,
		*TileIndices,
		bEnableDtmLoadTimeout ? TEXT("ON") : TEXT("OFF"),
		bDiagnoseDtmLoadConsistency ? TEXT("ON") : TEXT("OFF"));

	if (CleanFbxPath.IsEmpty())
	{
		Result.Message = TEXT("FbxOutputPath is required (final deliverable).");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	UWorld* World = ResolveEditorWorld(WorldContextObject);
	if (!World)
	{
		Result.Message = TEXT("Could not resolve an editor world. Open a map first.");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	ACesiumGeoreference* Georeference = BuildingCesiumPlacement::FindGeoreference(World);
	if (!Georeference)
	{
		Result.Message = TEXT("No ACesiumGeoreference found in the level. Add Cesium Georeference (same one used by tilesets).");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	ACesium3DTileset* TerrainTileset = BuildingCesiumTerrain::FindTerrainTileset(World);
	if (!TerrainTileset)
	{
		Result.Message = TEXT("No ACesium3DTileset found for DTM sampling. Add Cesium World Terrain (or your DTM tileset) to the level.");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (CleanInputPath.IsEmpty())
	{
		Result.Message = TEXT("ShapefilePath is empty (provide a .shp path).");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (FPaths::GetExtension(CleanInputPath).Equals(TEXT("gpkg"), ESearchCase::IgnoreCase))
	{
		Result.Message = TEXT("GeoPackage (.gpkg) is not supported. Export to ESRI Shapefile (.shp/.dbf) and use that path.");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	const FString HeightField = HeightFieldName.IsEmpty() ? TEXT("RELATIVE_F") : HeightFieldName;

	TArray<FBuildingShapefileFeature> Features;
	FString ReadError;
	if (!BuildingShapefileReader::ReadPolygonBuildings(CleanInputPath, HeightField, TEXT(""), Features, ReadError))
	{
		Result.Message = ReadError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		UE_LOG(LogBuildingExtruder, Display, TEXT("========== Extrude END (read failed) =========="));
		return Result;
	}

	constexpr double MaxValidHeightM = 50.0;
	double ValidSum = 0.0;
	int32 ValidCount = 0;
	for (const FBuildingShapefileFeature& Feature : Features)
	{
		if (Feature.HeightM > 0.0 && Feature.HeightM <= MaxValidHeightM)
		{
			ValidSum += Feature.HeightM;
			++ValidCount;
		}
	}

	if (ValidCount == 0)
	{
		Result.Message = FString::Printf(
			TEXT("No valid %s values in (0, %.0f] m; cannot compute replacement average."),
			*HeightField,
			MaxValidHeightM);
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	const double AverageHeightM = ValidSum / static_cast<double>(ValidCount);
	int32 AnomalyCount = 0;
	for (FBuildingShapefileFeature& Feature : Features)
	{
		if (Feature.HeightM <= 0.0 || Feature.HeightM > MaxValidHeightM)
		{
			Feature.HeightM = AverageHeightM;
			++AnomalyCount;
		}
	}

	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("RELATIVE_F sanitize: valid=%d anomalies=%d average=%.3f m"),
		ValidCount,
		AnomalyCount,
		AverageHeightM);

	const int32 ToImport = Features.Num();
	UE_LOG(LogBuildingExtruder, Display, TEXT("Will process %d footprints"), ToImport);

	// Lon/lat bounds for tiling BEFORE DTM so tile indices match a full run with the same
	// TargetTileCount + dataset (linear index = TY * TilesX + TX).
	double MinLon = TNumericLimits<double>::Max();
	double MaxLon = TNumericLimits<double>::Lowest();
	double MinLat = TNumericLimits<double>::Max();
	double MaxLat = TNumericLimits<double>::Lowest();
	int32 BoundsCount = 0;
	for (int32 I = 0; I < ToImport; ++I)
	{
		if (Features[I].OuterRingLonLat.Num() < 3)
		{
			continue;
		}
		const FVector2D C = FeatureCentroidLonLat(Features[I]);
		MinLon = FMath::Min(MinLon, C.X);
		MaxLon = FMath::Max(MaxLon, C.X);
		MinLat = FMath::Min(MinLat, C.Y);
		MaxLat = FMath::Max(MaxLat, C.Y);
		++BoundsCount;
	}

	if (BoundsCount == 0)
	{
		Result.Message = TEXT("No buildings with valid footprints to place.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}
	const double LonSpan = FMath::Max(MaxLon - MinLon, 1.0e-9);
	const double LatSpan = FMath::Max(MaxLat - MinLat, 1.0e-9);

	int32 TilesX = 1;
	int32 TilesY = 1;
	ChooseSquareTileGrid(MinLon, MaxLon, MinLat, MaxLat, TargetTileCount, TilesX, TilesY);

	TSet<int32> SelectedTileIndices;
	FString TileFilterError;
	if (!ParseTileIndicesFilter(TileIndices, SelectedTileIndices, TileFilterError))
	{
		Result.Message = TileFilterError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	const int32 TileSlotCount = TilesX * TilesY;
	for (const int32 Idx : SelectedTileIndices)
	{
		if (Idx >= TileSlotCount)
		{
			Result.Message = FString::Printf(
				TEXT("Tile index %d is out of range for grid %dx%d (%d slots, indices 0..%d)."),
				Idx,
				TilesX,
				TilesY,
				TileSlotCount,
				TileSlotCount - 1);
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
			return Result;
		}
	}

	TArray<TArray<int32>> TileFeatureIndices;
	TileFeatureIndices.SetNum(TileSlotCount);
	for (int32 I = 0; I < ToImport; ++I)
	{
		if (Features[I].OuterRingLonLat.Num() < 3)
		{
			continue;
		}
		const FVector2D C = FeatureCentroidLonLat(Features[I]);
		int32 TX = FMath::FloorToInt(static_cast<float>((C.X - MinLon) / LonSpan * TilesX));
		int32 TY = FMath::FloorToInt(static_cast<float>((C.Y - MinLat) / LatSpan * TilesY));
		TX = FMath::Clamp(TX, 0, TilesX - 1);
		TY = FMath::Clamp(TY, 0, TilesY - 1);
		const int32 LinearIndex = TY * TilesX + TX;
		if (SelectedTileIndices.Num() > 0 && !SelectedTileIndices.Contains(LinearIndex))
		{
			continue;
		}
		TileFeatureIndices[LinearIndex].Add(I);
	}

	int32 NonEmptyTiles = 0;
	int32 BuildingsInSelectedTiles = 0;
	for (const TArray<int32>& Bucket : TileFeatureIndices)
	{
		if (Bucket.Num() > 0)
		{
			++NonEmptyTiles;
			BuildingsInSelectedTiles += Bucket.Num();
		}
	}

	if (SelectedTileIndices.Num() > 0)
	{
		TArray<int32> SortedFilter = SelectedTileIndices.Array();
		SortedFilter.Sort();
		FString FilterList;
		for (int32 I = 0; I < SortedFilter.Num(); ++I)
		{
			if (I > 0)
			{
				FilterList += TEXT(",");
			}
			FilterList += FString::FromInt(SortedFilter[I]);
		}
		UE_LOG(
			LogBuildingExtruder,
			Display,
			TEXT("Tile filter active: [%s] → %d non-empty selected tiles, %d buildings"),
			*FilterList,
			NonEmptyTiles,
			BuildingsInSelectedTiles);
	}

	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("Tiling: buildings=%d target=%d chosenGrid=%dx%d (%d slots) processingNonEmpty=%d bounds lon[%.6f,%.6f] lat[%.6f,%.6f]"),
		ToImport,
		TargetTileCount,
		TilesX,
		TilesY,
		TileSlotCount,
		NonEmptyTiles,
		MinLon,
		MaxLon,
		MinLat,
		MaxLat);

	if (NonEmptyTiles == 0)
	{
		Result.Message = SelectedTileIndices.Num() > 0
			? TEXT("No buildings found in the selected TileIndices.")
			: TEXT("No buildings assigned to any tile.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	// --- Sample DTM only for buildings in tiles we will process ---
	TArray<FVector> SamplePoints;
	TArray<int32> SamplePointTileIndices;
	TArray<int32> FeatureSampleStart;
	TArray<int32> FeatureSampleCount;
	FeatureSampleStart.Init(-1, ToImport);
	FeatureSampleCount.Init(0, ToImport);
	SamplePoints.Reserve(BuildingsInSelectedTiles * 8);
	SamplePointTileIndices.Reserve(BuildingsInSelectedTiles * 8);

	for (int32 TileIndex = 0; TileIndex < TileFeatureIndices.Num(); ++TileIndex)
	{
		for (const int32 FeatureIndex : TileFeatureIndices[TileIndex])
		{
			FeatureSampleStart[FeatureIndex] = SamplePoints.Num();
			for (const FVector2D& LonLat : Features[FeatureIndex].OuterRingLonLat)
			{
				SamplePoints.Add(FVector(LonLat.X, LonLat.Y, 0.0));
				SamplePointTileIndices.Add(TileIndex);
			}
			FeatureSampleCount[FeatureIndex] = SamplePoints.Num() - FeatureSampleStart[FeatureIndex];
		}
	}

	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("Sampling DTM at %d footprint vertices for %d buildings (timeout=%s)..."),
		SamplePoints.Num(),
		BuildingsInSelectedTiles,
		bEnableDtmLoadTimeout ? TEXT("ON") : TEXT("OFF"));

	// DTM-only sampling (other actors hidden / unticked / no collision inside SampleHeightsBlocking).
	constexpr int32 BatchSize = 64;
	const int32 NumBatches = FMath::DivideAndRoundUp(FMath::Max(SamplePoints.Num(), 1), BatchSize);
	constexpr float SampleTotalWork = 100.0f;
	constexpr float DtmDoneThresholdPercent = 95.0f;

	FScopedSlowTask SampleTask(
		SampleTotalWork,
		NSLOCTEXT("BuildingExtruder", "SampleDtm", "Sampling Cesium DTM heights..."));
	SampleTask.MakeDialog(true);

	TArray<double> SampleHeights;
	TArray<bool> SampleOk;
	SampleHeights.SetNum(SamplePoints.Num());
	SampleOk.SetNum(SamplePoints.Num());
	float SampleWorkDone = 0.0f;

	auto SetSampleProgress = [&SampleTask, &SampleWorkDone, SampleTotalWork](float Overall01)
	{
		Overall01 = FMath::Clamp(Overall01, 0.0f, 1.0f);
		const float Target = Overall01 * SampleTotalWork;
		const float Delta = Target - SampleWorkDone;
		if (Delta > KINDA_SMALL_NUMBER)
		{
			SampleTask.EnterProgressFrame(Delta);
			SampleWorkDone = Target;
		}
		else
		{
			SampleTask.EnterProgressFrame(0.0f);
		}
	};

	for (int32 Batch = 0; Batch < NumBatches; ++Batch)
	{
		if (SamplePoints.Num() == 0)
		{
			break;
		}

		SetSampleProgress(static_cast<float>(Batch) / static_cast<float>(FMath::Max(NumBatches, 1)));

		if (SampleTask.ShouldCancel())
		{
			Result.bCancelled = true;
			Result.Message = TEXT("Cancelled during DTM sampling.");
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogBuildingExtruder, Warning, TEXT("%s"), *Result.Message);
			return Result;
		}

		const int32 StartIdx = Batch * BatchSize;
		const int32 Count = FMath::Min(BatchSize, SamplePoints.Num() - StartIdx);
		TArray<FVector> BatchPoints;
		TArray<int32> BatchTileIndices;
		BatchPoints.Reserve(Count);
		BatchTileIndices.Reserve(Count);
		for (int32 I = 0; I < Count; ++I)
		{
			BatchPoints.Add(SamplePoints[StartIdx + I]);
			BatchTileIndices.Add(SamplePointTileIndices[StartIdx + I]);
		}

		auto UpdateDtmUi = [Batch, NumBatches, SetSampleProgress, DtmDoneThresholdPercent](
							   float LoadProgressPercent,
							   bool bWaitFinished,
							   bool /*bReachedTarget*/)
		{
			const float WithinBatch = bWaitFinished
				? 1.0f
				: FMath::Clamp(LoadProgressPercent / DtmDoneThresholdPercent, 0.0f, 1.0f);
			const float Overall01 =
				(static_cast<float>(Batch) + WithinBatch) / static_cast<float>(FMath::Max(NumBatches, 1));
			SetSampleProgress(Overall01);
		};

		TArray<double> BatchHeights;
		TArray<bool> BatchOk;
		FString SampleError;
		if (!BuildingCesiumTerrain::SampleHeightsBlocking(
				*World,
				*Georeference,
				*TerrainTileset,
				BatchPoints,
				BatchTileIndices,
				bEnableDtmLoadTimeout,
				DtmDoneThresholdPercent,
				BatchHeights,
				BatchOk,
				SampleError,
				UpdateDtmUi,
				[&SampleTask]() { return SampleTask.ShouldCancel(); }))
		{
			if (SampleError.Contains(TEXT("Cancelled")))
			{
				Result.bCancelled = true;
			}
			Result.Message = SampleError;
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			if (Result.bCancelled)
			{
				UE_LOG(LogBuildingExtruder, Warning, TEXT("%s"), *Result.Message);
			}
			else
			{
				UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
			}
			return Result;
		}

		for (int32 I = 0; I < Count; ++I)
		{
			SampleHeights[StartIdx + I] = BatchHeights[I];
			SampleOk[StartIdx + I] = BatchOk[I];
		}

		SetSampleProgress(static_cast<float>(Batch + 1) / static_cast<float>(FMath::Max(NumBatches, 1)));
	}

	SetSampleProgress(1.0f);

	// Lock DTM heights into features.
	int32 DtmFailBuildings = 0;
	for (int32 I = 0; I < ToImport; ++I)
	{
		if (FeatureSampleStart[I] < 0)
		{
			continue;
		}
		double MinH = TNumericLimits<double>::Max();
		int32 OkCount = 0;
		const int32 StartIdx = FeatureSampleStart[I];
		const int32 Count = FeatureSampleCount[I];
		for (int32 V = 0; V < Count; ++V)
		{
			const int32 Idx = StartIdx + V;
			if (SampleOk[Idx])
			{
				MinH = FMath::Min(MinH, SampleHeights[Idx]);
				++OkCount;
			}
		}
		if (OkCount == 0)
		{
			++DtmFailBuildings;
			Features[I].OuterRingLonLat.Reset();
		}
		else
		{
			Features[I].ElevationM = MinH;
		}
	}

	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("DTM base altitude: %d buildings OK, %d failed (no successful samples)"),
		BuildingsInSelectedTiles - DtmFailBuildings,
		DtmFailBuildings);

	// Prove load/LOD timing: re-sample with longer refine; log buildings whose floor min moves.
	// Placement still uses the first (normal) sample above.
	if (bDiagnoseDtmLoadConsistency && SamplePoints.Num() > 0)
	{
		UE_LOG(
			LogBuildingExtruder,
			Display,
			TEXT("DTM load diagnose: re-sampling with timeout=OFF doneThreshold=99%% (placement unchanged)..."));

		TArray<double> DeepHeights;
		TArray<bool> DeepOk;
		DeepHeights.SetNum(SamplePoints.Num());
		DeepOk.SetNum(SamplePoints.Num());

		FScopedSlowTask DiagTask(
			static_cast<float>(FMath::Max(NumBatches, 1)),
			NSLOCTEXT("BuildingExtruder", "DiagnoseDtmLoad", "Diagnose: longer DTM refine..."));
		DiagTask.MakeDialog(true);

		constexpr float DeepDonePercent = 99.0f;
		for (int32 Batch = 0; Batch < NumBatches; ++Batch)
		{
			DiagTask.EnterProgressFrame(1.0f);
			if (DiagTask.ShouldCancel())
			{
				Result.bCancelled = true;
				Result.Message = TEXT("Cancelled during DTM load diagnose.");
				Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
				UE_LOG(LogBuildingExtruder, Warning, TEXT("%s"), *Result.Message);
				return Result;
			}

			const int32 StartIdx = Batch * BatchSize;
			const int32 Count = FMath::Min(BatchSize, SamplePoints.Num() - StartIdx);
			TArray<FVector> BatchPoints;
			TArray<int32> BatchTileIndices;
			BatchPoints.Reserve(Count);
			BatchTileIndices.Reserve(Count);
			for (int32 I = 0; I < Count; ++I)
			{
				BatchPoints.Add(SamplePoints[StartIdx + I]);
				BatchTileIndices.Add(SamplePointTileIndices[StartIdx + I]);
			}

			TArray<double> BatchHeights;
			TArray<bool> BatchOk;
			FString SampleError;
			if (!BuildingCesiumTerrain::SampleHeightsBlocking(
					*World,
					*Georeference,
					*TerrainTileset,
					BatchPoints,
					BatchTileIndices,
					/*bEnableDtmLoadTimeout*/ false,
					DeepDonePercent,
					BatchHeights,
					BatchOk,
					SampleError,
					BuildingCesiumTerrain::FDtmProgressCallback(),
					[&DiagTask]() { return DiagTask.ShouldCancel(); }))
			{
				if (SampleError.Contains(TEXT("Cancelled")))
				{
					Result.bCancelled = true;
					Result.Message = SampleError;
					Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
					UE_LOG(LogBuildingExtruder, Warning, TEXT("%s"), *Result.Message);
					return Result;
				}
				UE_LOG(
					LogBuildingExtruder,
					Warning,
					TEXT("DTM load diagnose batch %d failed: %s (continuing)"),
					Batch + 1,
					*SampleError);
				continue;
			}

			for (int32 I = 0; I < Count; ++I)
			{
				DeepHeights[StartIdx + I] = BatchHeights[I];
				DeepOk[StartIdx + I] = BatchOk[I];
			}
		}

		constexpr double FloorDeltaEpsM = 0.05;
		constexpr double VertexSpreadWarnM = 2.0;
		int32 BuildingsFloorChanged = 0;
		int32 BuildingsWideSpread = 0;

		for (int32 I = 0; I < ToImport; ++I)
		{
			if (FeatureSampleStart[I] < 0 || Features[I].OuterRingLonLat.Num() < 3)
			{
				continue;
			}

			const int32 StartIdx = FeatureSampleStart[I];
			const int32 Count = FeatureSampleCount[I];
			const int32 TileIndex =
				SamplePointTileIndices.IsValidIndex(StartIdx) ? SamplePointTileIndices[StartIdx] : INDEX_NONE;

			double FirstMin = TNumericLimits<double>::Max();
			double FirstMax = TNumericLimits<double>::Lowest();
			int32 FirstOk = 0;
			double DeepMin = TNumericLimits<double>::Max();
			double DeepMax = TNumericLimits<double>::Lowest();
			int32 DeepOkCount = 0;

			for (int32 V = 0; V < Count; ++V)
			{
				const int32 Idx = StartIdx + V;
				if (SampleOk.IsValidIndex(Idx) && SampleOk[Idx])
				{
					FirstMin = FMath::Min(FirstMin, SampleHeights[Idx]);
					FirstMax = FMath::Max(FirstMax, SampleHeights[Idx]);
					++FirstOk;
				}
				if (DeepOk.IsValidIndex(Idx) && DeepOk[Idx])
				{
					DeepMin = FMath::Min(DeepMin, DeepHeights[Idx]);
					DeepMax = FMath::Max(DeepMax, DeepHeights[Idx]);
					++DeepOkCount;
				}
			}

			if (FirstOk > 0 && (FirstMax - FirstMin) > VertexSpreadWarnM)
			{
				++BuildingsWideSpread;
				UE_LOG(
					LogBuildingExtruder,
					Warning,
					TEXT("DTM load diagnose: record=%d tile=%d first-pass vertex spread=%.3fm (min=%.3f max=%.3f) "
						 "— possible mixed LOD on one footprint"),
					Features[I].RecordIndex,
					TileIndex,
					FirstMax - FirstMin,
					FirstMin,
					FirstMax);
			}

			if (FirstOk == 0 || DeepOkCount == 0)
			{
				continue;
			}

			const double FloorDelta = DeepMin - FirstMin;
			if (FMath::Abs(FloorDelta) <= FloorDeltaEpsM)
			{
				continue;
			}

			++BuildingsFloorChanged;
			UE_LOG(
				LogBuildingExtruder,
				Warning,
				TEXT("DTM load diagnose: record=%d tile=%d floorMin FIRST=%.3fm DEEP=%.3fm delta=%+.3fm "
					 "→ longer refine changed floor (load/LOD timing)"),
				Features[I].RecordIndex,
				TileIndex,
				FirstMin,
				DeepMin,
				FloorDelta);

			for (int32 V = 0; V < Count; ++V)
			{
				const int32 Idx = StartIdx + V;
				const bool bA = SampleOk.IsValidIndex(Idx) && SampleOk[Idx];
				const bool bB = DeepOk.IsValidIndex(Idx) && DeepOk[Idx];
				if (!bA && !bB)
				{
					continue;
				}
				const double HA = bA ? SampleHeights[Idx] : 0.0;
				const double HB = bB ? DeepHeights[Idx] : 0.0;
				const double Lon = SamplePoints[Idx].X;
				const double Lat = SamplePoints[Idx].Y;
				if (!bA || !bB || FMath::Abs(HB - HA) > FloorDeltaEpsM)
				{
					UE_LOG(
						LogBuildingExtruder,
						Warning,
						TEXT("  vertex[%d] lon=%.6f lat=%.6f first=%s deep=%s delta=%s"),
						V,
						Lon,
						Lat,
						bA ? *FString::Printf(TEXT("%.3fm"), HA) : TEXT("MISS"),
						bB ? *FString::Printf(TEXT("%.3fm"), HB) : TEXT("MISS"),
						(bA && bB) ? *FString::Printf(TEXT("%+.3fm"), HB - HA) : TEXT("n/a"));
				}
			}
		}

		UE_LOG(
			LogBuildingExtruder,
			Display,
			TEXT("DTM load diagnose summary: %d buildings floor-min changed after longer refine (eps=%.2fm); "
				 "%d buildings had first-pass vertex spread > %.1fm. "
				 "If floor-min changes > 0, float/sink is partly load/LOD timing."),
			BuildingsFloorChanged,
			FloorDeltaEpsM,
			BuildingsWideSpread,
			VertexSpreadWarnM);
	}

	FScopedSlowTask SlowTask(
		static_cast<float>(FMath::Max(NonEmptyTiles, 1)) + 1.0f,
		NSLOCTEXT("BuildingExtruder", "ImportProgress", "Extruding tiled buildings..."));
	SlowTask.MakeDialog(true);

	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("BldgTile") : ActorLabelPrefix;
	int32 BuildingsMeshed = 0;
	int32 BuildingsSkipped = 0;
	int32 TilesSpawned = 0;
	TArray<AStaticMeshActor*> SpawnedTileActors;
	SpawnedTileActors.Reserve(NonEmptyTiles);
	UMaterialInterface* CorrectMaterial = BuildingStaticMeshUtils::GetTwoSidedBuildingMaterial();
	TArray<FName> TileTags;
	TileTags.Add(FName(TEXT("BuildingExtruderTile")));

	for (int32 TileY = 0; TileY < TilesY; ++TileY)
	{
		for (int32 TileX = 0; TileX < TilesX; ++TileX)
		{
			const TArray<int32>& Bucket = TileFeatureIndices[TileY * TilesX + TileX];
			if (Bucket.Num() == 0)
			{
				continue;
			}

			const FString TileLabel = FString::Printf(TEXT("%s_%d_%d"), *LabelPrefix, TileX, TileY);
			SlowTask.EnterProgressFrame(
				1.0f,
				FText::Format(
					NSLOCTEXT("BuildingExtruder", "ImportProgressTile", "Tile {0} ({1} buildings)"),
					FText::FromString(TileLabel),
					FText::AsNumber(Bucket.Num())));

			if (SlowTask.ShouldCancel())
			{
				Result.bCancelled = true;
				Result.BuildingsSpawned = BuildingsMeshed;
				Result.BuildingsSkipped = BuildingsSkipped + (ToImport - BuildingsMeshed - BuildingsSkipped);
				Result.TilesSpawned = TilesSpawned;
				Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
				Result.Message = FString::Printf(
					TEXT("Cancelled. Tiles=%d buildings=%d. FBX not written. Elapsed: %.2fs."),
					TilesSpawned,
					BuildingsMeshed,
					Result.ElapsedSeconds);
				UE_LOG(LogBuildingExtruder, Warning, TEXT("%s"), *Result.Message);
				return Result;
			}

			FExtrudedPrismMesh TileWorldMesh;
			int32 TileBuildingCount = 0;
			for (const int32 FeatureIndex : Bucket)
			{
				FExtrudedPrismMesh BuildingWorldMesh;
				FVector Centroid;
				FString ExtrudeError;
				if (!BuildFeaturePrismWorld(
						*Georeference,
						Features[FeatureIndex],
						Features[FeatureIndex].ElevationM,
						BuildingWorldMesh,
						Centroid,
						ExtrudeError))
				{
					++BuildingsSkipped;
					UE_LOG(
						LogBuildingExtruder,
						Warning,
						TEXT("Skipped record %d: %s"),
						Features[FeatureIndex].RecordIndex,
						*ExtrudeError);
					continue;
				}

				AppendWorldMesh(TileWorldMesh, BuildingWorldMesh, FVector::ZeroVector);
				++BuildingsMeshed;
				++TileBuildingCount;
			}

			if (TileBuildingCount == 0)
			{
				continue;
			}

			AStaticMeshActor* SpawnedActor = nullptr;
			FString SpawnError;
			if (!SpawnTileStaticMeshActor(
					*World,
					TileWorldMesh,
					TileLabel,
					EditorFolderPath,
					CorrectMaterial,
					TileTags,
					SpawnedActor,
					SpawnError))
			{
				UE_LOG(LogBuildingExtruder, Error, TEXT("Tile %s failed: %s"), *TileLabel, *SpawnError);
				BuildingsSkipped += TileBuildingCount;
				BuildingsMeshed -= TileBuildingCount;
				continue;
			}
			SpawnedTileActors.Add(SpawnedActor);
			++TilesSpawned;
		}
	}


	SlowTask.EnterProgressFrame(1.0f, NSLOCTEXT("BuildingExtruder", "WriteFbx", "Writing FBX..."));

	FString WrittenFbxPath = CleanFbxPath;
	if (!WrittenFbxPath.EndsWith(TEXT(".fbx"), ESearchCase::IgnoreCase))
	{
		WrittenFbxPath += TEXT(".fbx");
	}

	if (TilesSpawned <= 0)
	{
		Result.BuildingsSpawned = 0;
		Result.BuildingsSkipped = BuildingsSkipped;
		Result.TilesSpawned = 0;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(
			TEXT("No tiles spawned (%d buildings skipped). FBX not written. Elapsed: %.2fs."),
			BuildingsSkipped,
			Result.ElapsedSeconds);
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	FString FbxError;
	if (!BuildingFbxExporter::ExportTileActors(*World, SpawnedTileActors, WrittenFbxPath, FbxError))
	{
		Result.BuildingsSpawned = BuildingsMeshed;
		Result.BuildingsSkipped = BuildingsSkipped;
		Result.TilesSpawned = TilesSpawned;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(
			TEXT("Spawned %d tiles (%d buildings) for editor preview, but FBX write failed: %s"),
			TilesSpawned,
			BuildingsMeshed,
			*FbxError);
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	Result.bSuccess = true;
	Result.BuildingsSpawned = BuildingsMeshed;
	Result.BuildingsSkipped = BuildingsSkipped;
	Result.TilesSpawned = TilesSpawned;
	Result.FbxOutputPath = WrittenFbxPath;
	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	Result.Message = FString::Printf(
		TEXT("Spawned %d tile StaticMeshActors (%d buildings, %d skipped), wrote FBX '%s'. Elapsed: %.2fs."),
		TilesSpawned,
		BuildingsMeshed,
		BuildingsSkipped,
		*WrittenFbxPath,
		Result.ElapsedSeconds);

	World->MarkPackageDirty();
	UE_LOG(LogBuildingExtruder, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogBuildingExtruder, Display, TEXT("========== Extrude END =========="));
	return Result;
}
