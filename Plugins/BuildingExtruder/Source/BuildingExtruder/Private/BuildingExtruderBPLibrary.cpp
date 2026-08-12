#include "BuildingExtruderBPLibrary.h"

#include "BuildingCesiumPlacement.h"
#include "BuildingExtrudeUtils.h"
#include "BuildingExtruderLog.h"
#include "BuildingFbxExporter.h"
#include "BuildingShapefileReader.h"
#include "BuildingStaticMeshUtils.h"

#include "CesiumGeoreference.h"
#include "Editor.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "Internationalization/Internationalization.h"
#include "Materials/MaterialInterface.h"
#include "Misc/Paths.h"
#include "Misc/ScopedSlowTask.h"

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
	 * Chooses TXxTY with TX*TY == TargetTileCount (exact factor pair).
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
			const double AspectError = FMath::Abs(FMath::Loge(CellAspect));
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

	/** Parses "0,6,12" into a set of linear tile indices. Empty string -> empty set (meaning all). */
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

	bool BuildFeaturePartsWorld(
		ACesiumGeoreference& Georeference,
		const FBuildingShapefileFeature& Feature,
		double BaseAltitudeM,
		double MetersPerUv,
		FExtrudedPrismMesh& OutWallsAndFloorWorld,
		FExtrudedPrismMesh& OutRoofWorld,
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

		FExtrudedPrismMesh LocalWalls;
		FExtrudedPrismMesh LocalRoof;
		if (!BuildingExtrudeUtils::BuildPrismPartsFromRings(
				BaseLocal,
				TopLocal,
				MetersPerUv,
				LocalWalls,
				LocalRoof,
				OutError))
		{
			return false;
		}

		OutWallsAndFloorWorld = FExtrudedPrismMesh();
		OutRoofWorld = FExtrudedPrismMesh();
		AppendWorldMesh(OutWallsAndFloorWorld, LocalWalls, Origin);
		AppendWorldMesh(OutRoofWorld, LocalRoof, Origin);
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

		const FString AssetName = ActorLabel;
		UStaticMesh* StaticMesh = BuildingStaticMeshUtils::CreatePersistentStaticMesh(
			TEXT("/Game/BuildingExtruder/Meshes"),
			AssetName,
			LocalMesh,
			Material,
			OutError);
		if (!StaticMesh)
		{
			return false;
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

		UStaticMeshComponent* Comp = Actor->GetStaticMeshComponent();
		Comp->SetMobility(EComponentMobility::Static);
		Comp->Modify();
		Comp->SetStaticMesh(StaticMesh);
		Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Actor->Modify();
		Actor->MarkPackageDirty();
		OutActor = Actor;
		return true;
	}
}

FBuildingExtrudeResult UBuildingExtruderBPLibrary::ImportAndExtrudeBuildingsFromShapefile(
	UObject* WorldContextObject,
	const FString& ShapefilePath,
	const FString& FbxOutputPath,
	const FString& AltitudeFieldName,
	const FString& HeightFieldName,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath,
	int32 TargetTileCount,
	const FString& TileIndices,
	float MetersPerUv)
{
	FBuildingExtrudeResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString AltitudeField = AltitudeFieldName.IsEmpty() ? TEXT("altitude") : AltitudeFieldName;
	const FString HeightField = HeightFieldName.IsEmpty() ? TEXT("RELATIVE_F") : HeightFieldName;
	const double UvMeters = FMath::Max(static_cast<double>(MetersPerUv), 0.01);

	const FString CleanInputPath = SanitizeFilePath(ShapefilePath);
	const FString CleanFbxPath = SanitizeFilePath(FbxOutputPath);

	UE_LOG(LogBuildingExtruder, Display, TEXT("========== Extrude START =========="));
	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("shp='%s' fbx='%s' altitudeField='%s' heightField='%s' targetTiles=%d tileFilter='%s' metersPerUv=%.3f"),
		*CleanInputPath,
		*CleanFbxPath,
		*AltitudeField,
		*HeightField,
		TargetTileCount,
		*TileIndices,
		UvMeters);

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

	TArray<FBuildingShapefileFeature> Features;
	FString ReadError;
	if (!BuildingShapefileReader::ReadPolygonBuildings(
			CleanInputPath, HeightField, AltitudeField, Features, ReadError))
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
		TEXT("Height sanitize (%s): valid=%d anomalies=%d average=%.3f m"),
		*HeightField,
		ValidCount,
		AnomalyCount,
		AverageHeightM);

	const int32 ToImport = Features.Num();
	UE_LOG(LogBuildingExtruder, Display, TEXT("Will process %d footprints"), ToImport);

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
			TEXT("Tile filter active: [%s] -> %d non-empty selected tiles, %d buildings"),
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

	{
		int32 AltOk = 0;
		double AltMin = TNumericLimits<double>::Max();
		double AltMax = TNumericLimits<double>::Lowest();
		for (int32 TileIndex = 0; TileIndex < TileFeatureIndices.Num(); ++TileIndex)
		{
			for (const int32 FeatureIndex : TileFeatureIndices[TileIndex])
			{
				const double Z = Features[FeatureIndex].ElevationM;
				AltMin = FMath::Min(AltMin, Z);
				AltMax = FMath::Max(AltMax, Z);
				++AltOk;
			}
		}
		UE_LOG(
			LogBuildingExtruder,
			Display,
			TEXT("Using shapefile altitude field '%s' for %d buildings. Z range [%.3f, %.3f] m"),
			*AltitudeField,
			AltOk,
			AltOk > 0 ? AltMin : 0.0,
			AltOk > 0 ? AltMax : 0.0);
	}

	const float SpawnTileWork = static_cast<float>(FMath::Max(NonEmptyTiles, 1));
	FScopedSlowTask SlowTask(
		SpawnTileWork + 1.0f,
		NSLOCTEXT("BuildingExtruder", "ImportProgress", "Extruding tiled buildings..."));
	SlowTask.MakeDialog(true);

	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("BldgTile") : ActorLabelPrefix;
	const FString FolderPath = EditorFolderPath;

	int32 BuildingsMeshed = 0;
	int32 BuildingsSkipped = 0;
	int32 TilesSpawned = 0;
	TArray<AStaticMeshActor*> SpawnedTileActors;
	SpawnedTileActors.Reserve(NonEmptyTiles * 2);
	UMaterialInterface* CorrectMaterial = nullptr;
	{
		FString MaterialError;
		CorrectMaterial = BuildingStaticMeshUtils::GetOrCreateBuildingMaterial(MaterialError);
		if (!CorrectMaterial)
		{
			Result.Message = MaterialError.IsEmpty()
				? TEXT("Failed to create /Game/BuildingExtruder building material.")
				: MaterialError;
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
			return Result;
		}
	}
	TArray<FName> WallsTags;
	WallsTags.Add(FName(TEXT("BuildingExtruderTile")));
	WallsTags.Add(FName(TEXT("BuildingExtruderWalls")));
	TArray<FName> RoofTags;
	RoofTags.Add(FName(TEXT("BuildingExtruderTile")));
	RoofTags.Add(FName(TEXT("BuildingExtruderRoof")));

	const FString WallsFolder = FolderPath.IsEmpty() ? FString(TEXT("Walls")) : (FolderPath + TEXT("/Walls"));
	const FString RoofFolder = FolderPath.IsEmpty() ? FString(TEXT("Roofs")) : (FolderPath + TEXT("/Roofs"));

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

			FExtrudedPrismMesh TileWallsMesh;
			FExtrudedPrismMesh TileRoofMesh;
			int32 TileBuildingCount = 0;
			for (const int32 FeatureIndex : Bucket)
			{
				FExtrudedPrismMesh BuildingWalls;
				FExtrudedPrismMesh BuildingRoof;
				FVector Centroid;
				FString ExtrudeError;
				if (!BuildFeaturePartsWorld(
						*Georeference,
						Features[FeatureIndex],
						Features[FeatureIndex].ElevationM,
						UvMeters,
						BuildingWalls,
						BuildingRoof,
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

				AppendWorldMesh(TileWallsMesh, BuildingWalls, FVector::ZeroVector);
				AppendWorldMesh(TileRoofMesh, BuildingRoof, FVector::ZeroVector);
				++BuildingsMeshed;
				++TileBuildingCount;
			}

			if (TileBuildingCount == 0)
			{
				continue;
			}

			const FString WallsLabel = TileLabel + TEXT("_Walls");
			const FString RoofLabel = TileLabel + TEXT("_Roof");

			AStaticMeshActor* WallsActor = nullptr;
			AStaticMeshActor* RoofActor = nullptr;
			FString SpawnError;
			if (!SpawnTileStaticMeshActor(
					*World,
					TileWallsMesh,
					WallsLabel,
					WallsFolder,
					CorrectMaterial,
					WallsTags,
					WallsActor,
					SpawnError))
			{
				UE_LOG(LogBuildingExtruder, Error, TEXT("Tile %s walls failed: %s"), *TileLabel, *SpawnError);
				BuildingsSkipped += TileBuildingCount;
				BuildingsMeshed -= TileBuildingCount;
				continue;
			}
			if (!SpawnTileStaticMeshActor(
					*World,
					TileRoofMesh,
					RoofLabel,
					RoofFolder,
					CorrectMaterial,
					RoofTags,
					RoofActor,
					SpawnError))
			{
				UE_LOG(LogBuildingExtruder, Error, TEXT("Tile %s roof failed: %s"), *TileLabel, *SpawnError);
				if (WallsActor)
				{
					WallsActor->Destroy();
				}
				BuildingsSkipped += TileBuildingCount;
				BuildingsMeshed -= TileBuildingCount;
				continue;
			}

			SpawnedTileActors.Add(WallsActor);
			SpawnedTileActors.Add(RoofActor);
			++TilesSpawned;
		}
	}

	SlowTask.EnterProgressFrame(1.0f, NSLOCTEXT("BuildingExtruder", "WriteFbx", "Writing FBX..."));

	FString WrittenFbxPath = CleanFbxPath;
	if (!WrittenFbxPath.EndsWith(TEXT(".fbx"), ESearchCase::IgnoreCase))
	{
		WrittenFbxPath += TEXT(".fbx");
	}

	if (SpawnedTileActors.Num() <= 0)
	{
		Result.BuildingsSpawned = 0;
		Result.BuildingsSkipped = BuildingsSkipped;
		Result.TilesSpawned = TilesSpawned;
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
		TEXT("Spawned %d tiles (%d walls + %d roof actors, %d buildings, %d skipped), "
			 "saved meshes under /Game/BuildingExtruder/Meshes, wrote FBX '%s'. Elapsed: %.2fs."),
		TilesSpawned,
		TilesSpawned,
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
