#include "TreePlacerBPLibrary.h"

#include "TreeCesiumPlacement.h"
#include "TreeMeshFolderLoader.h"
#include "TreePlacerLog.h"
#include "TreePlacerTileActor.h"
#include "TreeShapefileReader.h"

#include "CesiumGeoreference.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Editor.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "Internationalization/Internationalization.h"
#include "Misc/ScopedSlowTask.h"
#include "StaticMeshResources.h"

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

	int32 GetMeshMaxLODIndex(const UStaticMesh* Mesh)
	{
		if (!Mesh)
		{
			return 0;
		}
		return FMath::Max(Mesh->GetNumLODs() - 1, 0);
	}

	int32 ResolveDisappearLOD(int32 RequestedLOD, int32 MaxLOD)
	{
		// Negative or past the last available LOD → disappear at the mesh maximum LOD.
		if (RequestedLOD < 0 || RequestedLOD > MaxLOD)
		{
			return MaxLOD;
		}
		return RequestedLOD;
	}

	float GetMeshLODScreenSize(const UStaticMesh* Mesh, int32 LODIndex)
	{
		if (!Mesh || LODIndex < 0 || LODIndex >= Mesh->GetNumLODs())
		{
			return 0.01f;
		}

		// Prefer cooked render data screen sizes when available.
		// UE 5.1: ScreenSize is a fixed array (not TArray), so no IsValidIndex.
		if (const FStaticMeshRenderData* RenderData = Mesh->GetRenderData())
		{
			const float ScreenSize = RenderData->ScreenSize[LODIndex].Default;
			if (ScreenSize > KINDA_SMALL_NUMBER)
			{
				return ScreenSize;
			}
		}

#if WITH_EDITORONLY_DATA
		if (Mesh->GetNumSourceModels() > LODIndex)
		{
			const float ScreenSize = Mesh->GetSourceModel(LODIndex).ScreenSize.Default;
			if (ScreenSize > KINDA_SMALL_NUMBER)
			{
				return ScreenSize;
			}
		}
#endif

		// Fallback: each successive LOD roughly halves screen size.
		return FMath::Pow(0.5f, static_cast<float>(LODIndex));
	}

	/** Approximate camera distance (cm) where the mesh would switch to the given LOD. */
	int32 EstimateCullDistanceForLOD(const UStaticMesh* Mesh, int32 LODIndex)
	{
		if (!Mesh)
		{
			return 0;
		}

		const float Radius = FMath::Max(Mesh->GetBounds().SphereRadius, 1.0f);
		float ScreenSize = GetMeshLODScreenSize(Mesh, LODIndex);
		// LOD0 screen size is often ~1.0 (near camera). Clamping avoids near-zero cull
		// distances when disappearing at early/max LODs on simple meshes.
		ScreenSize = FMath::Clamp(ScreenSize, 1.0e-4f, 0.08f);
		const float DistanceCm = Radius / ScreenSize;
		return FMath::Max(FMath::CeilToInt(DistanceCm), 1);
	}

	void ConfigureHismCullDistance(UHierarchicalInstancedStaticMeshComponent* HISM, int32 EndCullDistanceCm)
	{
		if (!HISM)
		{
			return;
		}
		const int32 EndDist = FMath::Max(EndCullDistanceCm, 0);
		const int32 StartDist = (EndDist > 0)
			? FMath::Max(0, EndDist - FMath::Max(EndDist / 20, 100))
			: 0;
		HISM->InstanceStartCullDistance = StartDist;
		HISM->InstanceEndCullDistance = EndDist;
	}

	UHierarchicalInstancedStaticMeshComponent* CreateTreeHism(
		AActor& Actor,
		USceneComponent& Root,
		UStaticMesh* Mesh,
		FName CompName,
		bool bVisibleMesh,
		bool bCastShadows,
		int32 EndCullDistanceCm,
		const TArray<FTransform>& WorldTransforms)
	{
		UHierarchicalInstancedStaticMeshComponent* HISM = NewObject<UHierarchicalInstancedStaticMeshComponent>(
			&Actor,
			CompName,
			RF_Transactional);
		HISM->CreationMethod = EComponentCreationMethod::Instance;
		HISM->SetupAttachment(&Root);
		HISM->SetStaticMesh(Mesh);
		HISM->SetMobility(EComponentMobility::Static);
		HISM->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		HISM->SetGenerateOverlapEvents(false);
		HISM->bHasPerInstanceHitProxies = false;

		HISM->SetVisibility(bVisibleMesh, /*bPropagateToChildren*/ false);
		HISM->SetHiddenInGame(!bVisibleMesh);
		HISM->SetCastShadow(bCastShadows);
		HISM->bCastHiddenShadow = bCastShadows && !bVisibleMesh;

		ConfigureHismCullDistance(HISM, EndCullDistanceCm);

		Actor.AddInstanceComponent(HISM);
		HISM->RegisterComponent();

		for (const FTransform& Xform : WorldTransforms)
		{
			HISM->AddInstance(Xform, /*bWorldSpace*/ true);
		}
#if ENGINE_MAJOR_VERSION >= 5
		HISM->BuildTreeIfOutdated(/*bAsync*/ false, /*bForceUpdate*/ true);
#endif
		HISM->MarkRenderStateDirty();
		HISM->Modify();
		return HISM;
	}

	bool SpawnTreeTileActor(
		UWorld& World,
		const FString& ActorLabel,
		const FString& EditorFolderPath,
		const TArray<UStaticMesh*>& TreeMeshes,
		const TArray<TArray<FTransform>>& TransformsPerMesh,
		const FVector& TileOrigin,
		int32 TreeDisappearLOD,
		int32 ShadowDisappearLOD,
		AActor*& OutActor,
		FString& OutError)
	{
		OutActor = nullptr;

		int32 TotalInstances = 0;
		for (const TArray<FTransform>& Transforms : TransformsPerMesh)
		{
			TotalInstances += Transforms.Num();
		}
		if (TotalInstances == 0)
		{
			OutError = TEXT("Tile has no tree instances.");
			return false;
		}

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		SpawnParams.ObjectFlags |= RF_Transactional;

		ATreePlacerTileActor* Actor = World.SpawnActor<ATreePlacerTileActor>(TileOrigin, FRotator::ZeroRotator, SpawnParams);
		if (!Actor)
		{
			OutError = TEXT("Failed to spawn tile actor.");
			return false;
		}

		Actor->SetActorLabel(ActorLabel);
		Actor->Tags.Add(FName(TEXT("TreePlacer")));
		Actor->Tags.Add(FName(TEXT("TreePlacerTile")));
		if (!EditorFolderPath.IsEmpty())
		{
			Actor->SetFolderPath(FName(*EditorFolderPath));
		}

		USceneComponent* Root = Actor->GetRootComponent();
		if (!Root)
		{
			OutError = TEXT("Tile actor missing root component.");
			Actor->Destroy();
			return false;
		}
		Root->SetWorldLocation(TileOrigin);

		for (int32 MeshIndex = 0; MeshIndex < TreeMeshes.Num(); ++MeshIndex)
		{
			const TArray<FTransform>& Transforms = TransformsPerMesh[MeshIndex];
			UStaticMesh* Mesh = TreeMeshes[MeshIndex];
			if (Transforms.Num() == 0 || !Mesh)
			{
				continue;
			}

			const int32 MaxLOD = GetMeshMaxLODIndex(Mesh);
			const int32 TreeLOD = ResolveDisappearLOD(TreeDisappearLOD, MaxLOD);
			const int32 ShadowLOD = ResolveDisappearLOD(ShadowDisappearLOD, MaxLOD);
			const int32 TreeCullCm = EstimateCullDistanceForLOD(Mesh, TreeLOD);
			const int32 ShadowCullCm = EstimateCullDistanceForLOD(Mesh, ShadowLOD);

			UE_LOG(
				LogTreePlacer,
				Verbose,
				TEXT("HISM mesh '%s': maxLOD=%d treeLOD=%d (cull~%dcm) shadowLOD=%d (cull~%dcm)"),
				*Mesh->GetName(),
				MaxLOD,
				TreeLOD,
				TreeCullCm,
				ShadowLOD,
				ShadowCullCm);

			if (TreeCullCm == ShadowCullCm)
			{
				// Same cutoff: one component draws mesh + shadows.
				CreateTreeHism(
					*Actor,
					*Root,
					Mesh,
					*FString::Printf(TEXT("HISM_%d"), MeshIndex),
					/*bVisibleMesh*/ true,
					/*bCastShadows*/ true,
					TreeCullCm,
					Transforms);
			}
			else
			{
				// Split so mesh and shadows can disappear at different LODs/distances.
				CreateTreeHism(
					*Actor,
					*Root,
					Mesh,
					*FString::Printf(TEXT("HISM_%d_Mesh"), MeshIndex),
					/*bVisibleMesh*/ true,
					/*bCastShadows*/ false,
					TreeCullCm,
					Transforms);
				CreateTreeHism(
					*Actor,
					*Root,
					Mesh,
					*FString::Printf(TEXT("HISM_%d_Shadow"), MeshIndex),
					/*bVisibleMesh*/ false,
					/*bCastShadows*/ true,
					ShadowCullCm,
					Transforms);
			}
		}

		Actor->Modify();
		Actor->MarkPackageDirty();
		OutActor = Actor;
		return true;
	}
}

FTreePlaceResult UTreePlacerBPLibrary::PlaceTreesFromShapefile(
	UObject* WorldContextObject,
	const FString& ShapefilePath,
	const FString& TreeMeshFolder,
	const FString& AltitudeFieldName,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath,
	int32 TargetTileCount,
	const FString& TileIndices,
	int32 RandomSeed,
	int32 TreeDisappearLOD,
	int32 ShadowDisappearLOD)
{
	FTreePlaceResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString AltitudeField = AltitudeFieldName.IsEmpty() ? TEXT("altitude") : AltitudeFieldName;
	const FString CleanInputPath = SanitizeFilePath(ShapefilePath);

	UE_LOG(LogTreePlacer, Display, TEXT("========== Tree Place START =========="));
	UE_LOG(
		LogTreePlacer,
		Display,
		TEXT("shp='%s' meshes='%s' altitudeField='%s' targetTiles=%d tileFilter='%s' seed=%d treeLOD=%d shadowLOD=%d"),
		*CleanInputPath,
		*TreeMeshFolder,
		*AltitudeField,
		TargetTileCount,
		*TileIndices,
		RandomSeed,
		TreeDisappearLOD,
		ShadowDisappearLOD);

	UWorld* World = ResolveEditorWorld(WorldContextObject);
	if (!World)
	{
		Result.Message = TEXT("Could not resolve an editor world. Open a map first.");
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	ACesiumGeoreference* Georeference = TreeCesiumPlacement::FindGeoreference(World);
	if (!Georeference)
	{
		Result.Message = TEXT("No ACesiumGeoreference found in the level.");
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (CleanInputPath.IsEmpty())
	{
		Result.Message = TEXT("ShapefilePath is empty (provide a .shp path).");
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<UStaticMesh*> TreeMeshes;
	FString MeshError;
	if (!TreeMeshFolderLoader::LoadTreeMeshesFromFolder(TreeMeshFolder, TreeMeshes, MeshError))
	{
		Result.Message = MeshError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<FTreeShapefilePoint> Points;
	FString ReadError;
	if (!TreeShapefileReader::ReadPoints(CleanInputPath, AltitudeField, Points, ReadError))
	{
		Result.Message = ReadError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	const int32 PointCount = Points.Num();
	double MinLon = TNumericLimits<double>::Max();
	double MaxLon = TNumericLimits<double>::Lowest();
	double MinLat = TNumericLimits<double>::Max();
	double MaxLat = TNumericLimits<double>::Lowest();
	for (const FTreeShapefilePoint& Point : Points)
	{
		MinLon = FMath::Min(MinLon, Point.LonDeg);
		MaxLon = FMath::Max(MaxLon, Point.LonDeg);
		MinLat = FMath::Min(MinLat, Point.LatDeg);
		MaxLat = FMath::Max(MaxLat, Point.LatDeg);
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
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
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
			UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
			return Result;
		}
	}

	TArray<TArray<int32>> TilePointIndices;
	TilePointIndices.SetNum(TileSlotCount);
	for (int32 I = 0; I < PointCount; ++I)
	{
		const FTreeShapefilePoint& Point = Points[I];
		int32 TX = FMath::FloorToInt(static_cast<float>((Point.LonDeg - MinLon) / LonSpan * TilesX));
		int32 TY = FMath::FloorToInt(static_cast<float>((Point.LatDeg - MinLat) / LatSpan * TilesY));
		TX = FMath::Clamp(TX, 0, TilesX - 1);
		TY = FMath::Clamp(TY, 0, TilesY - 1);
		const int32 LinearIndex = TY * TilesX + TX;
		if (SelectedTileIndices.Num() > 0 && !SelectedTileIndices.Contains(LinearIndex))
		{
			continue;
		}
		TilePointIndices[LinearIndex].Add(I);
	}

	int32 NonEmptyTiles = 0;
	int32 PointsInSelectedTiles = 0;
	for (const TArray<int32>& Bucket : TilePointIndices)
	{
		if (Bucket.Num() > 0)
		{
			++NonEmptyTiles;
			PointsInSelectedTiles += Bucket.Num();
		}
	}

	UE_LOG(
		LogTreePlacer,
		Display,
		TEXT("Tiling: points=%d target=%d chosenGrid=%dx%d (%d slots) processingNonEmpty=%d pointsInTiles=%d"),
		PointCount,
		TargetTileCount,
		TilesX,
		TilesY,
		TileSlotCount,
		NonEmptyTiles,
		PointsInSelectedTiles);

	if (NonEmptyTiles == 0)
	{
		Result.Message = SelectedTileIndices.Num() > 0
			? TEXT("No points found in the selected TileIndices.")
			: TEXT("No points assigned to any tile.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	FRandomStream Rng;
	if (RandomSeed == 0)
	{
		Rng.GenerateNewSeed();
	}
	else
	{
		Rng.Initialize(RandomSeed);
	}

	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("TreeTile") : ActorLabelPrefix;
	const FString FolderPath = EditorFolderPath;

	FScopedSlowTask SlowTask(
		static_cast<float>(NonEmptyTiles),
		NSLOCTEXT("TreePlacer", "PlaceProgress", "Placing tiled trees..."));
	SlowTask.MakeDialog(true);

	int32 TreesPlaced = 0;
	int32 TreesSkipped = 0;
	int32 TilesSpawned = 0;

	for (int32 TileY = 0; TileY < TilesY; ++TileY)
	{
		for (int32 TileX = 0; TileX < TilesX; ++TileX)
		{
			const TArray<int32>& Bucket = TilePointIndices[TileY * TilesX + TileX];
			if (Bucket.Num() == 0)
			{
				continue;
			}

			const FString TileLabel = FString::Printf(TEXT("%s_%d_%d"), *LabelPrefix, TileX, TileY);
			SlowTask.EnterProgressFrame(
				1.0f,
				FText::Format(
					NSLOCTEXT("TreePlacer", "PlaceProgressTile", "Tile {0} ({1} trees)"),
					FText::FromString(TileLabel),
					FText::AsNumber(Bucket.Num())));

			if (SlowTask.ShouldCancel())
			{
				Result.bCancelled = true;
				Result.TreesPlaced = TreesPlaced;
				Result.TreesSkipped = TreesSkipped;
				Result.TilesSpawned = TilesSpawned;
				Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
				Result.Message = FString::Printf(
					TEXT("Cancelled. Tiles=%d trees=%d. Elapsed: %.2fs."),
					TilesSpawned,
					TreesPlaced,
					Result.ElapsedSeconds);
				UE_LOG(LogTreePlacer, Warning, TEXT("%s"), *Result.Message);
				return Result;
			}

			TArray<TArray<FTransform>> TransformsPerMesh;
			TransformsPerMesh.SetNum(TreeMeshes.Num());
			FVector OriginSum = FVector::ZeroVector;
			int32 OriginCount = 0;

			for (const int32 PointIndex : Bucket)
			{
				const FTreeShapefilePoint& Point = Points[PointIndex];
				const FVector WorldPos = TreeCesiumPlacement::LonLatHeightToUnreal(
					*Georeference,
					Point.LonDeg,
					Point.LatDeg,
					Point.AltitudeM);

				const int32 MeshIndex = Rng.RandRange(0, TreeMeshes.Num() - 1);
				const float YawDeg = Rng.FRandRange(0.0f, 360.0f);
				const FTransform WorldXform(
					FRotator(0.0, YawDeg, 0.0),
					WorldPos,
					FVector::OneVector);

				TransformsPerMesh[MeshIndex].Add(WorldXform);
				OriginSum += WorldPos;
				++OriginCount;
			}

			if (OriginCount == 0)
			{
				continue;
			}

			const FVector TileOrigin = OriginSum / static_cast<double>(OriginCount);

			// Keep transforms in world space (identical Cesium LonLatHeight -> Unreal as buildings).
			// HISM AddInstance(..., bWorldSpace=true) converts to component space.

			AActor* TileActor = nullptr;
			FString SpawnError;
			if (!SpawnTreeTileActor(
					*World,
					TileLabel,
					FolderPath,
					TreeMeshes,
					TransformsPerMesh,
					TileOrigin,
					TreeDisappearLOD,
					ShadowDisappearLOD,
					TileActor,
					SpawnError))
			{
				UE_LOG(LogTreePlacer, Error, TEXT("Tile %s failed: %s"), *TileLabel, *SpawnError);
				TreesSkipped += Bucket.Num();
				continue;
			}

			int32 TileTrees = 0;
			for (const TArray<FTransform>& Transforms : TransformsPerMesh)
			{
				TileTrees += Transforms.Num();
			}
			TreesPlaced += TileTrees;
			++TilesSpawned;
		}
	}

	if (TilesSpawned <= 0)
	{
		Result.TreesPlaced = 0;
		Result.TreesSkipped = TreesSkipped;
		Result.TilesSpawned = 0;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(
			TEXT("No tiles spawned (%d trees skipped). Elapsed: %.2fs."),
			TreesSkipped,
			Result.ElapsedSeconds);
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	Result.bSuccess = true;
	Result.TreesPlaced = TreesPlaced;
	Result.TreesSkipped = TreesSkipped;
	Result.TilesSpawned = TilesSpawned;
	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	Result.Message = FString::Printf(
		TEXT("Spawned %d tree tiles (%d trees, %d skipped) using %d mesh types from '%s'. Elapsed: %.2fs."),
		TilesSpawned,
		TreesPlaced,
		TreesSkipped,
		TreeMeshes.Num(),
		*TreeMeshFolder,
		Result.ElapsedSeconds);

	World->MarkPackageDirty();
	UE_LOG(LogTreePlacer, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogTreePlacer, Display, TEXT("========== Tree Place END =========="));
	return Result;
}
