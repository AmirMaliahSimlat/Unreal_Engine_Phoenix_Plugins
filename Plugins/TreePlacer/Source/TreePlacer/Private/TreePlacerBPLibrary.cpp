#include "TreePlacerBPLibrary.h"

#include "TreeCesiumPlacement.h"
#include "TreeMeshFolderLoader.h"
#include "TreePlacerLog.h"
#include "TreeShapefileReader.h"

#include "CesiumGeoreference.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Editor.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "FoliageType.h"
#include "FoliageType_InstancedStaticMesh.h"
#include "HAL/PlatformTime.h"
#include "InstancedFoliage.h"
#include "InstancedFoliageActor.h"
#include "Internationalization/Internationalization.h"
#include "Misc/ScopedSlowTask.h"
#include "Runtime/Launch/Resources/Version.h"

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

	/** Convert meters to Unreal cull cm. <= 0 means no distance culling (UE uses 0). */
	int32 MetersToCullDistanceCm(float DistanceMeters)
	{
		if (DistanceMeters <= 0.0f)
		{
			return 0;
		}
		return FMath::Max(FMath::CeilToInt(DistanceMeters * 100.0f), 1);
	}

	void ConfigureHismCullDistance(UHierarchicalInstancedStaticMeshComponent* HISM, int32 EndCullDistanceCm)
	{
		if (!HISM)
		{
			return;
		}
		const int32 EndDist = FMath::Max(EndCullDistanceCm, 0);
		// Fade starts at 35% of the end distance before full cull (min 1 m band).
		const int32 StartDist = (EndDist > 0)
			? FMath::Max(0, EndDist - FMath::Max((EndDist * 35) / 100, 100))
			: 0;
		HISM->InstanceStartCullDistance = StartDist;
		HISM->InstanceEndCullDistance = EndDist;
	}

	void ApplyFoliageCullDistance(
		UFoliageType* FoliageType,
		FFoliageInfo* FoliageInfo,
		int32 TreeCullCm)
	{
		if (!FoliageType)
		{
			return;
		}

		const int32 EndDist = FMath::Max(TreeCullCm, 0);
		const int32 StartDist = (EndDist > 0)
			? FMath::Max(0, EndDist - FMath::Max((EndDist * 35) / 100, 100))
			: 0;
		FoliageType->CullDistance.Min = StartDist;
		FoliageType->CullDistance.Max = EndDist;
		FoliageType->bCastShadow = true;
		FoliageType->AlignToNormal = false;
		FoliageType->RandomYaw = false;
		FoliageType->Modify();

		if (FoliageInfo)
		{
			if (UHierarchicalInstancedStaticMeshComponent* HISM =
					Cast<UHierarchicalInstancedStaticMeshComponent>(FoliageInfo->GetComponent()))
			{
				ConfigureHismCullDistance(HISM, TreeCullCm);
				HISM->SetCastShadow(true);
				HISM->MarkRenderStateDirty();
			}
		}
	}

	struct FTreeFoliageSlot
	{
		UFoliageType* Type = nullptr;
		FFoliageInfo* Info = nullptr;
	};

	UFoliageType* FindFoliageTypeForMesh(AInstancedFoliageActor& IFA, const UStaticMesh* Mesh)
	{
		UFoliageType* Found = nullptr;
		IFA.ForEachFoliageInfo([&](UFoliageType* Type, FFoliageInfo& /*Info*/)
		{
			if (!Type || !Mesh)
			{
				return true;
			}
			if (Type->GetSource() == Mesh)
			{
				Found = Type;
				return false;
			}
			if (const UFoliageType_InstancedStaticMesh* ISMType = Cast<UFoliageType_InstancedStaticMesh>(Type))
			{
				if (ISMType->GetStaticMesh() == Mesh)
				{
					Found = Type;
					return false;
				}
			}
			return true;
		});
		return Found;
	}

	bool GetOrCreateFoliageSlot(
		AInstancedFoliageActor& IFA,
		UStaticMesh* Mesh,
		int32 TreeCullCm,
		FTreeFoliageSlot& OutSlot,
		FString& OutError)
	{
		OutSlot = FTreeFoliageSlot();
		if (!Mesh)
		{
			OutError = TEXT("Null tree mesh.");
			return false;
		}

		UFoliageType* Type = FindFoliageTypeForMesh(IFA, Mesh);
		FFoliageInfo* Info = Type ? IFA.FindInfo(Type) : nullptr;
		if (!Type || !Info)
		{
			Type = nullptr;
			Info = IFA.AddMesh(Mesh, &Type);
		}
		if (!Type || !Info)
		{
			OutError = FString::Printf(TEXT("Failed to add foliage type for mesh '%s'."), *Mesh->GetName());
			return false;
		}

		ApplyFoliageCullDistance(Type, Info, TreeCullCm);
		OutSlot.Type = Type;
		OutSlot.Info = Info;
		return true;
	}

	bool AddFoliageInstances(
		AInstancedFoliageActor& IFA,
		FTreeFoliageSlot& Slot,
		const TArray<FTransform>& WorldTransforms)
	{
		if (!Slot.Type || !Slot.Info || WorldTransforms.Num() == 0)
		{
			return WorldTransforms.Num() == 0;
		}

#if ENGINE_MAJOR_VERSION > 5 || (ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION >= 2)
		TArray<FFoliageInstance> Instances;
		Instances.Reserve(WorldTransforms.Num());
		for (const FTransform& Xform : WorldTransforms)
		{
			FFoliageInstance Instance;
			Instance.Location = Xform.GetLocation();
			Instance.Rotation = Xform.Rotator();
			Instance.DrawScale3D = Xform.GetScale3D();
			Instances.Add(Instance);
		}
		Slot.Info->AddInstances(Slot.Type, Instances);
#else
		for (const FTransform& Xform : WorldTransforms)
		{
			FFoliageInstance Instance;
			Instance.Location = Xform.GetLocation();
			Instance.Rotation = Xform.Rotator();
			Instance.DrawScale3D = Xform.GetScale3D();
			// Rebuild once after all tiles via Refresh (per-instance rebuild is too slow).
			Slot.Info->AddInstance(&IFA, Slot.Type, Instance, /*InRebuildFoliageTree*/ false);
		}
#endif
		return true;
	}

	void RefreshFoliageSlots(TArray<FTreeFoliageSlot>& Slots, int32 TreeCullCm)
	{
		for (FTreeFoliageSlot& Slot : Slots)
		{
			if (!Slot.Info)
			{
				continue;
			}
			Slot.Info->Refresh(/*Async*/ false, /*Force*/ true);
			ApplyFoliageCullDistance(Slot.Type, Slot.Info, TreeCullCm);
		}
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
	float TreeCullDistanceMeters)
{
	FTreePlaceResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString AltitudeField = AltitudeFieldName.IsEmpty() ? TEXT("altitude") : AltitudeFieldName;
	const FString CleanInputPath = SanitizeFilePath(ShapefilePath);

	UE_LOG(LogTreePlacer, Display, TEXT("========== Tree Place START =========="));
	UE_LOG(
		LogTreePlacer,
		Display,
		TEXT("shp='%s' meshes='%s' altitudeField='%s' targetTiles=%d tileFilter='%s' seed=%d treeCullM=%.1f"),
		*CleanInputPath,
		*TreeMeshFolder,
		*AltitudeField,
		TargetTileCount,
		*TileIndices,
		RandomSeed,
		TreeCullDistanceMeters);

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

	AInstancedFoliageActor* IFA = AInstancedFoliageActor::GetInstancedFoliageActorForCurrentLevel(World, /*bCreateIfNone*/ true);
	if (!IFA)
	{
		Result.Message = TEXT("Could not get or create the level InstancedFoliageActor.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	IFA->Modify();
	if (!FolderPath.IsEmpty())
	{
		IFA->SetFolderPath(FName(*FolderPath));
	}

	const int32 TreeCullCm = MetersToCullDistanceCm(TreeCullDistanceMeters);

	TArray<FTreeFoliageSlot> FoliageSlots;
	FoliageSlots.SetNum(TreeMeshes.Num());
	for (int32 MeshIndex = 0; MeshIndex < TreeMeshes.Num(); ++MeshIndex)
	{
		FString SlotError;
		if (!GetOrCreateFoliageSlot(*IFA, TreeMeshes[MeshIndex], TreeCullCm, FoliageSlots[MeshIndex], SlotError))
		{
			Result.Message = SlotError;
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
			return Result;
		}
	}

	FScopedSlowTask SlowTask(
		static_cast<float>(NonEmptyTiles),
		NSLOCTEXT("TreePlacer", "PlaceProgress", "Placing trees on InstancedFoliageActor..."));
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
				RefreshFoliageSlots(FoliageSlots, TreeCullCm);
				IFA->Modify();
				World->MarkPackageDirty();
				Result.bCancelled = true;
				Result.TreesPlaced = TreesPlaced;
				Result.TreesSkipped = TreesSkipped;
				Result.TilesSpawned = TilesSpawned;
				Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
				Result.Message = FString::Printf(
					TEXT("Cancelled. Tiles=%d trees=%d on InstancedFoliageActor. Elapsed: %.2fs."),
					TilesSpawned,
					TreesPlaced,
					Result.ElapsedSeconds);
				UE_LOG(LogTreePlacer, Warning, TEXT("%s"), *Result.Message);
				return Result;
			}

			TArray<TArray<FTransform>> TransformsPerMesh;
			TransformsPerMesh.SetNum(TreeMeshes.Num());

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
				TransformsPerMesh[MeshIndex].Add(FTransform(
					FRotator(0.0, YawDeg, 0.0),
					WorldPos,
					FVector::OneVector));
			}

			bool bTileAdded = false;
			for (int32 MeshIndex = 0; MeshIndex < TreeMeshes.Num(); ++MeshIndex)
			{
				const TArray<FTransform>& Transforms = TransformsPerMesh[MeshIndex];
				if (Transforms.Num() == 0)
				{
					continue;
				}
				if (!AddFoliageInstances(*IFA, FoliageSlots[MeshIndex], Transforms))
				{
					UE_LOG(LogTreePlacer, Error, TEXT("Tile %s failed to add instances for mesh '%s'."),
						*TileLabel,
						*TreeMeshes[MeshIndex]->GetName());
					TreesSkipped += Transforms.Num();
					continue;
				}
				TreesPlaced += Transforms.Num();
				bTileAdded = true;
			}

			if (bTileAdded)
			{
				++TilesSpawned;
			}
		}
	}

	RefreshFoliageSlots(FoliageSlots, TreeCullCm);
	IFA->Modify();
	World->MarkPackageDirty();

	if (TilesSpawned <= 0)
	{
		Result.TreesPlaced = 0;
		Result.TreesSkipped = TreesSkipped;
		Result.TilesSpawned = 0;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(
			TEXT("No trees added to InstancedFoliageActor (%d skipped). Elapsed: %.2fs."),
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
		TEXT("Added %d trees (%d skipped) from %d tiles onto InstancedFoliageActor using %d mesh types from '%s'. Elapsed: %.2fs."),
		TreesPlaced,
		TreesSkipped,
		TilesSpawned,
		TreeMeshes.Num(),
		*TreeMeshFolder,
		Result.ElapsedSeconds);

	UE_LOG(LogTreePlacer, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogTreePlacer, Display, TEXT("========== Tree Place END =========="));
	return Result;
}
