#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "TreePlacerBPLibrary.generated.h"

USTRUCT(BlueprintType)
struct FTreePlaceResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Tree Placer")
	bool bSuccess = false;

	UPROPERTY(BlueprintReadOnly, Category = "Tree Placer")
	int32 TreesPlaced = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Tree Placer")
	int32 TreesSkipped = 0;

	/** Non-empty tiles that contributed instances to the foliage actor. */
	UPROPERTY(BlueprintReadOnly, Category = "Tree Placer")
	int32 TilesSpawned = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Tree Placer")
	double ElapsedSeconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Tree Placer")
	bool bCancelled = false;

	UPROPERTY(BlueprintReadOnly, Category = "Tree Placer")
	FString Message;
};

/**
 * Blueprint API for the Tree Placer editor plugin.
 * Requires an ACesiumGeoreference in the open editor map.
 * Tree meshes must already exist as Content assets (StaticMesh or FoliageType).
 * Instances are added to the level's AInstancedFoliageActor (same actor Foliage Mode uses).
 * Tiling is only used to batch work and filter TileIndices.
 */
UCLASS()
class TREEPLACER_API UTreePlacerBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Reads EPSG:4326 tree points from a shapefile and places random tree meshes from
	 * TreeMeshFolder onto the level Instanced Foliage Actor.
	 *
	 * @param ShapefilePath Path to point .shp (.dbf required beside it).
	 * @param TreeMeshFolder Content folder with StaticMesh and/or FoliageType assets (e.g. /Game/Trees).
	 * @param AltitudeFieldName DBF column for ground altitude in meters (default altitude).
	 * @param ActorLabelPrefix Prefix used only in progress labels (e.g. TreeTile).
	 * @param EditorFolderPath World Outliner folder for the InstancedFoliageActor.
	 * @param TargetTileCount Exact tile slot count; XxY chosen for near-square cells.
	 * @param TileIndices Optional comma-separated linear tile indices (Y*TilesX+X).
	 * @param RandomSeed RNG seed for mesh + yaw; 0 uses a non-deterministic seed.
	 * @param TreeCullDistanceMeters Camera distance in meters at which trees and their
	 *        shadows disappear together. <= 0 disables distance culling.
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "Tree Placer",
		meta = (
			WorldContext = "WorldContextObject",
			CPP_Default_AltitudeFieldName = "altitude",
			CPP_Default_ActorLabelPrefix = "TreeTile",
			CPP_Default_EditorFolderPath = "PlacedTrees",
			CPP_Default_TargetTileCount = "64",
			CPP_Default_TileIndices = "",
			CPP_Default_RandomSeed = "0",
			CPP_Default_TreeCullDistanceMeters = "0.0"))
	static FTreePlaceResult PlaceTreesFromShapefile(
		UObject* WorldContextObject,
		const FString& ShapefilePath,
		const FString& TreeMeshFolder,
		const FString& AltitudeFieldName,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath,
		int32 TargetTileCount,
		const FString& TileIndices,
		int32 RandomSeed,
		float TreeCullDistanceMeters);
};
