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

	/** Non-empty tile actors spawned (each tile holds HISM components). */
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
 * Large point counts are placed as Hierarchical Instanced Static Meshes per tile
 * (same tiling grid logic as Building Extruder) so level save / undo stay stable.
 */
UCLASS()
class TREEPLACER_API UTreePlacerBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Reads EPSG:4326 tree points from a shapefile and places random tree meshes from
	 * TreeMeshFolder into tiled HISM actors in the open level.
	 *
	 * @param ShapefilePath Path to point .shp (.dbf required beside it).
	 * @param TreeMeshFolder Content folder with StaticMesh and/or FoliageType assets (e.g. /Game/Trees).
	 * @param AltitudeFieldName DBF column for ground altitude in meters (default altitude).
	 * @param ActorLabelPrefix Prefix for tile actor labels (e.g. TreeTile).
	 * @param EditorFolderPath World Outliner folder.
	 * @param TargetTileCount Exact tile slot count; XxY chosen for near-square cells.
	 * @param TileIndices Optional comma-separated linear tile indices (Y*TilesX+X).
	 * @param RandomSeed RNG seed for mesh + yaw; 0 uses a non-deterministic seed.
	 * @param TreeDisappearLOD Mesh LOD index at which trees cull (and stop drawing).
	 *        Negative or above the mesh max LOD → use that mesh's maximum LOD.
	 * @param ShadowDisappearLOD Mesh LOD index at which tree shadows stop casting.
	 *        Negative or above the mesh max LOD → use that mesh's maximum LOD.
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
			CPP_Default_TreeDisappearLOD = "-1",
			CPP_Default_ShadowDisappearLOD = "-1"))
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
		int32 TreeDisappearLOD,
		int32 ShadowDisappearLOD);
};
