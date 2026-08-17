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
	 * @param RedFieldName DBF 0–255 red column (used only when LeafTintMaterialPath is set).
	 * @param GreenFieldName DBF 0–255 green column.
	 * @param BlueFieldName DBF 0–255 blue column.
	 * @param LeafTintMaterialPath Content path of the leaf material to instance.
	 *        Empty = no coloring (current behavior). Recreate this Blueprint node after updating.
	 * @param LeafTintParameterName Vector parameter on that material set to the cluster color.
	 * @param LeafMaterialSlotIndex Mesh material slot to override (0 = first, often bark; 1 = leaves).
	 * @param ColorClusterCount K-means palette size K (not the mesh count). Clamped 1–64.
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
			CPP_Default_TreeCullDistanceMeters = "0.0",
			CPP_Default_RedFieldName = "R",
			CPP_Default_GreenFieldName = "G",
			CPP_Default_BlueFieldName = "B",
			CPP_Default_LeafTintMaterialPath = "",
			CPP_Default_LeafTintParameterName = "LeafTint",
			CPP_Default_LeafMaterialSlotIndex = "1",
			CPP_Default_ColorClusterCount = "8"))
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
		float TreeCullDistanceMeters,
		const FString& RedFieldName,
		const FString& GreenFieldName,
		const FString& BlueFieldName,
		const FString& LeafTintMaterialPath,
		const FString& LeafTintParameterName,
		int32 LeafMaterialSlotIndex,
		int32 ColorClusterCount);
};
