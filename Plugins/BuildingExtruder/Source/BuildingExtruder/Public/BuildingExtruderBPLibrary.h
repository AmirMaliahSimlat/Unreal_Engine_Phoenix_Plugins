#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "BuildingExtruderBPLibrary.generated.h"

USTRUCT(BlueprintType)
struct FBuildingExtrudeResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	bool bSuccess = false;

	/** Buildings successfully meshed into tiles. */
	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	int32 BuildingsSpawned = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	int32 BuildingsSkipped = 0;

	/** Non-empty tile slots spawned (one actor per tile; walls/roofs split by material). */
	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	int32 TilesSpawned = 0;

	/** Roof props added to the level InstancedFoliageActor (0 if placement is off). */
	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	int32 RoofObjectsPlaced = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	double ElapsedSeconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	bool bCancelled = false;

	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	FString Message;
};

/**
 * Blueprint API for the Building Extruder editor plugin.
 * Requires an ACesiumGeoreference in the open editor map.
 * Floor Z comes from a shapefile altitude DBF field (no Cesium DTM sampling).
 */
UCLASS()
class BUILDINGEXTRUDER_API UBuildingExtruderBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Reads EPSG:4326 building footprints from a shapefile (.shp + .dbf),
	 * places floors from AltitudeFieldName, and saves tiled StaticMesh assets under
	 * /Game/BuildingExtruder/Meshes (one tile actor; one mesh per wall/roof material group).
	 * Recreate this Blueprint node after updating.
	 *
	 * Shapefile
	 * @param ShapefilePath Path to .shp (with or without extension; .dbf required beside it).
	 * @param AltitudeFieldName DBF column for floor altitude in meters (exact name, default altitude).
	 * @param HeightFieldName DBF column for wall height in meters (default height).
	 *
	 * Level / tiling
	 * @param ActorLabelPrefix Prefix for tile actor labels (e.g. BldgTile).
	 * @param EditorFolderPath World Outliner folder.
	 * @param TargetTileCount Tile slot count; XxY chosen for near-square cells. Entire map is processed.
	 *
	 * Materials (one component per group)
	 * @param WallMaterialSlotCount Wall/floor groups per tile. Buildings are shuffled then split evenly
	 *        into this many mesh assets (one material slot each). Leftover groups stay empty if a tile
	 *        has fewer buildings than the count.
	 * @param RoofMaterialSlotCount Roof groups per tile (same split, independent of walls).
	 * @param MetersPerUv Texture mapping scale in meters per UV unit (walls and roof/floor).
	 * @param MaterialRandomSeed RNG for material groups and roof-object poses; 0 = non-deterministic.
	 *
	 * Roof shapes (optional)
	 * @param bUseRoofTypes If false, every roof is flat (roof-type DBF column not required).
	 * @param RoofTypeFieldName DBF column with integer roof-type codes. Used only if bUseRoofTypes.
	 * @param FlatRoofIndex DBF value that means a flat roof.
	 * @param HippedRoofIndex DBF value that means a hipped / cross-hipped roof.
	 * @param ParapetRoofIndex DBF value that means a parapet roof.
	 * @param ParapetHeightMeters Height of the parapet ring in meters (included in wall height).
	 * @param ParapetWidthMeters Inward thickness of the parapet ring in meters.
	 * @param HippedHeightMeters Ridge height above the wall top in meters (not included in wall height).
	 *
	 * Roof objects
	 * @param bPlaceRoofObjects If true, place antenna/boiler/etc. meshes on roofs via InstancedFoliageActor.
	 * @param RoofObjectMeshFolder Content folder of roof prop StaticMeshes. Used only if bPlaceRoofObjects.
	 *        Only StaticMesh assets are used. Each mesh is independently rolled per roof (0 or 1).
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "Building Extruder",
		meta = (
			WorldContext = "WorldContextObject",
			CPP_Default_AltitudeFieldName = "altitude",
			CPP_Default_HeightFieldName = "height",
			CPP_Default_ActorLabelPrefix = "BldgTile",
			CPP_Default_EditorFolderPath = "ExtrudedBuildings",
			CPP_Default_TargetTileCount = "64",
			CPP_Default_WallMaterialSlotCount = "1",
			CPP_Default_RoofMaterialSlotCount = "1",
			CPP_Default_MetersPerUv = "3.0",
			CPP_Default_MaterialRandomSeed = "0",
			CPP_Default_bUseRoofTypes = "false",
			CPP_Default_RoofTypeFieldName = "roof_type",
			CPP_Default_FlatRoofIndex = "0",
			CPP_Default_HippedRoofIndex = "1",
			CPP_Default_ParapetRoofIndex = "2",
			CPP_Default_ParapetHeightMeters = "1.0",
			CPP_Default_ParapetWidthMeters = "0.3",
			CPP_Default_HippedHeightMeters = "2.0",
			CPP_Default_bPlaceRoofObjects = "false",
			CPP_Default_RoofObjectMeshFolder = ""))
	static FBuildingExtrudeResult ImportAndExtrudeBuildingsFromShapefile(
		UObject* WorldContextObject,
		const FString& ShapefilePath,
		const FString& AltitudeFieldName,
		const FString& HeightFieldName,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath,
		int32 TargetTileCount,
		int32 WallMaterialSlotCount,
		int32 RoofMaterialSlotCount,
		float MetersPerUv,
		int32 MaterialRandomSeed,
		bool bUseRoofTypes,
		const FString& RoofTypeFieldName,
		int32 FlatRoofIndex,
		int32 HippedRoofIndex,
		int32 ParapetRoofIndex,
		float ParapetHeightMeters,
		float ParapetWidthMeters,
		float HippedHeightMeters,
		bool bPlaceRoofObjects,
		const FString& RoofObjectMeshFolder);
};
