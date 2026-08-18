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

	/** Non-empty tile slots spawned (each tile = walls actor + roof actor). */
	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	int32 TilesSpawned = 0;

	/** Roof props added to the level InstancedFoliageActor (0 if placement is off). */
	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	int32 RoofObjectsPlaced = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	double ElapsedSeconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	bool bCancelled = false;

	/** Absolute path of the written FBX (empty if not written). */
	UPROPERTY(BlueprintReadOnly, Category = "Building Extruder")
	FString FbxOutputPath;

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
	 * places floors from AltitudeFieldName, saves tiled StaticMesh assets under
	 * /Game/BuildingExtruder/Meshes (walls+floor and roof as separate actors per tile),
	 * and writes a combined FBX.
	 *
	 * @param ShapefilePath Path to .shp (with or without extension; .dbf required beside it).
	 * @param FbxOutputPath Required output path for the combined FBX.
	 * @param AltitudeFieldName DBF column for floor altitude in meters (default altitude).
	 * @param HeightFieldName DBF column for wall height in meters (default RELATIVE_F).
	 *        Hipped ridge is extra (HippedHeightMeters). Parapet ring is part of this wall height.
	 * @param ActorLabelPrefix Prefix for tile actor labels (e.g. BldgTile).
	 * @param EditorFolderPath World Outliner folder.
	 * @param TargetTileCount Exact tile slot count; XxY chosen from factor pairs for square cells.
	 * @param TileIndices Optional comma-separated linear tile indices (Y*TilesX+X), e.g. "0,6,12".
	 * @param MetersPerUv Texture mapping scale in meters per UV unit (walls and roof/floor).
	 * @param WallMaterialSlotCount Number of material slots on wall/floor meshes (random per building).
	 * @param RoofMaterialSlotCount Number of material slots on roof meshes (random per building).
	 * @param MaterialRandomSeed RNG for material slots and roof-object poses; 0 = non-deterministic each run.
	 * @param RoofTypeFieldName DBF column with integer roof-type codes.
	 * @param FlatRoofIndex DBF value that means a flat roof.
	 * @param HippedRoofIndex DBF value that means a hipped / cross-hipped roof.
	 * @param ParapetRoofIndex DBF value that means a parapet roof.
	 * @param ParapetHeightMeters Height of the parapet ring in meters (included in wall height).
	 * @param ParapetWidthMeters Inward thickness of the parapet ring in meters.
	 * @param HippedHeightMeters Ridge height above the wall top in meters (not included in wall height).
	 * @param bPlaceRoofObjects If true, place antenna/boiler/etc. meshes on roofs via InstancedFoliageActor.
	 * @param RoofObjectMeshFolder Content folder of roof prop StaticMeshes. Used only if bPlaceRoofObjects.
	 *        Only StaticMesh assets are used (materials and other files in the folder are ignored).
	 *        Each mesh is independently rolled per roof (0 or 1); skipped if it cannot fit.
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "Building Extruder",
		meta = (
			WorldContext = "WorldContextObject",
			CPP_Default_AltitudeFieldName = "altitude",
			CPP_Default_HeightFieldName = "RELATIVE_F",
			CPP_Default_ActorLabelPrefix = "BldgTile",
			CPP_Default_EditorFolderPath = "ExtrudedBuildings",
			CPP_Default_TargetTileCount = "64",
			CPP_Default_TileIndices = "",
			CPP_Default_MetersPerUv = "3.0",
			CPP_Default_WallMaterialSlotCount = "1",
			CPP_Default_RoofMaterialSlotCount = "1",
			CPP_Default_MaterialRandomSeed = "0",
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
		const FString& FbxOutputPath,
		const FString& AltitudeFieldName,
		const FString& HeightFieldName,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath,
		int32 TargetTileCount,
		const FString& TileIndices,
		float MetersPerUv,
		int32 WallMaterialSlotCount,
		int32 RoofMaterialSlotCount,
		int32 MaterialRandomSeed,
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
