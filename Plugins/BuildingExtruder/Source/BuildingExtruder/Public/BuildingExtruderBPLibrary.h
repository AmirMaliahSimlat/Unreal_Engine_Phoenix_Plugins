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
	 * @param HeightFieldName DBF column for building extrusion height in meters (default RELATIVE_F).
	 * @param ActorLabelPrefix Prefix for tile actor labels (e.g. BldgTile).
	 * @param EditorFolderPath World Outliner folder.
	 * @param TargetTileCount Exact tile slot count; XxY chosen from factor pairs for square cells.
	 * @param TileIndices Optional comma-separated linear tile indices (Y*TilesX+X), e.g. "0,6,12".
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
			CPP_Default_TileIndices = ""))
	static FBuildingExtrudeResult ImportAndExtrudeBuildingsFromShapefile(
		UObject* WorldContextObject,
		const FString& ShapefilePath,
		const FString& FbxOutputPath,
		const FString& AltitudeFieldName,
		const FString& HeightFieldName,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath,
		int32 TargetTileCount,
		const FString& TileIndices);
};
