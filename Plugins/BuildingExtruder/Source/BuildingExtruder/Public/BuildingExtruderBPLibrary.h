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

	/** Non-empty tile StaticMeshActors spawned. */
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
 * Cesium DTM tileset is required only when not using shapefile altitudes.
 */
UCLASS()
class BUILDINGEXTRUDER_API UBuildingExtruderBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Reads EPSG:4326 building footprints from a shapefile (.shp + .dbf),
	 * places floors from either Cesium DTM sampling or a shapefile altitude field,
	 * extrudes tiled StaticMeshActors, and writes a combined FBX.
	 *
	 * @param ShapefilePath Path to .shp (with or without extension; .dbf required beside it).
	 * @param FbxOutputPath Required output path for the combined FBX.
	 * @param HeightFieldName DBF column for building extrusion in meters (default RELATIVE_F).
	 * @param ActorLabelPrefix Prefix for tile actor labels (e.g. BldgTile).
	 * @param EditorFolderPath World Outliner folder.
	 * @param TargetTileCount Exact tile slot count; XxY chosen from factor pairs for square cells.
	 * @param TileIndices Optional comma-separated linear tile indices (Y*TilesX+X), e.g. "0,6,12".
	 * @param bUseShapefileAltitude If true, use AltitudeFieldName from the DBF as floor Z and
	 *        skip Cesium DTM sampling. If false, sample DTM as before.
	 * @param AltitudeFieldName DBF column for floor altitude when bUseShapefileAltitude is true
	 *        (default "altitude"). Ignored when bUseShapefileAltitude is false.
	 * @param bEnableDtmLoadTimeout DTM refine timeout enable (ignored if shapefile altitude).
	 * @param DtmTimeoutSeconds DTM refine wait cap seconds; <=0 = default ~8-11s (ignored if shapefile altitude).
	 * @param DtmDoneProgressPercent Cesium load %% done threshold (ignored if shapefile altitude).
	 * @param bUsePerTileStableTimeout Per-tile stable DTM mode (ignored if shapefile altitude).
	 * @param bDiagnoseDtmLoadConsistency OLD vs STABLE dual layer (ignored if shapefile altitude).
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "Building Extruder",
		meta = (
			WorldContext = "WorldContextObject",
			CPP_Default_HeightFieldName = "RELATIVE_F",
			CPP_Default_ActorLabelPrefix = "BldgTile",
			CPP_Default_EditorFolderPath = "ExtrudedBuildings",
			CPP_Default_TargetTileCount = "64",
			CPP_Default_TileIndices = "",
			CPP_Default_bUseShapefileAltitude = "false",
			CPP_Default_AltitudeFieldName = "altitude",
			CPP_Default_bEnableDtmLoadTimeout = "true",
			CPP_Default_DtmTimeoutSeconds = "0.0",
			CPP_Default_DtmDoneProgressPercent = "95.0",
			CPP_Default_bUsePerTileStableTimeout = "false",
			CPP_Default_bDiagnoseDtmLoadConsistency = "false"))
	static FBuildingExtrudeResult ImportAndExtrudeBuildingsFromShapefile(
		UObject* WorldContextObject,
		const FString& ShapefilePath,
		const FString& FbxOutputPath,
		const FString& HeightFieldName,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath,
		int32 TargetTileCount,
		const FString& TileIndices,
		bool bUseShapefileAltitude,
		const FString& AltitudeFieldName,
		bool bEnableDtmLoadTimeout,
		float DtmTimeoutSeconds,
		float DtmDoneProgressPercent,
		bool bUsePerTileStableTimeout,
		bool bDiagnoseDtmLoadConsistency);
};
