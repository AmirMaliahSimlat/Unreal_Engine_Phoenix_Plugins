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
 * Requires an ACesiumGeoreference and a Cesium terrain/DTM tileset in the open editor map.
 */
UCLASS()
class BUILDINGEXTRUDER_API UBuildingExtruderBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Reads EPSG:4326 building footprints from a shapefile (.shp + .dbf),
	 * samples Cesium DTM height at each outer-ring vertex (DTM tileset only), places each
	 * flat floor at the minimum terrain height under that footprint, merges into tiles
	 * (one StaticMeshActor per tile), and writes a combined FBX.
	 *
	 * Tip: for fast-vs-slow A/B, run twice with different DtmTimeoutSeconds / DtmDoneProgressPercent
	 * and different ActorLabelPrefix or EditorFolderPath (separate outputs; no shared dual-pass).
	 * Restart editor or Refresh the DTM tileset between runs if you need a colder cache.
	 *
	 * @param ShapefilePath Path to .shp (with or without extension; .dbf required beside it).
	 * @param FbxOutputPath Required output path for the combined FBX.
	 * @param HeightFieldName DBF column for building extrusion in meters (default RELATIVE_F).
	 * @param ActorLabelPrefix Prefix for tile actor labels (e.g. BldgTile).
	 * @param EditorFolderPath World Outliner folder.
	 * @param TargetTileCount Exact tile slot count; XxY chosen from factor pairs for square cells.
	 * @param TileIndices Optional comma-separated linear tile indices (Y*TilesX+X), e.g. "0,6,12".
	 * @param bEnableDtmLoadTimeout If true, stop refine at DtmTimeoutSeconds (or default formula).
	 *        If false, wait until done % or stall (safety cap 600s).
	 * @param DtmTimeoutSeconds Refine wait cap in seconds when timeout is enabled.
	 *        <=0 means default 8 + 0.05*batchPointCount (~8-11s).
	 * @param DtmDoneProgressPercent Cesium GetLoadProgress() threshold to treat refine as done (1-99).
	 * @param bUsePerTileStableTimeout If true, per tile: hardest footprint -> time-to-stable (+ margin)
	 *        as that tile's timeout (overrides DtmTimeoutSeconds for that tile).
	 * @param bDiagnoseDtmLoadConsistency Optional dual-pass compare in one run (prefer separate runs).
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
		bool bEnableDtmLoadTimeout,
		float DtmTimeoutSeconds,
		float DtmDoneProgressPercent,
		bool bUsePerTileStableTimeout,
		bool bDiagnoseDtmLoadConsistency);
};
