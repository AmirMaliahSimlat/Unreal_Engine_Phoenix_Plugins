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
	 * @param ShapefilePath Path to .shp (with or without extension; .dbf required beside it).
	 * @param FbxOutputPath Required output path for the combined FBX.
	 * @param HeightFieldName DBF column for building extrusion in meters (default RELATIVE_F).
	 * @param ActorLabelPrefix Prefix for tile actor labels (e.g. BldgTile).
	 * @param EditorFolderPath World Outliner folder.
	 * @param TargetTileCount Exact tile slot count; X×Y chosen from factor pairs for square cells.
	 * @param TileIndices Optional comma-separated linear tile indices (Y*TilesX+X), e.g. "0,6,12".
	 *        Empty = all tiles. Same TargetTileCount + full dataset → same indices as a full run.
	 * @param bEnableDtmLoadTimeout If true, DTM refine waits with a short timeout (faster).
	 *        If false, waits until load progress ~95% or progress stalls (better for bad tiles).
	 *
	 * Temporary diagnose (no BP pin — keeps node signature stable):
	 * Console: BuildingExtruder.DiagnoseDtmLoadConsistency 1
	 * Cold-reloads DTM before each pass, samples normal (95%/~8-11s) then deep (98%/30s),
	 * spawns BOTH layers in the level (FBX = normal layer only), and logs floor-min deltas.
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
			CPP_Default_bEnableDtmLoadTimeout = "true"))
	static FBuildingExtrudeResult ImportAndExtrudeBuildingsFromShapefile(
		UObject* WorldContextObject,
		const FString& ShapefilePath,
		const FString& FbxOutputPath,
		const FString& HeightFieldName,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath,
		int32 TargetTileCount,
		const FString& TileIndices,
		bool bEnableDtmLoadTimeout);
};
