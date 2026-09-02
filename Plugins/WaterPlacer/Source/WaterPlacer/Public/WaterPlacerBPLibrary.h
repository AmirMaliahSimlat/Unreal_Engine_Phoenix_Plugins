#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "WaterPlacerBPLibrary.generated.h"

USTRUCT(BlueprintType)
struct FWaterPlaceResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	bool bSuccess = false;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 PolygonsRead = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 WaterMeshesSpawned = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 ClipPolygonsSpawned = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 TilesetsClipped = 0;

	/** Inner rings (islands in a lake) skipped in v1. */
	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 HoleRingsIgnored = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 PolygonsSkipped = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 TerrainSamplesHit = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 TerrainSamplesMissed = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	double ElapsedSeconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	bool bCancelled = false;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	FString Message;
};

/**
 * Blueprint API for the Water Placer editor plugin.
 * Requires an ACesiumGeoreference in the open editor map.
 * Spawns engine AStaticMeshActor water surfaces from shapefile polygons,
 * optionally draped onto Cesium World Terrain so shores match DTM elevation.
 * Optional Cesium clip hides imagery and DTM inside those polygons.
 * Recreate this Blueprint node after updating.
 */
UCLASS()
class WATERPLACER_API UWaterPlacerBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Reads EPSG:4326 water polygons and spawns AStaticMeshActor meshes shaped to each polygon.
	 *
	 * @param ShapefilePath Path to polygon .shp (.dbf required if AltitudeFieldName is set).
	 * @param AltitudeFieldName DBF column for water-surface altitude in meters. Empty = 0 (ellipsoid).
	 * @param WaterMaterialPath Optional Unreal asset path. Empty = built-in wavy translucent water
	 *        (Single Layer Water / Water_Material_Ocean is not visible on StaticMeshActors).
	 * @param MeshContentFolder Content folder for saved water static meshes.
	 * @param bClipGroundUnderWater If true, hide Cesium imagery and DTM inside each water polygon.
	 * @param MaxOutlineVertices Max vertices kept per polygon outline (meshes and clip polygons).
	 * @param OutlineSmoothMeters If > 0, simplify stair-stepped raster outlines then round corners.
	 *        Units are meters. 0 = keep the shapefile vertices (then cap with MaxOutlineVertices).
	 * @param bDrapeOnCesiumTerrain If true, sample Cesium World Terrain at each shoreline vertex so
	 *        the water mesh follows shore elevation (shapefile Z is not used / not required).
	 * @param DrapeHeightOffsetMeters Extra height above the sampled terrain to reduce z-fighting.
	 * @param SmoothShadingPasses 0 = faceted (hard edges). 1 = standard smooth shading.
	 *        2+ = extra neighbor-normal blur (lighting only; shore positions stay draped). Max 8.
	 * @param ActorLabelPrefix Prefix for spawned actor labels.
	 * @param EditorFolderPath World Outliner folder.
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "Water Placer",
		meta = (
			WorldContext = "WorldContextObject",
			CPP_Default_AltitudeFieldName = "altitude",
			CPP_Default_WaterMaterialPath = "",
			CPP_Default_MeshContentFolder = "/Game/WaterPlacer/Meshes",
			CPP_Default_bClipGroundUnderWater = "false",
			CPP_Default_MaxOutlineVertices = "8192",
			CPP_Default_OutlineSmoothMeters = "15.0",
			CPP_Default_bDrapeOnCesiumTerrain = "true",
			CPP_Default_DrapeHeightOffsetMeters = "0.3",
			CPP_Default_SmoothShadingPasses = "2",
			CPP_Default_ActorLabelPrefix = "Water",
			CPP_Default_EditorFolderPath = "PlacedWater"))
	static FWaterPlaceResult PlaceWaterFromShapefile(
		UObject* WorldContextObject,
		const FString& ShapefilePath,
		const FString& AltitudeFieldName,
		const FString& WaterMaterialPath,
		const FString& MeshContentFolder,
		bool bClipGroundUnderWater,
		int32 MaxOutlineVertices,
		float OutlineSmoothMeters,
		bool bDrapeOnCesiumTerrain,
		float DrapeHeightOffsetMeters,
		int32 SmoothShadingPasses,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath);
};
