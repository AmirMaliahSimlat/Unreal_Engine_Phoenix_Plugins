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
	int32 MaskPolygonsRead = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 ElevationPointsRead = 0;

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
	int32 SurfaceVertices = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 SurfaceTriangles = 0;

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
 * Reads a 2D water mask plus PointZ outline heights, fits one smooth surface
 * across each lake, and spawns engine AStaticMeshActor meshes. Optional Cesium
 * clip hides imagery and DTM inside those polygons.
 * Recreate this Blueprint node after updating.
 */
UCLASS()
class WATERPLACER_API UWaterPlacerBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Reads an EPSG:4326 water mask and outline PointZ heights, then spawns a
	 * smooth water StaticMeshActor per lake. Shoreline Z is the nearest PointZ sample.
	 * Interior height is one thin-plate surface over the polygon (not a flat deck with a shore ramp).
	 *
	 * @param MaskShapefilePath EPSG:4326 Polygon / PolygonZ water mask (2D fill).
	 * @param ElevationPointsPath EPSG:4326 Point / PointZ shoreline samples. Z = ellipsoid meters.
	 * @param OptionalAltitudeFieldName DBF column that overrides geometry Z when set. Empty = use PointZ.
	 * @param WaterMaterialPath Optional Unreal asset path. Used only when Apply Water Material is on.
	 *        Empty with the toggle on = built-in wavy translucent water.
	 * @param bApplyWaterMaterial If false, spawn the mesh with an empty material slot (no default
	 *        water material is created or assigned).
	 * @param MeshContentFolder Content folder for saved water static meshes.
	 * @param bClipGroundUnderWater If true, hide Cesium imagery and DTM inside each water polygon.
	 * @param MaxOutlineVertices Optional cap on mask-outline vertices (mesh + clip). 0 = keep all.
	 * @param OutlineSmoothMeters If > 0, simplify stair-stepped raster outlines then round corners.
	 *        Units are meters. 0 = keep the shapefile vertices.
	 * @param HeightOffsetMeters Extra height on the fitted surface (default 0.15 m).
	 * @param InteriorSpacingMeters Distance in meters between extra mesh vertices inside the lake.
	 *        These samples carry the fitted height across the water. Smaller = denser mesh.
	 *        Minimum 1 m. There is no upper vertex cap.
	 * @param ControlSpacingMeters Distance in meters along the shore between PointZ height samples
	 *        that define the surface. 0 = use every nearby PointZ. Smaller = closer to raw outlines.
	 * @param SmoothShadingPasses 0 = faceted (hard edges). 1 = standard smooth shading.
	 *        2+ = extra neighbor-normal blur (lighting only). Max 8.
	 * @param ActorLabelPrefix Prefix for spawned actor labels.
	 * @param EditorFolderPath World Outliner folder.
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "Water Placer",
		meta = (
			WorldContext = "WorldContextObject",
			CPP_Default_OptionalAltitudeFieldName = "",
			CPP_Default_WaterMaterialPath = "",
			CPP_Default_bApplyWaterMaterial = "true",
			CPP_Default_MeshContentFolder = "/Game/WaterPlacer/Meshes",
			CPP_Default_bClipGroundUnderWater = "false",
			CPP_Default_MaxOutlineVertices = "0",
			CPP_Default_OutlineSmoothMeters = "15.0",
			CPP_Default_HeightOffsetMeters = "0.15",
			CPP_Default_InteriorSpacingMeters = "40.0",
			CPP_Default_ControlSpacingMeters = "80.0",
			CPP_Default_SmoothShadingPasses = "2",
			CPP_Default_ActorLabelPrefix = "Water",
			CPP_Default_EditorFolderPath = "PlacedWater"))
	static FWaterPlaceResult PlaceWaterFromShapefile(
		UObject* WorldContextObject,
		const FString& MaskShapefilePath,
		const FString& ElevationPointsPath,
		const FString& OptionalAltitudeFieldName,
		const FString& WaterMaterialPath,
		bool bApplyWaterMaterial,
		const FString& MeshContentFolder,
		bool bClipGroundUnderWater,
		int32 MaxOutlineVertices,
		float OutlineSmoothMeters,
		float HeightOffsetMeters,
		float InteriorSpacingMeters,
		float ControlSpacingMeters,
		int32 SmoothShadingPasses,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath);
};
