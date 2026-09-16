#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "RoadPlacerBPLibrary.generated.h"

USTRUCT(BlueprintType)
struct FRoadPlaceResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	bool bSuccess = false;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	int32 MaskPolygonsRead = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	int32 ElevationPointsRead = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	int32 TilesSpawned = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	int32 TrianglesBuilt = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	int32 TilesSkipped = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	int32 ClipPolygonsSpawned = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	int32 TilesetsClipped = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	int32 ExportFeaturesWritten = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	double ElapsedSeconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	bool bCancelled = false;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	FString Message;
};

/**
 * Reads a road-mask polygon and outline PointZ elevations, builds a draped TIN, and
 * places tiled engine StaticMeshActors. Recreate this node after updating.
 */
UCLASS()
class ROADPLACER_API URoadPlacerBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * @param MaskShapefilePath EPSG:4326 Polygon / PolygonZ road mask.
	 * @param ElevationPointsPath EPSG:4326 Point / PointZ curb/outline samples. Z = ellipsoid meters.
	 *        Interior/center points are not used; flatten the DTM under the mask in QGIS instead.
	 * @param OptionalAltitudeFieldName DBF column that overrides geometry Z when set. Empty = use PointZ.
	 * @param RoadMaterialPath Optional Unreal material. Empty = engine default material.
	 * @param MeshContentFolder Content folder for saved road StaticMeshes.
	 * @param TargetTileCount Geographic tile slots (one StaticMeshActor per non-empty tile).
	 *        Use 16–64 for a city. 1 tile triangulates the whole AOI at once and can take hours.
	 * @param MaxEdgeMeters Optional longest-edge cap in meters. 0 = off (recommended).
	 *        Values below 100 are ignored so leftover 3.5 / 40 m pins do not shred the pavement.
	 * @param HeightOffsetMeters How far the road top sits above sampled Z (default 0.10 m).
	 * @param ThicknessMeters Wall depth below the top. Top = Z+offset, wall foot = Z+offset-thickness
	 *        (default 0.20 m). No underside cap. 0 = top surface only.
	 * @param AltitudeSampleMeters Along-curb spacing of Z control samples. 0 = use every PointZ
	 *        height (same drape as 1.8). E.g. 10 keeps XY at 0.3–3 m but takes altitude every
	 *        ~10 m along each shapefile curb walk (not the mask ring) and linearly fills the rest.
	 * @param bSoftenEdges If true, the pavement top uses averaged/smoothed normals (soft).
	 *        Walls stay sharp so curb shading is not pulled sideways.
	 * @param MetersPerUv Texture scale.
	 * @param bEnableCollision If true, road meshes have query+physics collision.
	 * @param bClipGroundUnderRoads If true, hide Cesium imagery and DTM under placed road meshes.
	 *        Only the spawned tile(s) are clipped — Only Tile Index does not overlay the whole map.
	 *        Touching road islands are merged into a few Cesium polygons so a city does not spawn
	 *        a thousand raster overlays. Courtyards enclosed by a loop of roads are left in place.
	 * @param OnlyTileIndex 1-based tile to generate (same numbering as the Output Log).
	 *        0 = all tiles. With Target Tile Count 64, use 1..64 for a single test tile.
	 * @param bSkipRoadMeshes Temporary test: triangulate without saving road StaticMeshes.
	 *        Use with Clip Ground (FPS) and/or Export Shapefile Path (QGIS).
	 * @param ExportShapefilePath Optional EPSG:4326 PolygonZ path (.shp). Empty = do not write.
	 *        Outlines match the Cesium clips from this run (before overlay merge). Z is TIN
	 *        ellipsoid height. Clip Ground can stay off. Recreate this node after adding the pin.
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "Road Placer",
		meta = (
			WorldContext = "WorldContextObject",
			CPP_Default_OptionalAltitudeFieldName = "",
			CPP_Default_RoadMaterialPath = "",
			CPP_Default_MeshContentFolder = "/Game/RoadPlacer/Meshes",
			CPP_Default_TargetTileCount = "64",
			CPP_Default_MaxEdgeMeters = "0.0",
			CPP_Default_HeightOffsetMeters = "0.10",
			CPP_Default_ThicknessMeters = "0.20",
			CPP_Default_AltitudeSampleMeters = "0.0",
			CPP_Default_bSoftenEdges = "true",
			CPP_Default_MetersPerUv = "10.0",
			CPP_Default_bEnableCollision = "true",
			CPP_Default_ActorLabelPrefix = "Road",
			CPP_Default_EditorFolderPath = "PlacedRoads",
			CPP_Default_bClipGroundUnderRoads = "false",
			CPP_Default_OnlyTileIndex = "0",
			CPP_Default_bSkipRoadMeshes = "false",
			CPP_Default_ExportShapefilePath = ""))
	static FRoadPlaceResult PlaceRoadsFromShapefiles(
		UObject* WorldContextObject,
		const FString& MaskShapefilePath,
		const FString& ElevationPointsPath,
		const FString& OptionalAltitudeFieldName,
		const FString& RoadMaterialPath,
		const FString& MeshContentFolder,
		int32 TargetTileCount,
		float MaxEdgeMeters,
		float HeightOffsetMeters,
		float ThicknessMeters,
		float AltitudeSampleMeters,
		bool bSoftenEdges,
		float MetersPerUv,
		bool bEnableCollision,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath,
		bool bClipGroundUnderRoads,
		int32 OnlyTileIndex,
		bool bSkipRoadMeshes,
		const FString& ExportShapefilePath);
};
