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
	double ElapsedSeconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	bool bCancelled = false;

	UPROPERTY(BlueprintReadOnly, Category = "Road Placer")
	FString Message;
};

/**
 * Reads a road-mask polygon and PointZ elevations, builds a draped TIN, and
 * places tiled engine StaticMeshActors. Recreate this node after updating.
 */
UCLASS()
class ROADPLACER_API URoadPlacerBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * @param MaskShapefilePath EPSG:4326 Polygon / PolygonZ road mask.
	 * @param ElevationPointsPath EPSG:4326 Point / PointZ samples along the mask outline
	 *        (not a filled interior grid). Z = ellipsoid meters. The TIN interpolates those
	 *        curb heights across the pavement so the slab tilts with the ground around the road.
	 * @param OptionalAltitudeFieldName DBF column that overrides geometry Z when set. Empty = use PointZ.
	 * @param RoadMaterialPath Optional Unreal material. Empty = engine default material.
	 * @param MeshContentFolder Content folder for saved road StaticMeshes.
	 * @param TargetTileCount Geographic tile slots (one StaticMeshActor per non-empty tile).
	 * @param MaxEdgeMeters Drop TIN triangles longer than this. Must be wider than the road
	 *        (outline-to-outline) so curb-to-curb triangles stay. Too small leaves only the curb.
	 * @param HeightOffsetMeters How far the road top sits above sampled Z (default 0.05 m).
	 * @param ThicknessMeters Slab thickness. Top = Z+offset, bottom = Z+offset-thickness
	 *        (default 0.10 m so the slab is 0.05 m above and 0.05 m into the DTM). 0 = thin surface.
	 * @param MetersPerUv Texture scale.
	 * @param bEnableCollision If true, road meshes have query+physics collision.
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
			CPP_Default_MaxEdgeMeters = "40.0",
			CPP_Default_HeightOffsetMeters = "0.05",
			CPP_Default_ThicknessMeters = "0.10",
			CPP_Default_MetersPerUv = "10.0",
			CPP_Default_bEnableCollision = "true",
			CPP_Default_ActorLabelPrefix = "Road",
			CPP_Default_EditorFolderPath = "PlacedRoads"))
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
		float MetersPerUv,
		bool bEnableCollision,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath);
};
