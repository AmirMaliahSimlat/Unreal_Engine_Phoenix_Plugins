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
	int32 ClipPolygonsSpawned = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 WaterBodiesSpawned = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 TilesetsClipped = 0;

	/** Inner rings (islands in a lake) skipped in v1. */
	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 HoleRingsIgnored = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	int32 PolygonsSkipped = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	double ElapsedSeconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	bool bCancelled = false;

	UPROPERTY(BlueprintReadOnly, Category = "Water Placer")
	FString Message;
};

/**
 * Blueprint API for the Water Placer editor plugin.
 * Requires an ACesiumGeoreference and at least one ACesium3DTileset in the open editor map.
 * Enable the Water plugin. Recreate this Blueprint node after updating.
 */
UCLASS()
class WATERPLACER_API UWaterPlacerBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Reads EPSG:4326 water polygons and clips Cesium tilesets to those shapes.
	 * Invert Selection off (default): hide tileset INSIDE each polygon (lakes) and spawn
	 * Unreal Water Body Lakes in the holes.
	 * Invert Selection on: hide tileset OUTSIDE the polygons (keep islands). Water bodies
	 * are not spawned on the land.
	 *
	 * @param ShapefilePath Path to polygon .shp (.dbf required if AltitudeFieldName is set).
	 * @param AltitudeFieldName DBF column for water-surface altitude in meters. Empty = 0 (ellipsoid).
	 * @param bInvertSelection If false, clip inside polygons (lakes). If true, clip outside (islands).
	 * @param bPlaceWaterBodies If true and Invert Selection is off, spawn AWaterBodyLake per polygon.
	 * @param ActorLabelPrefix Prefix for spawned actor labels.
	 * @param EditorFolderPath World Outliner folder.
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "Water Placer",
		meta = (
			WorldContext = "WorldContextObject",
			CPP_Default_AltitudeFieldName = "altitude",
			CPP_Default_bInvertSelection = "false",
			CPP_Default_bPlaceWaterBodies = "true",
			CPP_Default_ActorLabelPrefix = "Water",
			CPP_Default_EditorFolderPath = "PlacedWater"))
	static FWaterPlaceResult PlaceWaterFromShapefile(
		UObject* WorldContextObject,
		const FString& ShapefilePath,
		const FString& AltitudeFieldName,
		bool bInvertSelection,
		bool bPlaceWaterBodies,
		const FString& ActorLabelPrefix,
		const FString& EditorFolderPath);
};
