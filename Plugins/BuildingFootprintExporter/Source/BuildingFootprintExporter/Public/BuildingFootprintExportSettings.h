#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "BuildingFootprintExportSettings.generated.h"

/**
 * Georeferencing and export options for one city/map.
 * Create one data asset per map in your *game project* Content (not in this plugin),
 * or pass FBuildingFootprintMapInput at export time from Blueprints.
 *
 * Fixed conventions:
 * - Unreal +X = East, +Y = South
 * - 100 Unreal units = 1 meter
 * - Components are silhouetted in size-limited nearby groups (combined AABB must fit
 *   MaxGrid @ 1 cm) so large tiles do not force coarse cells / stair outlines
 */
UCLASS(BlueprintType)
class BUILDINGFOOTPRINTEXPORTER_API UBuildingFootprintExportSettings : public UDataAsset
{
	GENERATED_BODY()

public:
	/** Map / city label written into the shapefile attribute table. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Map")
	FString MapName = TEXT("UnnamedMap");

	/** Geographic origin latitude in decimal degrees (WGS84). Maps to Unreal (0,0,0). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Georeference")
	double OriginLatitude = 0.0;

	/** Geographic origin longitude in decimal degrees (WGS84). Maps to Unreal (0,0,0). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Georeference")
	double OriginLongitude = 0.0;

	/**
	 * Merge nearby mesh pieces into one building, but only while the combined 2D bounds
	 * still fit Max Grid Dimension at 1 cm/cell. Stops whole-tile mega-clusters that
	 * coarsen the grid and leave stair-step footprints.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Extraction", meta = (ClampMin = "0.0"))
	double ClusterMergeDistanceCm = 50.0;

	/**
	 * If non-empty, only components that use a material whose name contains this substring
	 * (case-insensitive) are exported. Example: "roof".
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Extraction")
	FString IncludeMaterialNameContains;

	/**
	 * After the outline is traced from the grid, remove vertices that bend the edge
	 * by less than this distance (cm). Think: "ignore wiggles smaller than X cm".
	 * 0 = keep every grid stair-step. Higher = smoother, fewer points, less fine detail.
	 * Applied after any cell-size coarsening. Kept as this world-space value unless the
	 * cell grows larger than X/2, in which case tolerance becomes 2*cell so fat grid
	 * stairs can still be cleared (never multiplies X just because cell doubled slightly).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Silhouette", meta = (ClampMin = "0.0"))
	double SilhouetteSimplifyToleranceCm = 30.0;

	/**
	 * Safety clamp on silhouette grid width/height. If the building bounds need more cells,
	 * cell size is increased automatically so the grid stays within this limit.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Silhouette", meta = (ClampMin = "64"))
	int32 SilhouetteMaxGridDimension = 8192;
};
