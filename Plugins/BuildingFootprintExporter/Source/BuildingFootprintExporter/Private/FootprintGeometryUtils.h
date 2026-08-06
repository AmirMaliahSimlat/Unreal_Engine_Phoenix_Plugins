#pragma once

#include "CoreMinimal.h"
#include "FootprintSilhouette.h"

class AActor;
class UBuildingFootprintExportSettings;
class UBuildingFootprintFilterSettings;
class UStaticMeshComponent;
class UWorld;
struct FFootprintGeoTransform;

struct FBuildingFootprintPolygon
{
	/** Outer ring in lon/lat degrees. */
	TArray<FVector2D> LonLatRing;

	/** Optional hole rings in lon/lat (courtyards). */
	TArray<TArray<FVector2D>> HoleLonLatRings;

	FString SourceActorLabel;
	double AreaM2 = 0.0;
	/** Building height in meters (Zmax - Zmin). */
	double HeightM = 0.0;
};

struct FFootprintExtractionResult
{
	TArray<FBuildingFootprintPolygon> Footprints;
	int32 ActorsScanned = 0;
	int32 ComponentsAccepted = 0;
	int32 ComponentsRejected = 0;
	double ElapsedSeconds = 0.0;
	bool bCancelled = false;
	FString ErrorMessage;
};

namespace FootprintGeometryUtils
{
	bool ActorPassesFilter(const AActor& Actor, const UBuildingFootprintExportSettings& ExportSettings, const UBuildingFootprintFilterSettings* Filter);
	bool ComponentPassesFilter(const UStaticMeshComponent& Component, const UBuildingFootprintExportSettings& ExportSettings, const UBuildingFootprintFilterSettings* Filter);

	bool CollectComponentGroundGeometry(
		const UStaticMeshComponent& Component,
		TArray<FVector2D>& OutGroundPointsCm,
		TArray<FGroundTriangle2D>& OutTrianglesCm);

	FFootprintExtractionResult ExtractFootprints(
		UWorld* World,
		const UBuildingFootprintExportSettings& ExportSettings,
		const UBuildingFootprintFilterSettings* Filter,
		const FFootprintGeoTransform& Geo);
}
