#pragma once

#include "CoreMinimal.h"

struct FBuildingShapefileFeature
{
	/** Outer ring in EPSG:4326 (WGS84): X = longitude degrees, Y = latitude degrees. Not necessarily closed. */
	TArray<FVector2D> OuterRingLonLat;

	/** Extrusion / wall height in meters (RELATIVE_F). */
	double HeightM = 0.0;

	/** Lowest-point base altitude in meters (ellipsoid). Filled from Cesium DTM sampling. */
	double ElevationM = 0.0;

	int32 RecordIndex = 0;
};

namespace BuildingShapefileReader
{
	/**
	 * Reads polygon shapefile (.shp + .dbf). Path may be with or without .shp extension.
	 * Coordinates are interpreted as EPSG:4326 lon/lat degrees (no reprojection).
	 * HeightField is required (extrusion). ElevationField is optional and unused for placement
	 * when empty (DTM sampling sets base altitude instead).
	 */
	bool ReadPolygonBuildings(
		const FString& ShapefilePath,
		const FString& HeightFieldName,
		const FString& ElevationFieldName,
		TArray<FBuildingShapefileFeature>& OutFeatures,
		FString& OutError);
}
