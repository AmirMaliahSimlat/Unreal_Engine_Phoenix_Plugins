#pragma once

#include "CoreMinimal.h"
#include "Math/Box2D.h"

struct FWaterShapefilePolygon
{
	/** Outer ring in EPSG:4326: X = longitude degrees, Y = latitude degrees. Not necessarily closed. */
	TArray<FVector2D> OuterRingLonLat;

	FBox2D Bounds = FBox2D(ForceInit);

	/** Optional DBF altitude (unused when PointZ outlines drive the surface). */
	double AltitudeM = 0.0;

	int32 RecordIndex = 0;
	int32 HoleRingCount = 0;
};

struct FWaterShapefilePoint
{
	double LonDeg = 0.0;
	double LatDeg = 0.0;
	double HeightM = 0.0;
	int32 RecordIndex = 0;
};

namespace WaterShapefileReader
{
	/**
	 * Reads polygon shapefile (.shp + .dbf). Path may be with or without .shp extension.
	 * Coordinates are interpreted as EPSG:4326 lon/lat degrees (no reprojection).
	 * AltitudeFieldName is optional (empty = AltitudeM stays 0).
	 * Inner rings (islands in a lake) are counted but not returned.
	 */
	bool ReadWaterPolygons(
		const FString& ShapefilePath,
		const FString& AltitudeFieldName,
		TArray<FWaterShapefilePolygon>& OutPolygons,
		FString& OutError);

	/**
	 * Reads Point / PointZ / PointM / MultiPoint.
	 * Geometry Z is used when present (PointZ). Optional DBF field overrides Z when set.
	 */
	bool ReadElevationPoints(
		const FString& ShapefilePath,
		const FString& OptionalAltitudeFieldName,
		TArray<FWaterShapefilePoint>& OutPoints,
		FString& OutError);
}
