#pragma once

#include "CoreMinimal.h"

struct FTreeShapefilePoint
{
	/** EPSG:4326 longitude degrees. */
	double LonDeg = 0.0;

	/** EPSG:4326 latitude degrees. */
	double LatDeg = 0.0;

	/** Floor / ground altitude in meters (AltitudeFieldName from DBF). */
	double AltitudeM = 0.0;

	int32 RecordIndex = 0;
};

namespace TreeShapefileReader
{
	/**
	 * Reads point shapefile (.shp + .dbf). Path may be with or without .shp extension.
	 * Coordinates are EPSG:4326 lon/lat. AltitudeFieldName is required in the DBF.
	 * Supports Point (1), PointZ (11), PointM (21). PointZ Z is ignored; altitude comes from DBF.
	 */
	bool ReadPoints(
		const FString& ShapefilePath,
		const FString& AltitudeFieldName,
		TArray<FTreeShapefilePoint>& OutPoints,
		FString& OutError);
}
