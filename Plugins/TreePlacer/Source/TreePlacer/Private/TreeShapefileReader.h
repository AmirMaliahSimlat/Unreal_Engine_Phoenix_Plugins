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

	/** True if R/G/B DBF fields parsed as 0–255. */
	bool bHasRgb = false;
	uint8 R = 0;
	uint8 G = 0;
	uint8 B = 0;

	int32 RecordIndex = 0;
};

namespace TreeShapefileReader
{
	/**
	 * Reads point shapefile (.shp + .dbf). Path may be with or without .shp extension.
	 * Coordinates are EPSG:4326 lon/lat. AltitudeFieldName is required in the DBF.
	 * Optional Red/Green/BlueFieldName (0–255). Empty names skip RGB.
	 * Supports Point (1), PointZ (11), PointM (21). PointZ Z is ignored; altitude comes from DBF.
	 */
	bool ReadPoints(
		const FString& ShapefilePath,
		const FString& AltitudeFieldName,
		TArray<FTreeShapefilePoint>& OutPoints,
		FString& OutError,
		const FString& RedFieldName = TEXT(""),
		const FString& GreenFieldName = TEXT(""),
		const FString& BlueFieldName = TEXT(""));
}
