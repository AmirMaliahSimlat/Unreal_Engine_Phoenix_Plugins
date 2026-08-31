#pragma once

#include "CoreMinimal.h"

struct FWaterShapefilePolygon
{
	/** Outer ring in EPSG:4326: X = longitude degrees, Y = latitude degrees. Not necessarily closed. */
	TArray<FVector2D> OuterRingLonLat;

	/** Water surface altitude in meters (AltitudeFieldName from DBF). 0 if the field is unused. */
	double AltitudeM = 0.0;

	int32 RecordIndex = 0;
	int32 HoleRingCount = 0;
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
}
