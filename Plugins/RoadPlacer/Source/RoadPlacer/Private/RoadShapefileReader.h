#pragma once

#include "CoreMinimal.h"

struct FRoadShapefileRing
{
	TArray<FVector2D> LonLat;
	/** Per-vertex ellipsoid height in meters. Empty or shorter than LonLat = no Z on this ring. */
	TArray<double> HeightM;
};

struct FRoadShapefileMask
{
	FRoadShapefileRing Outer;
	TArray<FRoadShapefileRing> Holes;
	int32 RecordIndex = 0;
};

struct FRoadShapefilePoint
{
	double LonDeg = 0.0;
	double LatDeg = 0.0;
	double HeightM = 0.0;
	int32 RecordIndex = 0;
};

namespace RoadShapefileReader
{
	bool ReadMaskPolygons(
		const FString& ShapefilePath,
		TArray<FRoadShapefileMask>& OutMasks,
		FString& OutError);

	/**
	 * Reads Point / PointZ / PointM.
	 * Geometry Z is used when present (PointZ). Optional DBF field overrides a missing/zero Z.
	 */
	bool ReadElevationPoints(
		const FString& ShapefilePath,
		const FString& OptionalAltitudeFieldName,
		TArray<FRoadShapefilePoint>& OutPoints,
		FString& OutError);
}
