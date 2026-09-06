#pragma once

#include "CoreMinimal.h"
#include "RoadShapefileReader.h"
#include "Templates/Function.h"

struct FRoadSample
{
	double Lon = 0.0;
	double Lat = 0.0;
	double HeightM = 0.0;
};

struct FRoadTin
{
	TArray<FRoadSample> Vertices;
	TArray<int32> Triangles;
};

namespace RoadTriangulate
{
	bool PointInMask(const FVector2D& LonLat, const TArray<FRoadShapefileMask>& Masks);

	double EdgeMeters(const FRoadSample& A, const FRoadSample& B);

	/**
	 * Delaunay TIN of Samples. Keeps triangles whose centroid is inside Masks
	 * and whose longest edge is <= MaxEdgeMeters.
	 * Progress(Fraction01, Stage) may be called during insert/clip; return false to cancel.
	 */
	bool BuildTin(
		const TArray<FRoadSample>& Samples,
		const TArray<FRoadShapefileMask>& Masks,
		double MaxEdgeMeters,
		FRoadTin& OutTin,
		FString& OutError,
		TFunction<bool(float, const TCHAR*)> Progress = nullptr);
}
