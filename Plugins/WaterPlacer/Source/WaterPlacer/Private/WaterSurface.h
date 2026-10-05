#pragma once

#include "CoreMinimal.h"
#include "WaterShapefileReader.h"
#include "Templates/Function.h"

struct FWaterSample
{
	double Lon = 0.0;
	double Lat = 0.0;
	double HeightM = 0.0;
};

struct FWaterTin
{
	TArray<FWaterSample> Vertices;
	TArray<int32> Triangles;
};

struct FWaterSurfaceResult
{
	FWaterTin Tin;
	TArray<FWaterSample> Outline;
	int32 ControlPoints = 0;
	int32 InteriorVertices = 0;
	double HeightMinM = 0.0;
	double HeightMaxM = 0.0;
	double ControlRmsM = 0.0;
};

namespace WaterSurface
{
	/**
	 * Builds a water TIN. Shore vertices use nearest PointZ height. Interior vertices
	 * use one thin-plate surface so height changes spread across the lake.
	 */
	bool BuildInterpolatedLake(
		const FWaterShapefilePolygon& Mask,
		const TArray<FWaterShapefilePoint>& ElevationPoints,
		int32 MaxOutlineVertices,
		double OutlineSmoothMeters,
		double InteriorSpacingMeters,
		double ControlSpacingMeters,
		FWaterSurfaceResult& OutSurface,
		FString& OutError,
		TFunction<bool(float, const TCHAR*)> Progress = nullptr);
}
