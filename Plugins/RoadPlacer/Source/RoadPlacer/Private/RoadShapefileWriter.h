#pragma once

#include "CoreMinimal.h"
#include "RoadShapefileReader.h"

struct FRoadShapefileVertex
{
	double Lon = 0.0;
	double Lat = 0.0;
	double HeightM = 0.0;
};

namespace RoadShapefileWriter
{
	/** Writes EPSG:4326 PolygonZ (.shp/.shx/.dbf/.prj/.cpg). Empty HeightM → Z = 0. */
	bool WritePolygonZRings(
		const FString& ShapefilePath,
		const TArray<FRoadShapefileRing>& Rings,
		FString& OutError,
		int32& OutFeatureCount);

	/** One PolygonZ feature per TIN triangle (closed 4-point ring). Road top only. */
	bool WritePolygonZTriangles(
		const FString& ShapefilePath,
		const TArray<FRoadShapefileVertex>& Vertices,
		const TArray<int32>& Triangles,
		FString& OutError,
		int32& OutFeatureCount);
}
