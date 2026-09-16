#pragma once

#include "CoreMinimal.h"
#include "RoadShapefileReader.h"

namespace RoadShapefileWriter
{
	/** Writes EPSG:4326 PolygonZ (.shp/.shx/.dbf/.prj/.cpg). Empty HeightM → Z = 0. */
	bool WritePolygonZRings(
		const FString& ShapefilePath,
		const TArray<FRoadShapefileRing>& Rings,
		FString& OutError,
		int32& OutFeatureCount);
}
