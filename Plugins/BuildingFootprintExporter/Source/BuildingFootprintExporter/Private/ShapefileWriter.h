#pragma once

#include "CoreMinimal.h"

struct FBuildingFootprintPolygon;

namespace ShapefileWriter
{
	/**
	 * Writes ESRI Shapefile polygon set (.shp/.shx/.dbf/.prj) in WGS84.
	 * OutputPathWithoutExtension example: "D:/Exports/Nablus_Buildings"
	 */
	bool WritePolygons(
		const FString& OutputPathWithoutExtension,
		const TArray<FBuildingFootprintPolygon>& Footprints,
		const FString& MapName,
		FString& OutError);
}
