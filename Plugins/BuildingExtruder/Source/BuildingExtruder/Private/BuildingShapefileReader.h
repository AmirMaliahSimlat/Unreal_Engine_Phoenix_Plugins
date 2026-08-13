#pragma once

#include "CoreMinimal.h"

struct FBuildingShapefileFeature
{
	/** Outer ring in EPSG:4326 (WGS84): X = longitude degrees, Y = latitude degrees. Not necessarily closed. */
	TArray<FVector2D> OuterRingLonLat;

	/** Extrusion / wall height in meters (HeightFieldName). */
	double HeightM = 0.0;

	/** Floor altitude in meters (AltitudeFieldName from DBF). */
	double ElevationM = 0.0;

	/** Raw roof-type enumeration from RoofTypeFieldName (integer code). */
	int32 RoofTypeCode = 0;

	int32 RecordIndex = 0;
};

namespace BuildingShapefileReader
{
	/**
	 * Reads polygon shapefile (.shp + .dbf). Path may be with or without .shp extension.
	 * Coordinates are interpreted as EPSG:4326 lon/lat degrees (no reprojection).
	 * HeightFieldName, ElevationFieldName, and RoofTypeFieldName are required DBF columns.
	 */
	bool ReadPolygonBuildings(
		const FString& ShapefilePath,
		const FString& HeightFieldName,
		const FString& ElevationFieldName,
		const FString& RoofTypeFieldName,
		TArray<FBuildingShapefileFeature>& OutFeatures,
		FString& OutError);
}
