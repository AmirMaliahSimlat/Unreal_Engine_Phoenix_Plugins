#pragma once

#include "CoreMinimal.h"
#include "BuildingFootprintExportSettings.h"

struct FFootprintGeoTransform
{
	double OriginLatitude = 0.0;
	double OriginLongitude = 0.0;
	double UnrealUnitsPerMeter = 100.0;

	static FFootprintGeoTransform FromSettings(const UBuildingFootprintExportSettings& Settings);

	/** Convert Unreal world cm position to WGS84 lon/lat degrees (X=Lon, Y=Lat). */
	FVector2D UnrealToLonLat(const FVector& UnrealWorldCm) const;

	/**
	 * East/North meters relative to geographic origin.
	 * Fixed convention: Unreal +X = East, +Y = South.
	 */
	void UnrealToEastNorthMeters(const FVector& UnrealWorldCm, double& OutEastM, double& OutNorthM) const;
};
