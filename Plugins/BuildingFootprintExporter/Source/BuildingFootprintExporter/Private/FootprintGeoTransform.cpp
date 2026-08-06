#include "FootprintGeoTransform.h"

namespace
{
	constexpr double MetersPerDegreeLat = 111320.0;
}

FFootprintGeoTransform FFootprintGeoTransform::FromSettings(const UBuildingFootprintExportSettings& Settings)
{
	FFootprintGeoTransform T;
	T.OriginLatitude = Settings.OriginLatitude;
	T.OriginLongitude = Settings.OriginLongitude;
	T.UnrealUnitsPerMeter = 100.0;
	return T;
}

void FFootprintGeoTransform::UnrealToEastNorthMeters(const FVector& UnrealWorldCm, double& OutEastM, double& OutNorthM) const
{
	// Geo origin is always Unreal (0,0,0). +X = East, +Y = South.
	const double X_m = UnrealWorldCm.X / UnrealUnitsPerMeter;
	const double Y_m = UnrealWorldCm.Y / UnrealUnitsPerMeter;
	OutEastM = X_m;
	OutNorthM = -Y_m;
}

FVector2D FFootprintGeoTransform::UnrealToLonLat(const FVector& UnrealWorldCm) const
{
	double EastM = 0.0;
	double NorthM = 0.0;
	UnrealToEastNorthMeters(UnrealWorldCm, EastM, NorthM);

	const double LatRad = FMath::DegreesToRadians(OriginLatitude);
	const double CosLat = FMath::Max(FMath::Abs(FMath::Cos(LatRad)), 1.0e-12);
	const double MetersPerDegreeLon = MetersPerDegreeLat * CosLat;

	const double Lat = OriginLatitude + (NorthM / MetersPerDegreeLat);
	const double Lon = OriginLongitude + (EastM / MetersPerDegreeLon);
	return FVector2D(Lon, Lat);
}
