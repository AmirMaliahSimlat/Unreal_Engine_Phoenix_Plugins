#include "BuildingCesiumPlacement.h"
#include "BuildingExtruderLog.h"

#include "CesiumGeoreference.h"
#include "EngineUtils.h"

ACesiumGeoreference* BuildingCesiumPlacement::FindGeoreference(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}

	if (ACesiumGeoreference* DefaultGeo = ACesiumGeoreference::GetDefaultGeoreference(World))
	{
		return DefaultGeo;
	}

	for (TActorIterator<ACesiumGeoreference> It(World); It; ++It)
	{
		if (*It)
		{
			return *It;
		}
	}
	return nullptr;
}

FVector BuildingCesiumPlacement::LonLatHeightToUnreal(
	ACesiumGeoreference& Georeference,
	double LonDeg,
	double LatDeg,
	double HeightM)
{
	const FVector LLH(LonDeg, LatDeg, HeightM);
	// Cesium for Unreal 5.1 and 5.3: Position suffix. Result is in the
	// Georeference actor's local frame — convert to world.
	const FVector LocalPos = Georeference.TransformLongitudeLatitudeHeightPositionToUnreal(LLH);
	return Georeference.GetActorTransform().TransformPosition(LocalPos);
}
