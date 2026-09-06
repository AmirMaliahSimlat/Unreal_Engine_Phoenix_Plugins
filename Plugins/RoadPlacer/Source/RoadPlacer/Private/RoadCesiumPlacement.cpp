#include "RoadCesiumPlacement.h"

#include "CesiumGeoreference.h"
#include "EngineUtils.h"

ACesiumGeoreference* RoadCesiumPlacement::FindGeoreference(UWorld* World)
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

FVector RoadCesiumPlacement::LonLatHeightToUnreal(
	ACesiumGeoreference& Georeference,
	double LonDeg,
	double LatDeg,
	double HeightM)
{
	const FVector LLH(LonDeg, LatDeg, HeightM);
	const FVector LocalPos = Georeference.TransformLongitudeLatitudeHeightPositionToUnreal(LLH);
	return Georeference.GetActorTransform().TransformPosition(LocalPos);
}
