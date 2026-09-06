#pragma once

#include "CoreMinimal.h"

class ACesiumGeoreference;
class UWorld;

namespace RoadCesiumPlacement
{
	ACesiumGeoreference* FindGeoreference(UWorld* World);

	FVector LonLatHeightToUnreal(
		ACesiumGeoreference& Georeference,
		double LonDeg,
		double LatDeg,
		double HeightM);
}
