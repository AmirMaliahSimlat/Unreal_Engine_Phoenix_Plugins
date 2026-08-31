#pragma once

#include "CoreMinimal.h"

class ACesiumGeoreference;
class UWorld;

namespace WaterCesiumPlacement
{
	ACesiumGeoreference* FindGeoreference(UWorld* World);

	FVector LonLatHeightToUnreal(
		ACesiumGeoreference& Georeference,
		double LonDeg,
		double LatDeg,
		double HeightM);
}
