#pragma once

#include "CoreMinimal.h"

class ACesiumGeoreference;
class UWorld;

namespace TreeCesiumPlacement
{
	ACesiumGeoreference* FindGeoreference(UWorld* World);

	FVector LonLatHeightToUnreal(
		ACesiumGeoreference& Georeference,
		double LonDeg,
		double LatDeg,
		double HeightM);
}
