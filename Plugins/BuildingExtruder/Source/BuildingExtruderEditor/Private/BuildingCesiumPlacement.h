#pragma once

#include "CoreMinimal.h"

class ACesiumGeoreference;
class UWorld;

namespace BuildingCesiumPlacement
{
	/** Finds Cesium georeference in the world (default or first). */
	ACesiumGeoreference* FindGeoreference(UWorld* World);

	/**
	 * Lon/Lat degrees + height meters (ellipsoid) -> Unreal world position.
	 * Uses TransformLongitudeLatitudeHeightPositionToUnreal (current Cesium API).
	 */
	FVector LonLatHeightToUnreal(ACesiumGeoreference& Georeference, double LonDeg, double LatDeg, double HeightM);
}
