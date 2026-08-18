#pragma once

#include "CoreMinimal.h"
#include "BuildingExtrudeUtils.h"

class UStaticMesh;

struct FPlacedRoofObject2D
{
	FVector2D CenterXY = FVector2D::ZeroVector;
	double RadiusCm = 0.0;
};

struct FRoofObjectFootprint
{
	double RadiusCm = 0.0;
	double PivotZMin = 0.0;
};

namespace BuildingRoofObjectPlacement
{
	FRoofObjectFootprint MakeFootprint(const UStaticMesh& Mesh);

	/**
	 * Picks a random yaw-only pose on the usable roof faces.
	 * Circle of RadiusCm+margin must stay inside a face (XY) and not overlap Occupied.
	 */
	bool TryPlace(
		const TArray<FRoofPlaceTriangle>& WorldTris,
		const FRoofObjectFootprint& Foot,
		const TArray<FPlacedRoofObject2D>& Occupied,
		FRandomStream& Rng,
		FTransform& OutXform,
		FPlacedRoofObject2D& OutOccupied);
}
