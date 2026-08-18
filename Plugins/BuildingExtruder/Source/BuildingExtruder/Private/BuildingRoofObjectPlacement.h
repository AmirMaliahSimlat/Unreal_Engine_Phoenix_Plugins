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
	double LocalMinX = 0.0;
	double LocalMinY = 0.0;
	double LocalMaxX = 0.0;
	double LocalMaxY = 0.0;
};

namespace BuildingRoofObjectPlacement
{
	FRoofObjectFootprint MakeFootprint(const UStaticMesh& Mesh);

	/**
	 * Picks a random yaw-only pose on the usable roof faces.
	 * The full XY box (rotated) must stay inside one roof face with a gap, and not overlap Occupied.
	 * On slopes, Z is the lowest roof height under the box so the downhill edge sits on the roof.
	 */
	bool TryPlace(
		const TArray<FRoofPlaceTriangle>& WorldTris,
		const FRoofObjectFootprint& Foot,
		const TArray<FPlacedRoofObject2D>& Occupied,
		FRandomStream& Rng,
		FTransform& OutXform,
		FPlacedRoofObject2D& OutOccupied);
}
