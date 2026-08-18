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
	 * Picks a pose on the usable roof faces. Yaw follows the slope eave (hipped) or the
	 * closest footprint edge (flat / parapet), with local +Y into the roof so the same
	 * side of the mesh faces the edge. The full XY box must stay inside one face with a
	 * gap, and not overlap Occupied. On slopes, Z is the lowest roof height under the box
	 * so the downhill edge sits on the roof.
	 */
	bool TryPlace(
		const TArray<FRoofPlaceTriangle>& WorldTris,
		const TArray<FVector2D>& FootprintXY,
		const FRoofObjectFootprint& Foot,
		const TArray<FPlacedRoofObject2D>& Occupied,
		FRandomStream& Rng,
		FTransform& OutXform,
		FPlacedRoofObject2D& OutOccupied);
}
