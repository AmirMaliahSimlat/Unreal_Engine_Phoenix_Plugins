#pragma once

#include "CoreMinimal.h"

struct FGroundTriangle2D
{
	FVector2D A;
	FVector2D B;
	FVector2D C;
	double ZA = 0.0;
	double ZB = 0.0;
	double ZC = 0.0;
};

struct FSilhouettePolygon2D
{
	TArray<FVector2D> OuterRingCm;
	TArray<TArray<FVector2D>> HoleRingsCm;
	double AreaM2 = 0.0;
	/** Building height in meters: Zmax - Zmin of geometry covering this footprint. */
	double HeightM = 0.0;
};

namespace FootprintSilhouette
{
	/**
	 * Build ground-silhouette polygons from projected triangles.
	 * Rasterizes at CellSizeCm, extracts outer rings + holes, then simplifies.
	 */
	TArray<FSilhouettePolygon2D> BuildSilhouettesFromTriangles(
		const TArray<FGroundTriangle2D>& TrianglesCm,
		double CellSizeCm,
		double SimplifyToleranceCm,
		int32 MaxGridDimension,
		double UnrealUnitsPerMeter);

	/**
	 * Union filled 2D rings (cm) into silhouette polygons.
	 * Used to merge footprints across actors (e.g. walls actor + roofs actor).
	 * HeightM on outputs is set to ForcedHeightM.
	 */
	TArray<FSilhouettePolygon2D> BuildSilhouettesFromFilledRings(
		const TArray<TArray<FVector2D>>& OuterRingsCm,
		double CellSizeCm,
		double SimplifyToleranceCm,
		int32 MaxGridDimension,
		double UnrealUnitsPerMeter,
		double ForcedHeightM);
}
