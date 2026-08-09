#pragma once

#include "CoreMinimal.h"

struct FExtrudedPrismMesh
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
};

namespace BuildingExtrudeUtils
{
	/**
	 * Builds separate wall+floor and roof meshes from matching base/top rings in actor-local space.
	 * Triangulates using base ring XY (ear clipping). Side faces connect base[i]→top[i].
	 * WallsAndFloor = bottom cap + vertical sides. Roof = top cap only.
	 */
	bool BuildPrismPartsFromRings(
		const TArray<FVector>& BaseRingLocal,
		const TArray<FVector>& TopRingLocal,
		FExtrudedPrismMesh& OutWallsAndFloor,
		FExtrudedPrismMesh& OutRoof,
		FString& OutError);

	/**
	 * Builds a full prism (walls + floor + roof) from matching base/top rings.
	 */
	bool BuildPrismFromRings(
		const TArray<FVector>& BaseRingLocal,
		const TArray<FVector>& TopRingLocal,
		FExtrudedPrismMesh& OutMesh,
		FString& OutError);

	/**
	 * Constant-height extrusion along +Z (cm). Convenience wrapper around BuildPrismFromRings.
	 */
	bool BuildPrism(
		const TArray<FVector>& BaseRingLocal,
		double HeightCm,
		FExtrudedPrismMesh& OutMesh,
		FString& OutError);
}
