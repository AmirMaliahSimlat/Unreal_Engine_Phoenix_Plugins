#pragma once

#include "CoreMinimal.h"

struct FExtrudedPrismMesh
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;

	/** One entry per triangle (Triangles.Num()/3). Material slot / section index. */
	TArray<int32> TriangleMaterialIndices;
};

enum class EBuildingRoofType : uint8
{
	Flat,
	Hipped,
	Parapet
};

namespace BuildingExtrudeUtils
{
	/** Maps a DBF roof-type code onto one of the three supported shapes. Unknown → Flat. */
	EBuildingRoofType ResolveRoofType(
		int32 RoofTypeCode,
		int32 FlatIndex,
		int32 HippedIndex,
		int32 ParapetIndex);
	/** Assigns every triangle in the mesh to MaterialSlotIndex. */
	void AssignAllTrianglesMaterialSlot(FExtrudedPrismMesh& Mesh, int32 MaterialSlotIndex);

	/**
	 * Builds separate wall+floor and roof meshes from matching base/top rings in actor-local space.
	 * Triangulates using base ring XY (ear clipping). Side faces connect base[i]→top[i].
	 * WallsAndFloor = bottom cap + vertical sides. Roof = top cap only.
	 * Normals face outward; UVs use MetersPerUv density.
	 */
	bool BuildPrismPartsFromRings(
		const TArray<FVector>& BaseRingLocal,
		const TArray<FVector>& TopRingLocal,
		double MetersPerUv,
		FExtrudedPrismMesh& OutWallsAndFloor,
		FExtrudedPrismMesh& OutRoof,
		FString& OutError);

	/** Flat roof (current prism: walls + floor + planar top cap). */
	bool BuildFlatRoofPartsFromRings(
		const TArray<FVector>& BaseRingLocal,
		const TArray<FVector>& TopRingLocal,
		double MetersPerUv,
		FExtrudedPrismMesh& OutWallsAndFloor,
		FExtrudedPrismMesh& OutRoof,
		FString& OutError);

	/** Hipped / cross-hipped roof (walls stay rectangular; ridge = HippedHeightMeters above eaves). */
	bool BuildHippedRoofPartsFromRings(
		const TArray<FVector>& BaseRingLocal,
		const TArray<FVector>& TopRingLocal,
		double MetersPerUv,
		double HippedHeightMeters,
		FExtrudedPrismMesh& OutWallsAndFloor,
		FExtrudedPrismMesh& OutRoof,
		FString& OutError);

	/** Parapet roof: inner deck lowered by height; ring width inward; inner walls on the roof mesh. */
	bool BuildParapetRoofPartsFromRings(
		const TArray<FVector>& BaseRingLocal,
		const TArray<FVector>& TopRingLocal,
		double MetersPerUv,
		double ParapetHeightMeters,
		double ParapetWidthMeters,
		FExtrudedPrismMesh& OutWallsAndFloor,
		FExtrudedPrismMesh& OutRoof,
		FString& OutError);

	/** Dispatches to the matching roof-shape builder. */
	bool BuildRoofPartsFromRings(
		EBuildingRoofType RoofType,
		const TArray<FVector>& BaseRingLocal,
		const TArray<FVector>& TopRingLocal,
		double MetersPerUv,
		double ParapetHeightMeters,
		double ParapetWidthMeters,
		double HippedHeightMeters,
		FExtrudedPrismMesh& OutWallsAndFloor,
		FExtrudedPrismMesh& OutRoof,
		FString& OutError);

	/**
	 * Builds a full prism (walls + floor + roof) from matching base/top rings.
	 */
	bool BuildPrismFromRings(
		const TArray<FVector>& BaseRingLocal,
		const TArray<FVector>& TopRingLocal,
		double MetersPerUv,
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
