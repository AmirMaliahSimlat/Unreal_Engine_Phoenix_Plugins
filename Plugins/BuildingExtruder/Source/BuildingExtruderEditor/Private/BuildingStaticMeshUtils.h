#pragma once

#include "CoreMinimal.h"
#include "BuildingExtrudeUtils.h"

class UStaticMesh;
class UObject;
class UMaterialInterface;

namespace BuildingStaticMeshUtils
{
	/**
	 * Default building material under /Game/BuildingExtruder/Materials.
	 * One-sided (outward normals) for correct lighting/shadows.
	 */
	UMaterialInterface* GetOrCreateBuildingMaterial(FString& OutError);

	/**
	 * Builds a UStaticMesh asset with NumMaterialSlots sections.
	 * Triangles use Mesh.TriangleMaterialIndices; unused slots still exist so materials can be assigned later.
	 */
	UStaticMesh* CreatePersistentStaticMesh(
		const FString& PackagePath,
		const FString& AssetName,
		const FExtrudedPrismMesh& Mesh,
		UMaterialInterface* Material,
		int32 NumMaterialSlots,
		FString& OutError);
}
