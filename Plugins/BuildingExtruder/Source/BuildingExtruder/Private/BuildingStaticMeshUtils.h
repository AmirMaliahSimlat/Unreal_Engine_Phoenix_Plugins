#pragma once

#include "CoreMinimal.h"
#include "BuildingExtrudeUtils.h"

class UStaticMesh;
class UObject;
class UMaterialInterface;

namespace BuildingStaticMeshUtils
{
	/** Default two-sided preview material for extruded buildings. */
	UMaterialInterface* GetTwoSidedBuildingMaterial();

	/** Distinct translucent material for diagnose pass-2 (deep refine) preview tiles. */
	UMaterialInterface* GetDiagnoseDeepCompareMaterial();

	/** Builds a transient UStaticMesh from prism triangle data (editor preview). */
	UStaticMesh* CreateTransientStaticMesh(
		UObject* Outer,
		FName MeshName,
		const FExtrudedPrismMesh& Mesh,
		UMaterialInterface* Material,
		FString& OutError);
}
