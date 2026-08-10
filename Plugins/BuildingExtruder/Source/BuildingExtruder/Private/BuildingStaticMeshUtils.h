#pragma once

#include "CoreMinimal.h"
#include "BuildingExtrudeUtils.h"

class UStaticMesh;
class UObject;
class UMaterialInterface;

namespace BuildingStaticMeshUtils
{
	/**
	 * Two-sided building material saved under /Game/BuildingExtruder/Materials.
	 * Created once if missing; then loaded from disk.
	 */
	UMaterialInterface* GetOrCreateBuildingMaterial(FString& OutError);

	/**
	 * Builds a UStaticMesh asset under PackagePath (e.g. /Game/BuildingExtruder/Meshes/BldgTile_0_0_Walls)
	 * and saves the package so level references survive undo and map save.
	 */
	UStaticMesh* CreatePersistentStaticMesh(
		const FString& PackagePath,
		const FString& AssetName,
		const FExtrudedPrismMesh& Mesh,
		UMaterialInterface* Material,
		FString& OutError);
}
