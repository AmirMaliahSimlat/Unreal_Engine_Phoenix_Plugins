#pragma once

#include "CoreMinimal.h"

class UStaticMesh;

namespace TreeMeshFolderLoader
{
	/**
	 * Loads UStaticMesh assets from a Content folder path (e.g. /Game/Trees).
	 * Also accepts UFoliageType_InstancedStaticMesh and uses their Mesh.
	 * Recursive. Duplicates (same mesh from foliage + static mesh) are removed.
	 */
	bool LoadTreeMeshesFromFolder(
		const FString& ContentFolderPath,
		TArray<UStaticMesh*>& OutMeshes,
		FString& OutError);
}
