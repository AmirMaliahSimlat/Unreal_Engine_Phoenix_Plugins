#pragma once

#include "CoreMinimal.h"

class UStaticMesh;

namespace BuildingMeshFolderLoader
{
	/** Loads unique UStaticMesh assets from a Content folder (recursive). */
	bool LoadStaticMeshesFromFolder(
		const FString& ContentFolderPath,
		TArray<UStaticMesh*>& OutMeshes,
		FString& OutError);
}
