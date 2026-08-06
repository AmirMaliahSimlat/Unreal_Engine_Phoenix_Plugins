#pragma once

#include "CoreMinimal.h"

class AStaticMeshActor;
class UWorld;

namespace BuildingFbxExporter
{
	/**
	 * Exports spawned tile StaticMeshActors to FBX using Unreal's native editor exporter
	 * (same path as File/Export Selected). Produces binary FBX that re-imports cleanly.
	 */
	bool ExportTileActors(
		UWorld& World,
		const TArray<AStaticMeshActor*>& TileActors,
		const FString& OutputPath,
		FString& OutError);
}
