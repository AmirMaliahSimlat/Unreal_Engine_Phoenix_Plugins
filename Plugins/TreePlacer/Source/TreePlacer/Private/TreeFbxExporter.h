#pragma once

#include "CoreMinimal.h"

class AActor;
class UWorld;

namespace TreeFbxExporter
{
	bool ExportTileActors(
		UWorld& World,
		const TArray<AActor*>& TileActors,
		const FString& OutputPath,
		FString& OutError);
}
