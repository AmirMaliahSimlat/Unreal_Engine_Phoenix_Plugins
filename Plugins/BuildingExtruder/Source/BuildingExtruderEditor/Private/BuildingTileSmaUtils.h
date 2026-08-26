#pragma once

#include "CoreMinimal.h"

class AStaticMeshActor;
class ABuildingExtruderTileActor;
class UWorld;

namespace BuildingTileSmaUtils
{
	void ConfigureEmptyTileRoot(AStaticMeshActor& Actor);

	AStaticMeshActor* ConvertTileActor(ABuildingExtruderTileActor& Source);

	int32 ConvertAllTileActors(UWorld& World);
}
