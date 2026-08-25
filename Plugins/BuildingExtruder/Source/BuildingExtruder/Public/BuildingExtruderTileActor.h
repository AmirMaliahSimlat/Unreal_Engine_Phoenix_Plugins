#pragma once

#include "GameFramework/Actor.h"
#include "BuildingExtruderTileActor.generated.h"

/** One map tile: Building → Wall/Roof scene folders, then one StaticMeshComponent per material.
 * Lives in the Runtime module so cooked pak / simulator builds can spawn it. */
UCLASS()
class BUILDINGEXTRUDER_API ABuildingExtruderTileActor : public AActor
{
	GENERATED_BODY()

public:
	ABuildingExtruderTileActor();
};
