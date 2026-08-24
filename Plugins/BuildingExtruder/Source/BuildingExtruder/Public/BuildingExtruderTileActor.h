#pragma once

#include "GameFramework/Actor.h"
#include "BuildingExtruderTileActor.generated.h"

/** One map tile: Building → Wall/Roof scene folders, then one StaticMeshComponent per material. */
UCLASS()
class BUILDINGEXTRUDER_API ABuildingExtruderTileActor : public AActor
{
	GENERATED_BODY()

public:
	ABuildingExtruderTileActor();
};
