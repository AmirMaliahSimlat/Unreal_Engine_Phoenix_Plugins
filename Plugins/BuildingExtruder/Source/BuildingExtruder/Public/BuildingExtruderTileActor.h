#pragma once

#include "GameFramework/Actor.h"
#include "BuildingExtruderTileActor.generated.h"

/** Legacy tile actor kept so older levels that already spawned it still load in the editor. New extrudes use AStaticMeshActor. */
UCLASS()
class BUILDINGEXTRUDER_API ABuildingExtruderTileActor : public AActor
{
	GENERATED_BODY()

public:
	ABuildingExtruderTileActor();
};
