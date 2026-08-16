#pragma once

#include "GameFramework/Actor.h"
#include "TreePlacerTileActor.generated.h"

/** Legacy tile actor kept so older levels that already spawned it still load. New placement uses AInstancedFoliageActor. */
UCLASS()
class TREEPLACER_API ATreePlacerTileActor : public AActor
{
	GENERATED_BODY()

public:
	ATreePlacerTileActor();
};
