#pragma once

#include "GameFramework/Actor.h"
#include "TreePlacerTileActor.generated.h"

/** Tile actor that owns Hierarchical Instanced Static Mesh components for trees. */
UCLASS()
class TREEPLACER_API ATreePlacerTileActor : public AActor
{
	GENERATED_BODY()

public:
	ATreePlacerTileActor();
};
