#include "BuildingExtruderTileActor.h"

#include "Components/SceneComponent.h"

ABuildingExtruderTileActor::ABuildingExtruderTileActor()
{
	PrimaryActorTick.bCanEverTick = false;
	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	Root->SetMobility(EComponentMobility::Static);
	SetRootComponent(Root);
}
