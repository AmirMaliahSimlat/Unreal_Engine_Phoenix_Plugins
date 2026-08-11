#include "TreePlacerTileActor.h"
#include "Components/SceneComponent.h"

ATreePlacerTileActor::ATreePlacerTileActor()
{
	PrimaryActorTick.bCanEverTick = false;
	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
}
