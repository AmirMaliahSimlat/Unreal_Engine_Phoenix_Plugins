#include "BuildingExtruderTileActor.h"

#include "Components/ActorComponent.h"
#include "Components/SceneComponent.h"

ABuildingExtruderTileActor::ABuildingExtruderTileActor()
{
	PrimaryActorTick.bCanEverTick = false;
	bIsEditorOnlyActor = false;
	SetActorHiddenInGame(false);

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	Root->SetMobility(EComponentMobility::Static);
	Root->bIsEditorOnly = false;
	SetRootComponent(Root);
}

void ABuildingExtruderTileActor::PostLoad()
{
	Super::PostLoad();
	bIsEditorOnlyActor = false;
	SetActorHiddenInGame(false);

	TArray<UActorComponent*> Comps;
	GetComponents<UActorComponent>(Comps);
	for (UActorComponent* Comp : Comps)
	{
		if (Comp)
		{
			Comp->bIsEditorOnly = false;
		}
	}
}

bool ABuildingExtruderTileActor::IsEditorOnly() const
{
	return false;
}

bool ABuildingExtruderTileActor::NeedsLoadForClient() const
{
	return true;
}

bool ABuildingExtruderTileActor::NeedsLoadForServer() const
{
	return true;
}
