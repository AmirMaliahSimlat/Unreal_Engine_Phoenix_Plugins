#include "BuildingTileSmaUtils.h"

#include "BuildingExtruderLog.h"
#include "BuildingExtruderTileActor.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"

namespace
{
	void CopyComponentTags(const UActorComponent& From, UActorComponent& To)
	{
		for (const FName& Tag : From.ComponentTags)
		{
			To.ComponentTags.AddUnique(Tag);
		}
	}

	USceneComponent* CloneAttachedTree(
		AActor& NewOwner,
		USceneComponent& NewParent,
		USceneComponent& OldComp)
	{
		USceneComponent* NewComp = nullptr;
		if (UStaticMeshComponent* OldSmc = Cast<UStaticMeshComponent>(&OldComp))
		{
			UStaticMeshComponent* NewSmc = NewObject<UStaticMeshComponent>(
				&NewOwner, OldComp.GetFName(), RF_Transactional);
			if (!NewSmc)
			{
				return nullptr;
			}
			NewSmc->SetMobility(EComponentMobility::Static);
			NewSmc->bIsEditorOnly = false;
			NewSmc->SetupAttachment(&NewParent);
			NewOwner.AddInstanceComponent(NewSmc);
			NewSmc->RegisterComponent();
			NewSmc->SetStaticMesh(OldSmc->GetStaticMesh());
			NewSmc->SetCollisionEnabled(OldSmc->GetCollisionEnabled());
			NewSmc->SetRelativeTransform(OldSmc->GetRelativeTransform());
			const int32 NumMats = OldSmc->GetNumMaterials();
			for (int32 Slot = 0; Slot < NumMats; ++Slot)
			{
				NewSmc->SetMaterial(Slot, OldSmc->GetMaterial(Slot));
			}
			CopyComponentTags(*OldSmc, *NewSmc);
			NewComp = NewSmc;
		}
		else
		{
			USceneComponent* NewFolder = NewObject<USceneComponent>(
				&NewOwner, OldComp.GetFName(), RF_Transactional);
			if (!NewFolder)
			{
				return nullptr;
			}
			NewFolder->SetMobility(EComponentMobility::Static);
			NewFolder->bIsEditorOnly = false;
			NewFolder->SetupAttachment(&NewParent);
			NewOwner.AddInstanceComponent(NewFolder);
			NewFolder->RegisterComponent();
			NewFolder->SetRelativeTransform(OldComp.GetRelativeTransform());
			CopyComponentTags(OldComp, *NewFolder);
			NewComp = NewFolder;
		}

		if (!NewComp)
		{
			return nullptr;
		}

		TArray<USceneComponent*> Children;
		OldComp.GetChildrenComponents(false, Children);
		for (USceneComponent* Child : Children)
		{
			if (Child && !CloneAttachedTree(NewOwner, *NewComp, *Child))
			{
				return nullptr;
			}
		}
		return NewComp;
	}
}

void BuildingTileSmaUtils::ConfigureEmptyTileRoot(AStaticMeshActor& Actor)
{
	Actor.bIsEditorOnlyActor = false;
	Actor.SetActorHiddenInGame(false);

	UStaticMeshComponent* Native = Actor.GetStaticMeshComponent();
	if (!Native)
	{
		return;
	}
	Native->SetMobility(EComponentMobility::Static);
	Native->bIsEditorOnly = false;
	Native->SetStaticMesh(nullptr);
	Native->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

AStaticMeshActor* BuildingTileSmaUtils::ConvertTileActor(ABuildingExtruderTileActor& Source)
{
	UWorld* World = Source.GetWorld();
	USceneComponent* OldRoot = Source.GetRootComponent();
	if (!World || !OldRoot)
	{
		return nullptr;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	SpawnParams.ObjectFlags |= RF_Transactional;

	AStaticMeshActor* Dest = World->SpawnActor<AStaticMeshActor>(
		Source.GetActorLocation(), Source.GetActorRotation(), SpawnParams);
	if (!Dest || !Dest->GetRootComponent())
	{
		return nullptr;
	}

	ConfigureEmptyTileRoot(*Dest);
	Dest->SetActorScale3D(Source.GetActorScale3D());
#if WITH_EDITOR
	Dest->SetActorLabel(Source.GetActorLabel());
	Dest->SetFolderPath(Source.GetFolderPath());
#endif
	for (const FName& Tag : Source.Tags)
	{
		Dest->Tags.AddUnique(Tag);
	}

	TArray<USceneComponent*> Children;
	OldRoot->GetChildrenComponents(false, Children);
	for (USceneComponent* Child : Children)
	{
		if (Child && !CloneAttachedTree(*Dest, *Dest->GetRootComponent(), *Child))
		{
			Dest->Destroy();
			return nullptr;
		}
	}

	Dest->Modify();
	Source.Destroy();
	return Dest;
}

int32 BuildingTileSmaUtils::ConvertAllTileActors(UWorld& World)
{
	TArray<ABuildingExtruderTileActor*> Tiles;
	for (TActorIterator<ABuildingExtruderTileActor> It(&World); It; ++It)
	{
		if (*It)
		{
			Tiles.Add(*It);
		}
	}

	int32 Converted = 0;
	for (ABuildingExtruderTileActor* Tile : Tiles)
	{
		if (!Tile)
		{
			continue;
		}
		if (ConvertTileActor(*Tile))
		{
			++Converted;
		}
		else
		{
			UE_LOG(
				LogBuildingExtruder,
				Error,
				TEXT("Failed to convert building tile '%s' to StaticMeshActor."),
				*Tile->GetName());
		}
	}

	if (Converted > 0)
	{
		World.MarkPackageDirty();
		UE_LOG(
			LogBuildingExtruder,
			Warning,
			TEXT("Converted %d building tile actor(s) to StaticMeshActor (Wall/Roof meshes kept as extra components). Save the map, then cook/export the pak."),
			Converted);
	}
	return Converted;
}
