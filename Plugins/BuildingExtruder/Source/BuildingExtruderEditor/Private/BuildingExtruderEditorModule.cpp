#include "BuildingExtruderEditorModule.h"

#include "BuildingExtruderLog.h"
#include "BuildingExtruderTileActor.h"

#include "Editor.h"
#include "Engine/World.h"
#include "EngineUtils.h"

#define LOCTEXT_NAMESPACE "FBuildingExtruderEditorModule"

namespace
{
	void ClearEditorOnlyCookFlags(UWorld* World)
	{
		if (!World)
		{
			return;
		}

		int32 FixedActors = 0;
		for (TActorIterator<ABuildingExtruderTileActor> It(World); It; ++It)
		{
			ABuildingExtruderTileActor* Actor = *It;
			if (!Actor)
			{
				continue;
			}

			bool bDirty = false;
			if (Actor->bIsEditorOnlyActor)
			{
				Actor->bIsEditorOnlyActor = false;
				bDirty = true;
			}
			if (Actor->IsHidden())
			{
				Actor->SetActorHiddenInGame(false);
				bDirty = true;
			}

			TArray<UActorComponent*> Comps;
			Actor->GetComponents<UActorComponent>(Comps);
			for (UActorComponent* Comp : Comps)
			{
				if (Comp && Comp->bIsEditorOnly)
				{
					Comp->bIsEditorOnly = false;
					bDirty = true;
				}
			}

			if (bDirty)
			{
				Actor->Modify();
				++FixedActors;
			}
		}

		if (FixedActors > 0)
		{
			World->MarkPackageDirty();
			UE_LOG(
				LogBuildingExtruder,
				Warning,
				TEXT("Cleared editor-only flags on %d building tile actor(s). Save the map, then cook/export the pak."),
				FixedActors);
		}
	}
}

void FBuildingExtruderEditorModule::StartupModule()
{
	UE_LOG(LogBuildingExtruder, Log, TEXT("BuildingExtruder editor module started. Filter Output Log by 'LogBuildingExtruder'."));

	MapOpenedHandle = FEditorDelegates::OnMapOpened.AddLambda([](const FString& /*Filename*/, bool /*bAsTemplate*/)
	{
		if (!GEditor)
		{
			return;
		}
		ClearEditorOnlyCookFlags(GEditor->GetEditorWorldContext().World());
	});
}

void FBuildingExtruderEditorModule::ShutdownModule()
{
	if (MapOpenedHandle.IsValid())
	{
		FEditorDelegates::OnMapOpened.Remove(MapOpenedHandle);
		MapOpenedHandle.Reset();
	}
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FBuildingExtruderEditorModule, BuildingExtruderEditor)
