#include "BuildingExtruderEditorModule.h"

#include "BuildingExtruderLog.h"
#include "BuildingTileSmaUtils.h"

#include "Editor.h"
#include "Engine/World.h"

#define LOCTEXT_NAMESPACE "FBuildingExtruderEditorModule"

void FBuildingExtruderEditorModule::StartupModule()
{
	UE_LOG(LogBuildingExtruder, Log, TEXT("BuildingExtruder editor module started. Filter Output Log by 'LogBuildingExtruder'."));

	MapOpenedHandle = FEditorDelegates::OnMapOpened.AddLambda([](const FString& /*Filename*/, bool /*bAsTemplate*/)
	{
		if (!GEditor)
		{
			return;
		}
		if (UWorld* World = GEditor->GetEditorWorldContext().World())
		{
			BuildingTileSmaUtils::ConvertAllTileActors(*World);
		}
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
