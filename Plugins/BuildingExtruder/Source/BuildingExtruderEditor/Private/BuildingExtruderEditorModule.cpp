#include "BuildingExtruderEditorModule.h"
#include "BuildingExtruderLog.h"

#define LOCTEXT_NAMESPACE "FBuildingExtruderEditorModule"

void FBuildingExtruderEditorModule::StartupModule()
{
	UE_LOG(LogBuildingExtruder, Log, TEXT("BuildingExtruder editor module started. Filter Output Log by 'LogBuildingExtruder'."));
}

void FBuildingExtruderEditorModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FBuildingExtruderEditorModule, BuildingExtruderEditor)
