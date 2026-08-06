#include "BuildingExtruderModule.h"
#include "BuildingExtruderLog.h"

#define LOCTEXT_NAMESPACE "FBuildingExtruderModule"

void FBuildingExtruderModule::StartupModule()
{
	UE_LOG(LogBuildingExtruder, Log, TEXT("BuildingExtruder module started. Filter Output Log by 'LogBuildingExtruder'."));
}

void FBuildingExtruderModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FBuildingExtruderModule, BuildingExtruder)
