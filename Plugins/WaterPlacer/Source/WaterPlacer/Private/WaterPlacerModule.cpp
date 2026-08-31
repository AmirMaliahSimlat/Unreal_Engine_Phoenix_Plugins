#include "WaterPlacerModule.h"
#include "WaterPlacerLog.h"

#define LOCTEXT_NAMESPACE "FWaterPlacerModule"

void FWaterPlacerModule::StartupModule()
{
	UE_LOG(LogWaterPlacer, Log, TEXT("WaterPlacer module started. Filter Output Log by 'LogWaterPlacer'."));
}

void FWaterPlacerModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FWaterPlacerModule, WaterPlacer)
