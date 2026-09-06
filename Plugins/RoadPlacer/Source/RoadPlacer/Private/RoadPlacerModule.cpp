#include "RoadPlacerModule.h"
#include "RoadPlacerLog.h"

#define LOCTEXT_NAMESPACE "FRoadPlacerModule"

void FRoadPlacerModule::StartupModule()
{
	UE_LOG(LogRoadPlacer, Log, TEXT("RoadPlacer module started. Filter Output Log by 'LogRoadPlacer'."));
}

void FRoadPlacerModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FRoadPlacerModule, RoadPlacer)
