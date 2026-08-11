#include "TreePlacerModule.h"
#include "TreePlacerLog.h"

#define LOCTEXT_NAMESPACE "FTreePlacerModule"

void FTreePlacerModule::StartupModule()
{
	UE_LOG(LogTreePlacer, Log, TEXT("TreePlacer module started. Filter Output Log by 'LogTreePlacer'."));
}

void FTreePlacerModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FTreePlacerModule, TreePlacer)
