#include "BuildingFootprintExporterModule.h"
#include "BuildingFootprintExporterLog.h"

#define LOCTEXT_NAMESPACE "FBuildingFootprintExporterModule"

void FBuildingFootprintExporterModule::StartupModule()
{
	UE_LOG(LogBuildingFootprintExporter, Log, TEXT("BuildingFootprintExporter module started. Filter Output Log by 'LogBuildingFootprintExporter'."));
}

void FBuildingFootprintExporterModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FBuildingFootprintExporterModule, BuildingFootprintExporter)
