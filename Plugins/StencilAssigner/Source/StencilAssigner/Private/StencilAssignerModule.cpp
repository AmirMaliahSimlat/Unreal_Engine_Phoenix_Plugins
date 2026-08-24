#include "StencilAssignerModule.h"
#include "StencilAssignerLog.h"

#define LOCTEXT_NAMESPACE "FStencilAssignerModule"

void FStencilAssignerModule::StartupModule()
{
	UE_LOG(LogStencilAssigner, Log, TEXT("StencilAssigner module started. Filter Output Log by 'LogStencilAssigner'."));
}

void FStencilAssignerModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FStencilAssignerModule, StencilAssigner)
