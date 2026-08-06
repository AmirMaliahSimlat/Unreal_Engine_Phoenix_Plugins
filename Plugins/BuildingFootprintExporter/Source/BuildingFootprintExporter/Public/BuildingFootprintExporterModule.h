#pragma once

#include "Modules/ModuleManager.h"

class FBuildingFootprintExporterModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
