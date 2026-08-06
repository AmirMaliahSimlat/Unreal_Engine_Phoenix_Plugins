#pragma once

#include "Modules/ModuleManager.h"

class FBuildingExtruderModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
