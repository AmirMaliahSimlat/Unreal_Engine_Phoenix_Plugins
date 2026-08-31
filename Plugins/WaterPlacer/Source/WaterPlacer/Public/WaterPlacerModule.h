#pragma once

#include "Modules/ModuleManager.h"

class FWaterPlacerModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
