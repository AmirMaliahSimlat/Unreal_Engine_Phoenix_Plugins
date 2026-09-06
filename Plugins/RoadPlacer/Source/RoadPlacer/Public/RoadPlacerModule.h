#pragma once

#include "Modules/ModuleManager.h"

class FRoadPlacerModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
