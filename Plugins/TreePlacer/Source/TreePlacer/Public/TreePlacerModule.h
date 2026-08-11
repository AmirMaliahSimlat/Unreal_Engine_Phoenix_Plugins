#pragma once

#include "Modules/ModuleManager.h"

class FTreePlacerModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
