#pragma once

#include "Modules/ModuleManager.h"

class FStencilAssignerModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
