using UnrealBuildTool;

public class BuildingExtruder : ModuleRules
{
	public BuildingExtruder(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bUseUnity = false;
		// Same include order on 5.1 and 5.3 so one codebase compiles in both editors.
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_1;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine"
		});
	}
}
