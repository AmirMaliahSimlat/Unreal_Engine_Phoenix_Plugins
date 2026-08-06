using UnrealBuildTool;

public class BuildingFootprintExporter : ModuleRules
{
	public BuildingFootprintExporter(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bUseUnity = false;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine"
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"UnrealEd",
			"EditorSubsystem",
			"Slate",
			"SlateCore",
			"RenderCore",
			"RHI",
			"MeshDescription",
			"StaticMeshDescription"
		});
	}
}
