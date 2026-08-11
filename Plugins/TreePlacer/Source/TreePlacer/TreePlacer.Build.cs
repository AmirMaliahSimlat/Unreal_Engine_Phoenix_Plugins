using UnrealBuildTool;

public class TreePlacer : ModuleRules
{
	public TreePlacer(ReadOnlyTargetRules Target) : base(Target)
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
			"AssetRegistry",
			"Foliage",
			"CesiumRuntime",
			"Json",
			"JsonUtilities"
		});
	}
}
