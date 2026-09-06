using UnrealBuildTool;

public class RoadPlacer : ModuleRules
{
	public RoadPlacer(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bUseUnity = false;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_1;

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
			"MeshDescription",
			"StaticMeshDescription",
			"AssetRegistry",
			"CesiumRuntime",
			"Json",
			"JsonUtilities"
		});
	}
}
