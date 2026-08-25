using UnrealBuildTool;

public class BuildingExtruderEditor : ModuleRules
{
	public BuildingExtruderEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bUseUnity = false;
		// Same include order on 5.1 and 5.3 so one codebase compiles in both editors.
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_1;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"BuildingExtruder"
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
			"Projects",
			"Foliage",
			"CesiumRuntime",
			"Json",
			"JsonUtilities"
		});
	}
}
