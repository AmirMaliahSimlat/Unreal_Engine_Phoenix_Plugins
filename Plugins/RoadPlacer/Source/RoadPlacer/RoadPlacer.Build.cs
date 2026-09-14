using UnrealBuildTool;

public class RoadPlacer : ModuleRules
{
	public RoadPlacer(ReadOnlyTargetRules Target) : base(Target)
	{
		// Private PCH so UnrealEd's C++20 shared PCH cannot compile <ppltasks.h>
		// before std::result_of is restored.
		PCHUsage = PCHUsageMode.NoSharedPCHs;
		PrivatePCHHeaderFile = "Private/RoadPlacerPrivatePCH.h";
		bUseUnity = false;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_1;
		// Cesium cartographic/overlay headers use std::span (C++20). Matching CesiumRuntime.
		CppStandard = CppStandardVersion.Cpp20;
		bEnableExceptions = true;

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
