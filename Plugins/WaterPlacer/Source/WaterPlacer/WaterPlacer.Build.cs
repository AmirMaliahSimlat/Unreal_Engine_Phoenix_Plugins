using UnrealBuildTool;

public class WaterPlacer : ModuleRules
{
	public WaterPlacer(ReadOnlyTargetRules Target) : base(Target)
	{
		// Private PCH so UnrealEd's C++20 shared PCH cannot compile <ppltasks.h>
		// before std::result_of is restored.
		PCHUsage = PCHUsageMode.NoSharedPCHs;
		PrivatePCHHeaderFile = "Private/WaterPlacerPrivatePCH.h";
		bUseUnity = false;
		// Same include order on 5.1 and 5.3 so one codebase compiles in both editors.
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_1;
		// Cesium tileset/overlay headers use std::span (C++20). Matching CesiumRuntime.
		CppStandard = CppStandardVersion.Cpp20;
		bEnableExceptions = true;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine"
		});

		AddEngineThirdPartyPrivateStaticDependencies(Target, "zlib");
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
