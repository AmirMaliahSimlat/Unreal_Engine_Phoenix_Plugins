#include "TreeMeshFolderLoader.h"
#include "TreePlacerLog.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/StaticMesh.h"
#include "FoliageType.h"
#include "FoliageType_InstancedStaticMesh.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"

namespace
{
	FString NormalizeContentFolderPath(const FString& InPath)
	{
		FString Folder = InPath.TrimStartAndEnd();
		while ((Folder.StartsWith(TEXT("\"")) && Folder.EndsWith(TEXT("\"")) && Folder.Len() >= 2)
			|| (Folder.StartsWith(TEXT("'")) && Folder.EndsWith(TEXT("'")) && Folder.Len() >= 2))
		{
			Folder = Folder.Mid(1, Folder.Len() - 2).TrimStartAndEnd();
		}

		Folder.ReplaceInline(TEXT("\\"), TEXT("/"));

		// Content Browser sometimes prefixes paths with /All.
		if (Folder.StartsWith(TEXT("/All/")))
		{
			Folder = Folder.RightChop(4); // keep leading '/'
		}
		else if (Folder.StartsWith(TEXT("All/")))
		{
			Folder = TEXT("/") + Folder.RightChop(4);
		}

		// If an object path is passed, convert to package path / folder.
		if (Folder.Contains(TEXT(".")))
		{
			const FString PackageName = FPackageName::ObjectPathToPackageName(Folder);
			if (!PackageName.IsEmpty())
			{
				Folder = FPackageName::GetLongPackagePath(PackageName);
			}
		}

		Folder.RemoveFromEnd(TEXT("/"));
		if (!Folder.StartsWith(TEXT("/")))
		{
			Folder = TEXT("/") + Folder;
		}
		return Folder;
	}

	/**
	 * Content Browser often shows plugin assets as /Game/Plugins/<ContentFolder>/...
	 * while the Asset Registry mounts them at /<PluginName>/...
	 * Example:
	 *   /Game/Plugins/PhoenixMapProxyContent/SHP_PLUGIN/trees
	 *     -> /PhoenixMapProxy/SHP_PLUGIN/trees
	 */
	FString ResolvePluginMountedFolder(const FString& Folder)
	{
		const FString Prefix = TEXT("/Game/Plugins/");
		if (!Folder.StartsWith(Prefix))
		{
			return Folder;
		}

		FString Remainder = Folder.Mid(Prefix.Len());
		FString PluginKey;
		FString SubPath;
		if (!Remainder.Split(TEXT("/"), &PluginKey, &SubPath))
		{
			PluginKey = Remainder;
			SubPath.Reset();
		}
		if (PluginKey.IsEmpty())
		{
			return Folder;
		}

		for (const TSharedRef<IPlugin>& Plugin : IPluginManager::Get().GetDiscoveredPlugins())
		{
			if (!Plugin->CanContainContent())
			{
				continue;
			}

			const FString PluginName = Plugin->GetName();
			FString MountedRoot = Plugin->GetMountedAssetPath(); // e.g. "/PhoenixMapProxy/"
			MountedRoot.ReplaceInline(TEXT("\\"), TEXT("/"));
			MountedRoot.RemoveFromEnd(TEXT("/"));

			const FString ContentDirBase = FPaths::GetCleanFilename(Plugin->GetContentDir());
			const bool bMatch =
				PluginKey.Equals(PluginName, ESearchCase::IgnoreCase)
				|| PluginKey.Equals(PluginName + TEXT("Content"), ESearchCase::IgnoreCase)
				|| PluginKey.Equals(ContentDirBase, ESearchCase::IgnoreCase)
				|| PluginKey.Equals(MountedRoot.RightChop(1), ESearchCase::IgnoreCase); // strip leading '/'

			if (!bMatch || MountedRoot.IsEmpty())
			{
				continue;
			}

			FString Resolved = MountedRoot;
			if (!SubPath.IsEmpty())
			{
				Resolved += TEXT("/");
				Resolved += SubPath;
			}

			UE_LOG(
				LogTreePlacer,
				Display,
				TEXT("Resolved plugin content path '%s' -> '%s' (plugin='%s')"),
				*Folder,
				*Resolved,
				*PluginName);
			return Resolved;
		}

		UE_LOG(
			LogTreePlacer,
			Warning,
			TEXT("Could not resolve plugin mount for '%s' (key='%s'). Using path as-is."),
			*Folder,
			*PluginKey);
		return Folder;
	}

	void GatherAssetsUnderFolder(IAssetRegistry& AssetRegistry, const FString& Folder, TArray<FAssetData>& OutAssets)
	{
		FARFilter Filter;
		Filter.bRecursivePaths = true;
		Filter.PackagePaths.Add(*Folder);
		Filter.bIncludeOnlyOnDiskAssets = false;
		// ClassPaths is the 5.1+ filter; ClassNames is deprecated in 5.1 and removed later.
		Filter.ClassPaths.Add(UStaticMesh::StaticClass()->GetClassPathName());
		Filter.ClassPaths.Add(UFoliageType::StaticClass()->GetClassPathName());
		Filter.ClassPaths.Add(UFoliageType_InstancedStaticMesh::StaticClass()->GetClassPathName());

		AssetRegistry.GetAssets(Filter, OutAssets);
		if (OutAssets.Num() == 0)
		{
			FARFilter FolderOnly;
			FolderOnly.bRecursivePaths = true;
			FolderOnly.PackagePaths.Add(*Folder);
			FolderOnly.bIncludeOnlyOnDiskAssets = false;
			AssetRegistry.GetAssets(FolderOnly, OutAssets);
		}
	}
}

bool TreeMeshFolderLoader::LoadTreeMeshesFromFolder(
	const FString& ContentFolderPath,
	TArray<UStaticMesh*>& OutMeshes,
	FString& OutError)
{
	OutMeshes.Reset();
	OutError.Reset();

	const FString Normalized = NormalizeContentFolderPath(ContentFolderPath);
	if (Normalized.IsEmpty() || Normalized == TEXT("/"))
	{
		OutError = TEXT("Tree mesh folder path is empty. Use a Content path like /Game/Trees or /PhoenixMapProxy/SHP_PLUGIN/trees.");
		return false;
	}

	const FString Resolved = ResolvePluginMountedFolder(Normalized);

	TArray<FString> CandidateFolders;
	CandidateFolders.Add(Resolved);
	if (!Normalized.Equals(Resolved, ESearchCase::IgnoreCase))
	{
		CandidateFolders.Add(Normalized);
	}

	FAssetRegistryModule& AssetRegistryModule =
		FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
	IAssetRegistry& AssetRegistry = AssetRegistryModule.Get();
	AssetRegistry.SearchAllAssets(true);

	TArray<FAssetData> Assets;
	FString UsedFolder;
	for (const FString& Candidate : CandidateFolders)
	{
		Assets.Reset();
		GatherAssetsUnderFolder(AssetRegistry, Candidate, Assets);
		UE_LOG(
			LogTreePlacer,
			Display,
			TEXT("Tree mesh folder scan '%s' -> %d assets"),
			*Candidate,
			Assets.Num());
		if (Assets.Num() > 0)
		{
			UsedFolder = Candidate;
			break;
		}
	}

	if (UsedFolder.IsEmpty())
	{
		UsedFolder = Resolved;
	}

	TSet<UStaticMesh*> UniqueMeshes;
	TMap<FName, int32> SeenAssetClasses;
	for (const FAssetData& AssetData : Assets)
	{
		SeenAssetClasses.FindOrAdd(AssetData.AssetClassPath.GetAssetName())++;

		UObject* Obj = AssetData.GetAsset();
		if (!Obj)
		{
			continue;
		}

		if (UStaticMesh* Mesh = Cast<UStaticMesh>(Obj))
		{
			UniqueMeshes.Add(Mesh);
			continue;
		}

		if (UFoliageType_InstancedStaticMesh* FoliageISM = Cast<UFoliageType_InstancedStaticMesh>(Obj))
		{
			if (UStaticMesh* Mesh = FoliageISM->GetStaticMesh())
			{
				UniqueMeshes.Add(Mesh);
			}
		}
	}

	OutMeshes = UniqueMeshes.Array();
	OutMeshes.Sort([](const UStaticMesh& A, const UStaticMesh& B)
	{
		return A.GetPathName() < B.GetPathName();
	});

	if (OutMeshes.Num() == 0)
	{
		FString ClassSummary;
		for (const TPair<FName, int32>& Pair : SeenAssetClasses)
		{
			if (!ClassSummary.IsEmpty())
			{
				ClassSummary += TEXT(", ");
			}
			ClassSummary += FString::Printf(TEXT("%s=%d"), *Pair.Key.ToString(), Pair.Value);
		}
		OutError = FString::Printf(
			TEXT("No StaticMesh or Static Mesh Foliage assets with a mesh found under '%s' "
				 "(also tried plugin-mount resolution from '%s'). Assets scanned: %d%s%s. "
				 "Tip: in Content Browser, right-click the trees folder -> Copy Path, or use "
				 "'/PhoenixMapProxy/SHP_PLUGIN/trees' style mount path."),
			*UsedFolder,
			*Normalized,
			Assets.Num(),
			ClassSummary.IsEmpty() ? TEXT("") : TEXT(" (classes: "),
			ClassSummary.IsEmpty() ? TEXT("") : *(ClassSummary + TEXT(")")));
		return false;
	}

	UE_LOG(
		LogTreePlacer,
		Display,
		TEXT("Loaded %d unique tree meshes from '%s'"),
		OutMeshes.Num(),
		*UsedFolder);
	return true;
}
