#include "BuildingMeshFolderLoader.h"
#include "BuildingExtruderLog.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/StaticMesh.h"
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
		if (Folder.StartsWith(TEXT("/All/")))
		{
			Folder = Folder.RightChop(4);
		}
		else if (Folder.StartsWith(TEXT("All/")))
		{
			Folder = TEXT("/") + Folder.RightChop(4);
		}
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
			FString MountedRoot = Plugin->GetMountedAssetPath();
			MountedRoot.ReplaceInline(TEXT("\\"), TEXT("/"));
			MountedRoot.RemoveFromEnd(TEXT("/"));
			const FString ContentDirBase = FPaths::GetCleanFilename(Plugin->GetContentDir());
			const bool bMatch =
				PluginKey.Equals(PluginName, ESearchCase::IgnoreCase)
				|| PluginKey.Equals(PluginName + TEXT("Content"), ESearchCase::IgnoreCase)
				|| PluginKey.Equals(ContentDirBase, ESearchCase::IgnoreCase)
				|| PluginKey.Equals(MountedRoot.RightChop(1), ESearchCase::IgnoreCase);
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
			return Resolved;
		}
		return Folder;
	}
}

bool BuildingMeshFolderLoader::LoadStaticMeshesFromFolder(
	const FString& ContentFolderPath,
	TArray<UStaticMesh*>& OutMeshes,
	FString& OutError)
{
	OutMeshes.Reset();
	OutError.Reset();

	const FString Normalized = NormalizeContentFolderPath(ContentFolderPath);
	if (Normalized.IsEmpty() || Normalized == TEXT("/"))
	{
		OutError = TEXT("Roof object mesh folder is empty. Use a Content path like /Game/RoofProps.");
		return false;
	}

	const FString Resolved = ResolvePluginMountedFolder(Normalized);
	FAssetRegistryModule& AssetRegistryModule =
		FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
	IAssetRegistry& AssetRegistry = AssetRegistryModule.Get();
	AssetRegistry.SearchAllAssets(true);

	TArray<FString> Candidates;
	Candidates.Add(Resolved);
	if (!Normalized.Equals(Resolved, ESearchCase::IgnoreCase))
	{
		Candidates.Add(Normalized);
	}

	TArray<FAssetData> Assets;
	FString UsedFolder;
	for (const FString& Candidate : Candidates)
	{
		FARFilter Filter;
		Filter.bRecursivePaths = true;
		Filter.PackagePaths.Add(*Candidate);
		Filter.bIncludeOnlyOnDiskAssets = false;
		Filter.ClassPaths.Add(UStaticMesh::StaticClass()->GetClassPathName());
		Assets.Reset();
		AssetRegistry.GetAssets(Filter, Assets);
		if (Assets.Num() > 0)
		{
			UsedFolder = Candidate;
			break;
		}
	}

	TSet<UStaticMesh*> UniqueMeshes;
	for (const FAssetData& AssetData : Assets)
	{
		if (UStaticMesh* Mesh = Cast<UStaticMesh>(AssetData.GetAsset()))
		{
			UniqueMeshes.Add(Mesh);
		}
	}
	OutMeshes = UniqueMeshes.Array();
	OutMeshes.Sort([](const UStaticMesh& A, const UStaticMesh& B)
	{
		return A.GetPathName() < B.GetPathName();
	});

	if (OutMeshes.Num() == 0)
	{
		OutError = FString::Printf(
			TEXT("No StaticMesh assets found under '%s'."),
			*(UsedFolder.IsEmpty() ? Resolved : UsedFolder));
		return false;
	}

	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("Loaded %d roof object meshes from '%s'"),
		OutMeshes.Num(),
		*(UsedFolder.IsEmpty() ? Resolved : UsedFolder));
	return true;
}
