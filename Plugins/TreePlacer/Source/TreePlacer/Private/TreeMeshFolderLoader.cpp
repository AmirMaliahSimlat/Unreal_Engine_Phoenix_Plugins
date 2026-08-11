#include "TreeMeshFolderLoader.h"
#include "TreePlacerLog.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/StaticMesh.h"
#include "FoliageType.h"
#include "FoliageType_InstancedStaticMesh.h"
#include "Misc/PackageName.h"

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

		// If an object path is passed, convert to package path.
		if (Folder.Contains(TEXT(".")))
		{
			const FString PackageName = FPackageName::ObjectPathToPackageName(Folder);
			if (!PackageName.IsEmpty())
			{
				Folder = PackageName;
			}
		}

		Folder.RemoveFromEnd(TEXT("/"));
		if (!Folder.StartsWith(TEXT("/")))
		{
			Folder = TEXT("/") + Folder;
		}
		return Folder;
	}
}

bool TreeMeshFolderLoader::LoadTreeMeshesFromFolder(
	const FString& ContentFolderPath,
	TArray<UStaticMesh*>& OutMeshes,
	FString& OutError)
{
	OutMeshes.Reset();
	OutError.Reset();

	FString Folder = NormalizeContentFolderPath(ContentFolderPath);
	if (Folder.IsEmpty())
	{
		OutError = TEXT("Tree mesh folder path is empty. Use a Content path like /Game/Trees.");
		return false;
	}
	if (!Folder.StartsWith(TEXT("/Game")) && !Folder.StartsWith(TEXT("/Engine")))
	{
		OutError = FString::Printf(
			TEXT("Tree mesh folder must be a Content path under /Game or /Engine (e.g. /Game/Trees), got '%s'."),
			*Folder);
		return false;
	}

	FAssetRegistryModule& AssetRegistryModule =
		FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry"));
	IAssetRegistry& AssetRegistry = AssetRegistryModule.Get();
	AssetRegistry.SearchAllAssets(true);

	FARFilter Filter;
	Filter.bRecursivePaths = true;
	Filter.PackagePaths.Add(*Folder);
	Filter.bIncludeOnlyOnDiskAssets = false;

#if ENGINE_MAJOR_VERSION >= 5
	Filter.ClassPaths.Add(UStaticMesh::StaticClass()->GetClassPathName());
	Filter.ClassPaths.Add(UFoliageType::StaticClass()->GetClassPathName());
	Filter.ClassPaths.Add(UFoliageType_InstancedStaticMesh::StaticClass()->GetClassPathName());
#else
	Filter.ClassNames.Add(UStaticMesh::StaticClass()->GetFName());
	Filter.ClassNames.Add(UFoliageType::StaticClass()->GetFName());
	Filter.ClassNames.Add(UFoliageType_InstancedStaticMesh::StaticClass()->GetFName());
#endif

	TArray<FAssetData> Assets;
	AssetRegistry.GetAssets(Filter, Assets);
	if (Assets.Num() == 0)
	{
		// Fallback without class filter in case foliage class path differs.
		FARFilter FolderOnly;
		FolderOnly.bRecursivePaths = true;
		FolderOnly.PackagePaths.Add(*Folder);
		AssetRegistry.GetAssets(FolderOnly, Assets);
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

		// UE 5.1: GetStaticMesh / Mesh live on InstancedStaticMesh foliage types, not UFoliageType base.
		if (UFoliageType_InstancedStaticMesh* FoliageISM = Cast<UFoliageType_InstancedStaticMesh>(Obj))
		{
			if (UStaticMesh* Mesh = FoliageISM->GetStaticMesh())
			{
				UniqueMeshes.Add(Mesh);
			}
			else if (FoliageISM->Mesh)
			{
				UniqueMeshes.Add(FoliageISM->Mesh);
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
			TEXT("No StaticMesh or FoliageType_InstancedStaticMesh assets with a mesh found under '%s'. "
				 "Assets scanned: %d%s%s"),
			*Folder,
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
		*Folder);
	return true;
}
