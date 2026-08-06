#include "BuildingFbxExporter.h"
#include "BuildingExtruderLog.h"

#include "AssetExportTask.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Exporters/Exporter.h"
#include "Exporters/FbxExportOption.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Editor.h"
#include "Engine/Selection.h"

namespace
{
	FString NormalizeFbxPath(const FString& InPath)
	{
		FString Path = InPath.TrimStartAndEnd();
		if (Path.IsEmpty())
		{
			return Path;
		}
		if (!FPaths::GetExtension(Path).Equals(TEXT("fbx"), ESearchCase::IgnoreCase))
		{
			Path += TEXT(".fbx");
		}
		return Path;
	}
}

bool BuildingFbxExporter::ExportTileActors(
	UWorld& World,
	const TArray<AStaticMeshActor*>& TileActors,
	const FString& OutputPath,
	FString& OutError)
{
	const FString FbxPath = NormalizeFbxPath(OutputPath);
	if (FbxPath.IsEmpty())
	{
		OutError = TEXT("FBX output path is empty.");
		return false;
	}
	if (TileActors.Num() == 0)
	{
		OutError = TEXT("No tile actors to export to FBX.");
		return false;
	}
	if (!GEditor)
	{
		OutError = TEXT("GEditor is null; cannot run native FBX export.");
		return false;
	}

	const FString Directory = FPaths::GetPath(FbxPath);
	if (!Directory.IsEmpty() && !IFileManager::Get().MakeDirectory(*Directory, true))
	{
		OutError = FString::Printf(TEXT("Failed to create FBX directory: %s"), *Directory);
		return false;
	}

	// Remember selection, then select only our tile actors (level FBX export uses selection).
	TArray<AActor*> PreviouslySelected;
	if (USelection* Selection = GEditor->GetSelectedActors())
	{
		Selection->GetSelectedObjects<AActor>(PreviouslySelected);
	}

	GEditor->SelectNone(/*bNoteSelectionChange*/ false, /*bDeselectBSPSurfs*/ true);
	int32 SelectedCount = 0;
	for (AStaticMeshActor* Actor : TileActors)
	{
		if (IsValid(Actor))
		{
			GEditor->SelectActor(Actor, /*bInSelected*/ true, /*bNotify*/ false, /*bSelectEvenIfHidden*/ true);
			++SelectedCount;
		}
	}

	if (SelectedCount == 0)
	{
		OutError = TEXT("All tile actors were invalid; nothing to export.");
		return false;
	}

	UFbxExportOption* Options = NewObject<UFbxExportOption>();
	Options->bASCII = false;
	Options->LevelOfDetail = false;
	Options->Collision = false;
	Options->VertexColor = false;

	UAssetExportTask* Task = NewObject<UAssetExportTask>();
	Task->Object = &World;
	Task->Filename = FbxPath;
	Task->bSelected = true;
	Task->bReplaceIdentical = true;
	Task->bPrompt = false;
	Task->bAutomated = true;
	Task->Options = Options;

	const bool bOk = UExporter::RunAssetExportTask(Task);

	// Restore prior selection.
	GEditor->SelectNone(false, true);
	for (AActor* Actor : PreviouslySelected)
	{
		if (IsValid(Actor))
		{
			GEditor->SelectActor(Actor, true, false, true);
		}
	}
	GEditor->NoteSelectionChange();

	if (!bOk || !FPaths::FileExists(FbxPath))
	{
		OutError = FString::Printf(
			TEXT("Native FBX export failed for %d actors -> '%s'."),
			SelectedCount,
			*FbxPath);
		return false;
	}

	const int64 FileSize = IFileManager::Get().FileSize(*FbxPath);
	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("FBX written via Unreal exporter: %s (%d tile actors, %.1f MB)"),
		*FbxPath,
		SelectedCount,
		static_cast<double>(FileSize) / (1024.0 * 1024.0));
	return true;
}
