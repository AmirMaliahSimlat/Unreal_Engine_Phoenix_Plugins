#include "BuildingFootprintExporterBPLibrary.h"

#include "BuildingFootprintExportSettings.h"
#include "BuildingFootprintExporterLog.h"
#include "BuildingFootprintFilterSettings.h"
#include "FootprintGeoTransform.h"
#include "FootprintGeometryUtils.h"
#include "ShapefileWriter.h"

#include "Editor.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "UObject/Package.h"

namespace
{
	void CopyMapInputToSettings(const FBuildingFootprintMapInput& MapInput, UBuildingFootprintExportSettings& Settings)
	{
		Settings.MapName = MapInput.MapName;
		Settings.OriginLatitude = MapInput.OriginLatitude;
		Settings.OriginLongitude = MapInput.OriginLongitude;
		Settings.ClusterMergeDistanceCm = MapInput.ClusterMergeDistanceCm;
		Settings.IncludeMaterialNameContains = MapInput.IncludeMaterialNameContains;
		Settings.SilhouetteSimplifyToleranceCm = MapInput.SilhouetteSimplifyToleranceCm;
		Settings.SilhouetteMaxGridDimension = MapInput.SilhouetteMaxGridDimension;
	}
}

FBuildingFootprintExportResult UBuildingFootprintExporterBPLibrary::ExportBuildingFootprintsWithMapInput(
	const FBuildingFootprintMapInput& MapInput,
	UBuildingFootprintFilterSettings* FilterSettings,
	const FString& OutputPathWithoutExtension)
{
	UBuildingFootprintExportSettings* Settings = MakeExportSettingsFromMapInput(nullptr, MapInput);
	return ExportBuildingFootprints(nullptr, Settings, FilterSettings, OutputPathWithoutExtension);
}

FBuildingFootprintExportResult UBuildingFootprintExporterBPLibrary::ExportBuildingFootprints(
	UObject* WorldContextObject,
	UBuildingFootprintExportSettings* ExportSettings,
	UBuildingFootprintFilterSettings* FilterSettings,
	const FString& OutputPathWithoutExtension)
{
	FBuildingFootprintExportResult Result;

	if (!ExportSettings)
	{
		Result.Message = TEXT("ExportSettings is required. Pass MapInput via ExportBuildingFootprintsWithMapInput, or a project data asset.");
		UE_LOG(LogBuildingFootprintExporter, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	UWorld* World = nullptr;
	if (WorldContextObject)
	{
		World = GEngine ? GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::ReturnNull) : nullptr;
	}
	if (!World && GEditor)
	{
		World = GEditor->GetEditorWorldContext().World();
	}
	if (!World)
	{
		Result.Message = TEXT("Could not resolve an editor world. Open a map first.");
		UE_LOG(LogBuildingFootprintExporter, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	const double TotalStart = FPlatformTime::Seconds();
	UE_LOG(
		LogBuildingFootprintExporter,
		Display,
		TEXT("========== Export START =========="));
	UE_LOG(
		LogBuildingFootprintExporter,
		Display,
		TEXT("map='%s' materialFilter='%s' out='%s' world='%s'"),
		*ExportSettings->MapName,
		*ExportSettings->IncludeMaterialNameContains,
		*OutputPathWithoutExtension,
		*World->GetName());

	const FFootprintGeoTransform Geo = FFootprintGeoTransform::FromSettings(*ExportSettings);
	const FFootprintExtractionResult Extraction = FootprintGeometryUtils::ExtractFootprints(
		World,
		*ExportSettings,
		FilterSettings,
		Geo);

	Result.ActorsScanned = Extraction.ActorsScanned;
	Result.ComponentsAccepted = Extraction.ComponentsAccepted;
	Result.ElapsedSeconds = Extraction.ElapsedSeconds;
	Result.bCancelled = Extraction.bCancelled;

	if (Extraction.bCancelled)
	{
		if (Extraction.Footprints.Num() > 0)
		{
			FString WriteError;
			if (ShapefileWriter::WritePolygons(OutputPathWithoutExtension, Extraction.Footprints, ExportSettings->MapName, WriteError))
			{
				Result.FootprintCount = Extraction.Footprints.Num();
				Result.Message = FString::Printf(
					TEXT("Cancelled. Wrote partial export: %d footprints to '%s'. Actors scanned: %d, elapsed: %.2fs."),
					Result.FootprintCount,
					*OutputPathWithoutExtension,
					Result.ActorsScanned,
					FPlatformTime::Seconds() - TotalStart);
			}
			else
			{
				Result.Message = FString::Printf(TEXT("Cancelled. %s"), *WriteError);
			}
		}
		else
		{
			Result.Message = Extraction.ErrorMessage;
		}
		Result.ElapsedSeconds = FPlatformTime::Seconds() - TotalStart;
		UE_LOG(LogBuildingFootprintExporter, Warning, TEXT("%s"), *Result.Message);
		UE_LOG(LogBuildingFootprintExporter, Display, TEXT("========== Export END (cancelled) =========="));
		return Result;
	}

	if (!Extraction.ErrorMessage.IsEmpty())
	{
		Result.Message = Extraction.ErrorMessage;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - TotalStart;
		UE_LOG(LogBuildingFootprintExporter, Error, TEXT("%s"), *Result.Message);
		UE_LOG(LogBuildingFootprintExporter, Display, TEXT("========== Export END (error) =========="));
		return Result;
	}

	if (Extraction.Footprints.Num() == 0)
	{
		Result.Message = FString::Printf(
			TEXT("No footprints found. Actors scanned: %d, components accepted: %d, rejected: %d. Elapsed: %.2fs."),
			Extraction.ActorsScanned,
			Extraction.ComponentsAccepted,
			Extraction.ComponentsRejected,
			Result.ElapsedSeconds);
		Result.ElapsedSeconds = FPlatformTime::Seconds() - TotalStart;
		UE_LOG(LogBuildingFootprintExporter, Warning, TEXT("%s"), *Result.Message);
		UE_LOG(LogBuildingFootprintExporter, Display, TEXT("========== Export END (empty) =========="));
		return Result;
	}

	UE_LOG(
		LogBuildingFootprintExporter,
		Display,
		TEXT("Writing shapefile for %d footprints to '%s'..."),
		Extraction.Footprints.Num(),
		*OutputPathWithoutExtension);

	FString WriteError;
	if (!ShapefileWriter::WritePolygons(OutputPathWithoutExtension, Extraction.Footprints, ExportSettings->MapName, WriteError))
	{
		Result.Message = WriteError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - TotalStart;
		UE_LOG(LogBuildingFootprintExporter, Error, TEXT("Shapefile write failed: %s"), *WriteError);
		UE_LOG(LogBuildingFootprintExporter, Display, TEXT("========== Export END (write failed) =========="));
		return Result;
	}

	Result.bSuccess = true;
	Result.FootprintCount = Extraction.Footprints.Num();
	Result.ElapsedSeconds = FPlatformTime::Seconds() - TotalStart;
	Result.Message = FString::Printf(
		TEXT("Exported %d footprints to '%s' (.shp/.shx/.dbf/.prj). Actors: %d, components: %d, elapsed: %.2fs."),
		Result.FootprintCount,
		*OutputPathWithoutExtension,
		Result.ActorsScanned,
		Result.ComponentsAccepted,
		Result.ElapsedSeconds);
	UE_LOG(LogBuildingFootprintExporter, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogBuildingFootprintExporter, Display, TEXT("========== Export END (success) =========="));
	return Result;
}

FBuildingFootprintExportResult UBuildingFootprintExporterBPLibrary::ExportBuildingFootprintsFromEditorWorld(
	UBuildingFootprintExportSettings* ExportSettings,
	UBuildingFootprintFilterSettings* FilterSettings,
	const FString& OutputPathWithoutExtension)
{
	return ExportBuildingFootprints(nullptr, ExportSettings, FilterSettings, OutputPathWithoutExtension);
}

UBuildingFootprintExportSettings* UBuildingFootprintExporterBPLibrary::MakeExportSettingsFromMapInput(
	UObject* WorldContextObject,
	const FBuildingFootprintMapInput& MapInput)
{
	UObject* Outer = GetTransientPackage();
	if (WorldContextObject)
	{
		if (UWorld* World = WorldContextObject->GetWorld())
		{
			Outer = World;
		}
	}

	UBuildingFootprintExportSettings* Settings = NewObject<UBuildingFootprintExportSettings>(Outer);
	CopyMapInputToSettings(MapInput, *Settings);
	return Settings;
}
