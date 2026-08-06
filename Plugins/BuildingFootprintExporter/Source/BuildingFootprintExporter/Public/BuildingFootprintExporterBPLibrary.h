#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "BuildingFootprintExportSettings.h"
#include "BuildingFootprintExporterBPLibrary.generated.h"

class UBuildingFootprintExportSettings;
class UBuildingFootprintFilterSettings;

USTRUCT(BlueprintType)
struct FBuildingFootprintExportResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Building Footprint")
	bool bSuccess = false;

	UPROPERTY(BlueprintReadOnly, Category = "Building Footprint")
	int32 FootprintCount = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Building Footprint")
	int32 ActorsScanned = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Building Footprint")
	int32 ComponentsAccepted = 0;

	/** Wall-clock seconds for extraction + shapefile write. */
	UPROPERTY(BlueprintReadOnly, Category = "Building Footprint")
	double ElapsedSeconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Building Footprint")
	bool bCancelled = false;

	UPROPERTY(BlueprintReadOnly, Category = "Building Footprint")
	FString Message;
};

/**
 * Per-map inputs the user provides when running the tool.
 * Nothing map-specific is hardcoded in the plugin.
 */
USTRUCT(BlueprintType)
struct FBuildingFootprintMapInput
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Map")
	FString MapName = TEXT("UnnamedMap");

	/** Latitude of Unreal (0,0,0) in degrees WGS84. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Georeference")
	double OriginLatitude = 0.0;

	/** Longitude of Unreal (0,0,0) in degrees WGS84. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Georeference")
	double OriginLongitude = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Extraction", meta = (ClampMin = "0.0"))
	double ClusterMergeDistanceCm = 50.0;

	/**
	 * If non-empty, only components with a material name containing this substring are used.
	 * Example: "roof"
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Extraction")
	FString IncludeMaterialNameContains;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Silhouette", meta = (ClampMin = "0.0"))
	double SilhouetteSimplifyToleranceCm = 30.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Silhouette", meta = (ClampMin = "64"))
	int32 SilhouetteMaxGridDimension = 8192;
};

/**
 * Blueprint API for the Building Footprint Exporter editor plugin.
 * Map georeference is always supplied by the user (struct pins or a project data asset).
 */
UCLASS()
class BUILDINGFOOTPRINTEXPORTER_API UBuildingFootprintExporterBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Preferred entry: pass map georeference as Blueprint inputs, export the open editor map.
	 * OutputPathWithoutExtension example: "D:/Exports/Balata_Buildings"
	 */
	UFUNCTION(BlueprintCallable, Category = "Building Footprint Exporter")
	static FBuildingFootprintExportResult ExportBuildingFootprintsWithMapInput(
		const FBuildingFootprintMapInput& MapInput,
		UBuildingFootprintFilterSettings* FilterSettings,
		const FString& OutputPathWithoutExtension);

	/**
	 * Same as above, but uses an Export Settings data asset from your game project Content
	 * (create one asset per map there — not inside this plugin).
	 */
	UFUNCTION(BlueprintCallable, Category = "Building Footprint Exporter", meta = (WorldContext = "WorldContextObject"))
	static FBuildingFootprintExportResult ExportBuildingFootprints(
		UObject* WorldContextObject,
		UBuildingFootprintExportSettings* ExportSettings,
		UBuildingFootprintFilterSettings* FilterSettings,
		const FString& OutputPathWithoutExtension);

	UFUNCTION(BlueprintCallable, Category = "Building Footprint Exporter")
	static FBuildingFootprintExportResult ExportBuildingFootprintsFromEditorWorld(
		UBuildingFootprintExportSettings* ExportSettings,
		UBuildingFootprintFilterSettings* FilterSettings,
		const FString& OutputPathWithoutExtension);

	/** Builds a transient settings object from user-provided map input (optional helper). */
	UFUNCTION(BlueprintCallable, Category = "Building Footprint Exporter", meta = (WorldContext = "WorldContextObject"))
	static UBuildingFootprintExportSettings* MakeExportSettingsFromMapInput(
		UObject* WorldContextObject,
		const FBuildingFootprintMapInput& MapInput);
};
