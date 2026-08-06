#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "BuildingFootprintFilterSettings.generated.h"

/**
 * Rules for deciding which mesh components participate in footprint extraction.
 * By default everything passes (current maps are building-only).
 * Tighten these later if tiles start including non-building props.
 */
UCLASS(BlueprintType)
class BUILDINGFOOTPRINTEXPORTER_API UBuildingFootprintFilterSettings : public UDataAsset
{
	GENERATED_BODY()

public:
	/** If non-empty, actor must have at least one of these tags. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Actor Filter")
	TArray<FName> IncludeActorTags;

	/** Actors with any of these tags are skipped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Actor Filter")
	TArray<FName> ExcludeActorTags;

	/** If non-empty, component must have at least one of these tags. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Component Filter")
	TArray<FName> IncludeComponentTags;

	/** Components with any of these tags are skipped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Component Filter")
	TArray<FName> ExcludeComponentTags;

	/**
	 * Case-insensitive substring match against component or static mesh name.
	 * Empty = allow all names.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Component Filter")
	TArray<FString> IncludeNameContains;

	/** Case-insensitive substring match; matching components are skipped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Component Filter")
	TArray<FString> ExcludeNameContains;

	/** Skip components whose world-space 2D bounds area is below this (m^2). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Size Filter", meta = (ClampMin = "0.0"))
	double MinComponentBoundsAreaM2 = 0.0;

	/**
	 * If non-empty, component must use at least one material whose name contains one of these
	 * substrings (case-insensitive). Example: "roof".
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Material Filter")
	TArray<FString> IncludeMaterialNameContains;

	/** Skip components that use any material whose name contains one of these substrings. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Material Filter")
	TArray<FString> ExcludeMaterialNameContains;
};
