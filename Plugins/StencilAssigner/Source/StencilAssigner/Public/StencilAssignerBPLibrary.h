#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "StencilAssignerBPLibrary.generated.h"

USTRUCT(BlueprintType)
struct FStencilAssignResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Stencil Assigner")
	bool bSuccess = false;

	UPROPERTY(BlueprintReadOnly, Category = "Stencil Assigner")
	int32 ComponentsMatched = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Stencil Assigner")
	int32 ComponentsUnmatched = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Stencil Assigner")
	FString UnmatchedReportPath;

	UPROPERTY(BlueprintReadOnly, Category = "Stencil Assigner")
	double ElapsedSeconds = 0.0;

	UPROPERTY(BlueprintReadOnly, Category = "Stencil Assigner")
	FString Message;
};

/**
 * Blueprint API for assigning Custom Depth stencil from component tags.
 * Recreate this Blueprint node after updating.
 */
UCLASS()
class STENCILASSIGNER_API UStencilAssignerBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Walks every primitive component in the editor map. The first mapping whose Tag is on
	 * the component wins: Custom Depth is enabled and StencilValue is written (0–255).
	 * Actor tags are ignored (HISMs on InstancedFoliageActor need component tags).
	 * Unmatched primitives are printed to the Output Log and written to UnmatchedReportPath.
	 *
	 * @param Mappings Pasteable list: Tag=Value pairs separated by commas or newlines.
	 *        Example: Wall=1,Roof=2,Tree=3,RoofObject=4  First match wins.
	 * @param UnmatchedReportPath Path to a .txt report of unmatched components (and their actors).
	 */
	UFUNCTION(
		BlueprintCallable,
		Category = "Stencil Assigner",
		meta = (
			WorldContext = "WorldContextObject",
			CPP_Default_Mappings = "Wall=1,Roof=2,Tree=3,RoofObject=4"))
	static FStencilAssignResult ApplyCustomStencilByTags(
		UObject* WorldContextObject,
		const FString& Mappings,
		const FString& UnmatchedReportPath);
};
