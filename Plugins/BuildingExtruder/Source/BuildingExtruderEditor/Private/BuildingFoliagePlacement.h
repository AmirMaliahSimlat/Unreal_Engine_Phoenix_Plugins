#pragma once

#include "CoreMinimal.h"

class AInstancedFoliageActor;
class UFoliageType;
class UStaticMesh;
struct FFoliageInfo;

struct FBuildingFoliageSlot
{
	UFoliageType* Type = nullptr;
	FFoliageInfo* Info = nullptr;
};

namespace BuildingFoliagePlacement
{
	bool GetOrCreateSlot(
		AInstancedFoliageActor& IFA,
		UStaticMesh* Mesh,
		FBuildingFoliageSlot& OutSlot,
		FString& OutError);

	bool AddInstances(FBuildingFoliageSlot& Slot, const TArray<FTransform>& WorldTransforms);

	void RefreshSlots(AInstancedFoliageActor& IFA, TArray<FBuildingFoliageSlot>& Slots);
}
