#include "BuildingFoliagePlacement.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "FoliageType.h"
#include "FoliageType_InstancedStaticMesh.h"
#include "InstancedFoliage.h"
#include "InstancedFoliageActor.h"

namespace
{
	UFoliageType* FindFoliageTypeForMesh(AInstancedFoliageActor& IFA, const UStaticMesh* Mesh)
	{
		UFoliageType* Found = nullptr;
		IFA.ForEachFoliageInfo([&](UFoliageType* Type, FFoliageInfo& /*Info*/)
		{
			if (!Type || !Mesh)
			{
				return true;
			}
			if (Type->GetSource() == Mesh)
			{
				Found = Type;
				return false;
			}
			if (const UFoliageType_InstancedStaticMesh* ISMType = Cast<UFoliageType_InstancedStaticMesh>(Type))
			{
				if (ISMType->GetStaticMesh() == Mesh)
				{
					Found = Type;
					return false;
				}
			}
			return true;
		});
		return Found;
	}

	void ConfigureType(UFoliageType* Type, FFoliageInfo* Info)
	{
		if (!Type)
		{
			return;
		}
		Type->CullDistance.Min = 0;
		Type->CullDistance.Max = 0;
		Type->CastShadow = true;
		Type->AlignToNormal = false;
		Type->RandomYaw = false;
		Type->Modify();
		if (Info)
		{
			if (UHierarchicalInstancedStaticMeshComponent* HISM =
					Cast<UHierarchicalInstancedStaticMeshComponent>(Info->GetComponent()))
			{
				HISM->InstanceStartCullDistance = 0;
				HISM->InstanceEndCullDistance = 0;
				HISM->SetCastShadow(true);
				HISM->MarkRenderStateDirty();
			}
		}
	}

	FFoliageInstance MakeInstance(const FTransform& Xform)
	{
		FFoliageInstance Instance;
		Instance.Location = Xform.GetLocation();
		Instance.Rotation = Xform.Rotator();
		Instance.DrawScale3D = FVector3f(Xform.GetScale3D());
		return Instance;
	}
}

bool BuildingFoliagePlacement::GetOrCreateSlot(
	AInstancedFoliageActor& IFA,
	UStaticMesh* Mesh,
	FBuildingFoliageSlot& OutSlot,
	FString& OutError)
{
	OutSlot = FBuildingFoliageSlot();
	if (!Mesh)
	{
		OutError = TEXT("Null roof object mesh.");
		return false;
	}

	UFoliageType* Type = FindFoliageTypeForMesh(IFA, Mesh);
	FFoliageInfo* Info = Type ? IFA.FindInfo(Type) : nullptr;
	if (!Type || !Info)
	{
		Type = nullptr;
		Info = IFA.AddMesh(Mesh, &Type);
	}
	if (!Type || !Info)
	{
		OutError = FString::Printf(TEXT("Failed to add foliage type for mesh '%s'."), *Mesh->GetName());
		return false;
	}

	ConfigureType(Type, Info);
	OutSlot.Type = Type;
	OutSlot.Info = Info;
	return true;
}

bool BuildingFoliagePlacement::AddInstances(
	FBuildingFoliageSlot& Slot,
	const TArray<FTransform>& WorldTransforms)
{
	if (!Slot.Type || !Slot.Info || WorldTransforms.Num() == 0)
	{
		return WorldTransforms.Num() == 0;
	}
	for (const FTransform& Xform : WorldTransforms)
	{
		Slot.Info->AddInstance(Slot.Type, MakeInstance(Xform));
	}
	return true;
}

void BuildingFoliagePlacement::RefreshSlots(TArray<FBuildingFoliageSlot>& Slots)
{
	for (FBuildingFoliageSlot& Slot : Slots)
	{
		if (!Slot.Info)
		{
			continue;
		}
		Slot.Info->Refresh(/*Async*/ false, /*Force*/ true);
		ConfigureType(Slot.Type, Slot.Info);
	}
}
