#include "BuildingFoliagePlacement.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
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

	void AppendComponentTags(UActorComponent& Comp, const TArray<FName>& Tags)
	{
		for (const FName& Tag : Tags)
		{
			Comp.ComponentTags.AddUnique(Tag);
		}
	}

	USceneComponent* FindOrMakeIFaFolder(
		AInstancedFoliageActor& IFA,
		FName FolderName,
		const TArray<FName>& Tags)
	{
		USceneComponent* Root = IFA.GetRootComponent();
		if (!Root)
		{
			return nullptr;
		}

		TArray<USceneComponent*> Comps;
		IFA.GetComponents(Comps);
		for (USceneComponent* Comp : Comps)
		{
			if (!Comp || Comp == Root || Comp->GetClass() != USceneComponent::StaticClass())
			{
				continue;
			}
			if (Comp->GetFName() == FolderName || Comp->ComponentTags.Contains(FolderName))
			{
				AppendComponentTags(*Comp, Tags);
				if (Comp->GetAttachParent() != Root)
				{
					Comp->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
				}
				Comp->SetRelativeTransform(FTransform::Identity);
				return Comp;
			}
		}

		USceneComponent* Folder = NewObject<USceneComponent>(&IFA, FolderName, RF_Transactional);
		if (!Folder)
		{
			return nullptr;
		}
		Folder->SetMobility(EComponentMobility::Static);
		Folder->SetupAttachment(Root);
		Folder->SetRelativeTransform(FTransform::Identity);
		IFA.AddInstanceComponent(Folder);
		Folder->RegisterComponent();
		AppendComponentTags(*Folder, Tags);
		return Folder;
	}

	void OrganizeFoliageHism(
		AInstancedFoliageActor& IFA,
		UFoliageType* Type,
		FFoliageInfo* Info,
		FName FolderName,
		const TArray<FName>& Tags)
	{
		if (!Info)
		{
			return;
		}

		UHierarchicalInstancedStaticMeshComponent* HISM =
			Cast<UHierarchicalInstancedStaticMeshComponent>(Info->GetComponent());
		if (!HISM)
		{
			return;
		}

		const FName TreeTag(TEXT("Tree"));
		const FName RoofObjectTag(TEXT("RoofObject"));
		if (FolderName == TreeTag && HISM->ComponentTags.Contains(RoofObjectTag))
		{
			return;
		}
		if (FolderName == RoofObjectTag && HISM->ComponentTags.Contains(TreeTag))
		{
			return;
		}

		USceneComponent* Folder = FindOrMakeIFaFolder(IFA, FolderName, Tags);
		if (!Folder)
		{
			return;
		}

		if (HISM->GetAttachParent() != Folder)
		{
			HISM->AttachToComponent(Folder, FAttachmentTransformRules::KeepWorldTransform);
		}
		if (USceneComponent* Root = IFA.GetRootComponent())
		{
			HISM->SetWorldTransform(Root->GetComponentTransform());
		}
		AppendComponentTags(*HISM, Tags);

		FString MeshName;
		if (const UFoliageType_InstancedStaticMesh* ISMType = Cast<UFoliageType_InstancedStaticMesh>(Type))
		{
			if (const UStaticMesh* Mesh = ISMType->GetStaticMesh())
			{
				MeshName = Mesh->GetName();
			}
		}
		if (MeshName.IsEmpty() && HISM->GetStaticMesh())
		{
			MeshName = HISM->GetStaticMesh()->GetName();
		}
		if (!MeshName.IsEmpty() && !HISM->GetName().StartsWith(MeshName))
		{
			const FName UniqueName = MakeUniqueObjectName(&IFA, HISM->GetClass(), FName(*MeshName));
			HISM->Rename(*UniqueName.ToString(), nullptr, REN_DontCreateRedirectors);
		}
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
	OrganizeFoliageHism(
		IFA,
		Type,
		Info,
		FName(TEXT("RoofObject")),
		{FName(TEXT("RoofObject"))});
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

void BuildingFoliagePlacement::RefreshSlots(AInstancedFoliageActor& IFA, TArray<FBuildingFoliageSlot>& Slots)
{
	for (FBuildingFoliageSlot& Slot : Slots)
	{
		if (!Slot.Info)
		{
			continue;
		}
		Slot.Info->Refresh(/*Async*/ false, /*Force*/ true);
		ConfigureType(Slot.Type, Slot.Info);
		OrganizeFoliageHism(
			IFA,
			Slot.Type,
			Slot.Info,
			FName(TEXT("RoofObject")),
			{FName(TEXT("RoofObject"))});
	}
}
