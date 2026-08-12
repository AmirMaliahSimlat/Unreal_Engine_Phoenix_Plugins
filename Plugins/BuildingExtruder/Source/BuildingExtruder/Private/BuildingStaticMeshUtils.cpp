#include "BuildingStaticMeshUtils.h"
#include "BuildingExtruderLog.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/StaticMesh.h"
#include "Materials/Material.h"
#include "MeshDescription.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
#include "StaticMeshAttributes.h"
#include "StaticMeshOperations.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"

namespace
{
	FString SanitizeAssetName(const FString& InName)
	{
		FString Out;
		Out.Reserve(InName.Len());
		for (const TCHAR C : InName)
		{
			if (FChar::IsAlnum(C) || C == TEXT('_'))
			{
				Out.AppendChar(C);
			}
			else if (C == TEXT(' ') || C == TEXT('-') || C == TEXT('.'))
			{
				Out.AppendChar(TEXT('_'));
			}
		}
		if (Out.IsEmpty())
		{
			Out = TEXT("BuildingMesh");
		}
		if (FChar::IsDigit(Out[0]))
		{
			Out = TEXT("M_") + Out;
		}
		return Out;
	}

	bool SaveAssetPackage(UPackage* Package, UObject* Asset, FString& OutError)
	{
		if (!Package || !Asset)
		{
			OutError = TEXT("SaveAssetPackage: null package or asset.");
			return false;
		}

		Package->MarkPackageDirty();

		const FString PackageFilename = FPackageName::LongPackageNameToFilename(
			Package->GetName(),
			FPackageName::GetAssetPackageExtension());

		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.Error = GError;
		SaveArgs.bForceByteSwapping = false;
		SaveArgs.bWarnOfLongFilename = true;
		SaveArgs.SaveFlags = SAVE_NoError;

		if (!UPackage::SavePackage(Package, Asset, *PackageFilename, SaveArgs))
		{
			OutError = FString::Printf(
				TEXT("Failed to save package '%s' to '%s'."),
				*Package->GetName(),
				*PackageFilename);
			return false;
		}
		return true;
	}

	void FillMeshDescription(
		const FExtrudedPrismMesh& Mesh,
		int32 NumMaterialSlots,
		FMeshDescription& MeshDescription,
		FString& OutError)
	{
		FStaticMeshAttributes Attributes(MeshDescription);
		Attributes.Register();
		Attributes.GetVertexInstanceUVs().SetNumChannels(1);

		TVertexAttributesRef<FVector3f> VertexPositions = Attributes.GetVertexPositions();
		TVertexInstanceAttributesRef<FVector3f> InstanceNormals = Attributes.GetVertexInstanceNormals();
		TVertexInstanceAttributesRef<FVector2f> InstanceUVs = Attributes.GetVertexInstanceUVs();

		TArray<FVertexID> VertexIds;
		VertexIds.Reserve(Mesh.Vertices.Num());
		for (int32 I = 0; I < Mesh.Vertices.Num(); ++I)
		{
			const FVertexID Vid = MeshDescription.CreateVertex();
			VertexPositions[Vid] = FVector3f(Mesh.Vertices[I]);
			VertexIds.Add(Vid);
		}

		const int32 SlotCount = FMath::Max(NumMaterialSlots, 1);
		TArray<FPolygonGroupID> Groups;
		Groups.Reserve(SlotCount);
		for (int32 S = 0; S < SlotCount; ++S)
		{
			Groups.Add(MeshDescription.CreatePolygonGroup());
		}

		const int32 NumTris = Mesh.Triangles.Num() / 3;
		for (int32 T = 0; T < NumTris; ++T)
		{
			const int32 I0 = Mesh.Triangles[T * 3 + 0];
			const int32 I1 = Mesh.Triangles[T * 3 + 1];
			const int32 I2 = Mesh.Triangles[T * 3 + 2];
			if (!VertexIds.IsValidIndex(I0) || !VertexIds.IsValidIndex(I1) || !VertexIds.IsValidIndex(I2))
			{
				OutError = TEXT("Triangle index out of range while building StaticMesh.");
				return;
			}

			int32 Slot = 0;
			if (Mesh.TriangleMaterialIndices.IsValidIndex(T))
			{
				Slot = Mesh.TriangleMaterialIndices[T];
			}
			Slot = FMath::Clamp(Slot, 0, SlotCount - 1);

			// Keep AddTri winding (already oriented to DesiredOutwardDir). Normals are
			// recomputed from faces below — same idea as Maya "Mesh Display → Set to Face".
			TArray<FVertexInstanceID, TInlineAllocator<3>> CornerIds;
			const int32 Indices[3] = { I0, I1, I2 };
			for (int32 C = 0; C < 3; ++C)
			{
				const int32 Vi = Indices[C];
				const FVertexInstanceID InstanceId = MeshDescription.CreateVertexInstance(VertexIds[Vi]);
				InstanceNormals[InstanceId] = FVector3f::ZeroVector;
				InstanceUVs.Set(InstanceId, 0, Mesh.UVs.IsValidIndex(Vi)
					? FVector2f(Mesh.UVs[Vi])
					: FVector2f::ZeroVector);
				CornerIds.Add(InstanceId);
			}

			MeshDescription.CreatePolygon(Groups[Slot], CornerIds);
		}

		// Extrude math "outward" was rendering inside-out in UE; flip faces once, then
		// rebuild hard face normals from the final winding (Maya Set to Face equivalent).
		MeshDescription.ReverseAllPolygonFacing();

		TEdgeAttributesRef<bool> EdgeHardnesses = Attributes.GetEdgeHardnesses();
		for (const FEdgeID EdgeId : MeshDescription.Edges().GetElementIDs())
		{
			EdgeHardnesses[EdgeId] = true;
		}

		FStaticMeshOperations::ComputeTangentsAndNormals(
			MeshDescription,
			EComputeNTBsFlags::Normals | EComputeNTBsFlags::Tangents);
	}
}

UMaterialInterface* BuildingStaticMeshUtils::GetOrCreateBuildingMaterial(FString& OutError)
{
	OutError.Reset();
	constexpr TCHAR MaterialPackagePath[] = TEXT("/Game/BuildingExtruder/Materials/M_BuildingExtruder_Default");
	constexpr TCHAR MaterialAssetName[] = TEXT("M_BuildingExtruder_Default");

	if (UMaterial* Existing = LoadObject<UMaterial>(nullptr, *(FString(MaterialPackagePath) + TEXT(".") + MaterialAssetName)))
	{
		if (Existing->TwoSided)
		{
			Existing->Modify();
			Existing->TwoSided = false;
			Existing->PostEditChange();
			SaveAssetPackage(Existing->GetOutermost(), Existing, OutError);
			OutError.Reset();
		}
		return Existing;
	}

	UPackage* Package = CreatePackage(MaterialPackagePath);
	if (!Package)
	{
		OutError = TEXT("Failed to create material package.");
		return nullptr;
	}
	Package->FullyLoad();

	UMaterial* Mat = NewObject<UMaterial>(
		Package,
		MaterialAssetName,
		RF_Public | RF_Standalone | RF_Transactional);
	if (!Mat)
	{
		OutError = TEXT("Failed to allocate building material.");
		return nullptr;
	}

	// One-sided: meshes now have outward normals. Two-sided caused dark double-shadowing.
	Mat->TwoSided = false;
	Mat->BlendMode = BLEND_Opaque;
	Mat->MaterialDomain = MD_Surface;
	Mat->PostEditChange();

	FAssetRegistryModule::AssetCreated(Mat);
	if (!SaveAssetPackage(Package, Mat, OutError))
	{
		return nullptr;
	}

	UE_LOG(LogBuildingExtruder, Display, TEXT("Created building material asset: %s"), MaterialPackagePath);
	return Mat;
}

UStaticMesh* BuildingStaticMeshUtils::CreatePersistentStaticMesh(
	const FString& PackagePath,
	const FString& AssetName,
	const FExtrudedPrismMesh& Mesh,
	UMaterialInterface* Material,
	int32 NumMaterialSlots,
	FString& OutError)
{
	OutError.Reset();
	if (Mesh.Vertices.Num() < 3 || Mesh.Triangles.Num() < 3 || (Mesh.Triangles.Num() % 3) != 0)
	{
		OutError = TEXT("Invalid mesh for StaticMesh build.");
		return nullptr;
	}

	const int32 SlotCount = FMath::Max(NumMaterialSlots, 1);
	const FString SafeName = SanitizeAssetName(AssetName);
	FString Folder = PackagePath.TrimStartAndEnd();
	Folder.RemoveFromEnd(TEXT("/"));
	if (Folder.IsEmpty())
	{
		Folder = TEXT("/Game/BuildingExtruder/Meshes");
	}
	else if (!Folder.StartsWith(TEXT("/")))
	{
		Folder = TEXT("/") + Folder;
	}

	FString SafePackagePath;
	if (Folder.EndsWith(SafeName) || Folder.EndsWith(AssetName))
	{
		SafePackagePath = Folder.EndsWith(SafeName) ? Folder : (FPaths::GetPath(Folder) / SafeName);
	}
	else
	{
		SafePackagePath = Folder / SafeName;
	}

	UPackage* Package = CreatePackage(*SafePackagePath);
	if (!Package)
	{
		OutError = FString::Printf(TEXT("Failed to create package '%s'."), *SafePackagePath);
		return nullptr;
	}
	Package->FullyLoad();

	UStaticMesh* StaticMesh = FindObject<UStaticMesh>(Package, *SafeName);
	const bool bCreatedNew = (StaticMesh == nullptr);
	if (!StaticMesh)
	{
		StaticMesh = NewObject<UStaticMesh>(
			Package,
			*SafeName,
			RF_Public | RF_Standalone | RF_Transactional);
	}
	if (!StaticMesh)
	{
		OutError = TEXT("Failed to allocate UStaticMesh.");
		return nullptr;
	}

	FMeshDescription MeshDescription;
	FillMeshDescription(Mesh, SlotCount, MeshDescription, OutError);
	if (!OutError.IsEmpty())
	{
		return nullptr;
	}

	StaticMesh->Modify();
	StaticMesh->GetStaticMaterials().Reset();
	UMaterialInterface* Mat = Material;
	if (!Mat)
	{
		Mat = GetOrCreateBuildingMaterial(OutError);
		if (!Mat)
		{
			return nullptr;
		}
	}
	for (int32 S = 0; S < SlotCount; ++S)
	{
		StaticMesh->GetStaticMaterials().Add(FStaticMaterial(Mat));
	}

	UStaticMesh::FBuildMeshDescriptionsParams BuildParams;
	BuildParams.bBuildSimpleCollision = false;
#if ENGINE_MAJOR_VERSION == 5
	// Fast build can skip/simplify NTBs; we need real face normals like Maya Set to Face.
	BuildParams.bFastBuild = false;
#endif

	TArray<const FMeshDescription*> Descriptions;
	Descriptions.Add(&MeshDescription);
	StaticMesh->BuildFromMeshDescriptions(Descriptions, BuildParams);

	if (bCreatedNew)
	{
		FAssetRegistryModule::AssetCreated(StaticMesh);
	}

	if (!SaveAssetPackage(Package, StaticMesh, OutError))
	{
		return nullptr;
	}

	return StaticMesh;
}
