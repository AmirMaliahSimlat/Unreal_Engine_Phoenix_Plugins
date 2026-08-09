#include "BuildingStaticMeshUtils.h"
#include "BuildingExtruderLog.h"

#include "Engine/StaticMesh.h"
#include "Materials/Material.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"

UMaterialInterface* BuildingStaticMeshUtils::GetTwoSidedBuildingMaterial()
{
	static TWeakObjectPtr<UMaterial> CachedMaterial;
	if (CachedMaterial.IsValid())
	{
		return CachedMaterial.Get();
	}

	UMaterial* Mat = NewObject<UMaterial>(
		GetTransientPackage(),
		TEXT("M_BuildingExtruder_TwoSided"),
		RF_Public | RF_Transient);
	Mat->TwoSided = true;
	Mat->BlendMode = BLEND_Opaque;
	Mat->MaterialDomain = MD_Surface;
	CachedMaterial = Mat;
	return Mat;
}

UStaticMesh* BuildingStaticMeshUtils::CreateTransientStaticMesh(
	UObject* Outer,
	FName MeshName,
	const FExtrudedPrismMesh& Mesh,
	UMaterialInterface* Material,
	FString& OutError)
{
	if (Mesh.Vertices.Num() < 3 || Mesh.Triangles.Num() < 3 || (Mesh.Triangles.Num() % 3) != 0)
	{
		OutError = TEXT("Invalid mesh for StaticMesh build.");
		return nullptr;
	}

	FMeshDescription MeshDescription;
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

	const FPolygonGroupID GroupId = MeshDescription.CreatePolygonGroup();
	const int32 NumTris = Mesh.Triangles.Num() / 3;
	for (int32 T = 0; T < NumTris; ++T)
	{
		const int32 I0 = Mesh.Triangles[T * 3 + 0];
		const int32 I1 = Mesh.Triangles[T * 3 + 1];
		const int32 I2 = Mesh.Triangles[T * 3 + 2];
		if (!VertexIds.IsValidIndex(I0) || !VertexIds.IsValidIndex(I1) || !VertexIds.IsValidIndex(I2))
		{
			OutError = TEXT("Triangle index out of range while building StaticMesh.");
			return nullptr;
		}

		TArray<FVertexInstanceID, TInlineAllocator<3>> CornerIds;
		const int32 Indices[3] = { I0, I1, I2 };
		for (int32 C = 0; C < 3; ++C)
		{
			const int32 Vi = Indices[C];
			const FVertexInstanceID InstanceId = MeshDescription.CreateVertexInstance(VertexIds[Vi]);
			InstanceNormals[InstanceId] = Mesh.Normals.IsValidIndex(Vi)
				? FVector3f(Mesh.Normals[Vi])
				: FVector3f::UpVector;
			InstanceUVs.Set(InstanceId, 0, Mesh.UVs.IsValidIndex(Vi)
				? FVector2f(Mesh.UVs[Vi])
				: FVector2f::ZeroVector);
			CornerIds.Add(InstanceId);
		}

		MeshDescription.CreatePolygon(GroupId, CornerIds);
	}

	UObject* MeshOuter = Outer ? Outer : GetTransientPackage();
	UStaticMesh* StaticMesh = NewObject<UStaticMesh>(
		MeshOuter,
		MeshName,
		RF_Public | RF_Standalone | RF_Transient);
	if (!StaticMesh)
	{
		OutError = TEXT("Failed to allocate UStaticMesh.");
		return nullptr;
	}

	UMaterialInterface* Mat = Material ? Material : GetTwoSidedBuildingMaterial();
	StaticMesh->GetStaticMaterials().Add(FStaticMaterial(Mat));

	UStaticMesh::FBuildMeshDescriptionsParams BuildParams;
	BuildParams.bBuildSimpleCollision = false;
#if ENGINE_MAJOR_VERSION == 5
	BuildParams.bFastBuild = true;
#endif

	TArray<const FMeshDescription*> Descriptions;
	Descriptions.Add(&MeshDescription);
	StaticMesh->BuildFromMeshDescriptions(Descriptions, BuildParams);
	StaticMesh->MarkPackageDirty();
	return StaticMesh;
}
