#include "RoadStaticMesh.h"
#include "RoadPlacerLog.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "MeshDescription.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
#include "StaticMeshAttributes.h"
#include "UObject/ObjectRedirector.h"
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
			Out = TEXT("RoadMesh");
		}
		if (FChar::IsDigit(Out[0]))
		{
			Out = TEXT("R_") + Out;
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
			OutError = FString::Printf(TEXT("Failed to save package '%s'."), *Package->GetName());
			return false;
		}
		return true;
	}

	void ClearRedirectorAt(UPackage* Package, const FString& AssetName)
	{
		if (!Package)
		{
			return;
		}
		UObjectRedirector* Redirector = FindObject<UObjectRedirector>(Package, *AssetName);
		if (!Redirector)
		{
			Redirector = LoadObject<UObjectRedirector>(nullptr, *(Package->GetName() + TEXT(".") + AssetName));
		}
		if (!Redirector)
		{
			return;
		}
		Redirector->ClearFlags(RF_Public | RF_Standalone);
		Redirector->Rename(nullptr, GetTransientPackage(), REN_DontCreateRedirectors | REN_NonTransactional | REN_DoNotDirty);
		Redirector->MarkAsGarbage();
	}

	void FillMeshDescription(
		const TArray<FVector>& Vertices,
		const TArray<int32>& Triangles,
		double MetersPerUv,
		int32 SmoothShadingPasses,
		FMeshDescription& MeshDescription,
		FString& OutError)
	{
		FStaticMeshAttributes Attributes(MeshDescription);
		Attributes.Register();
		Attributes.GetVertexInstanceUVs().SetNumChannels(1);
		TVertexAttributesRef<FVector3f> Positions = Attributes.GetVertexPositions();
		TVertexInstanceAttributesRef<FVector3f> Normals = Attributes.GetVertexInstanceNormals();
		TVertexInstanceAttributesRef<FVector2f> UVs = Attributes.GetVertexInstanceUVs();

		TArray<FVertexID> Vids;
		Vids.Reserve(Vertices.Num());
		for (const FVector& V : Vertices)
		{
			const FVertexID Id = MeshDescription.CreateVertex();
			Positions[Id] = FVector3f(V);
			Vids.Add(Id);
		}

		const FPolygonGroupID Group = MeshDescription.CreatePolygonGroup();
		const double UvScale = FMath::Max(MetersPerUv * 100.0, 1.0);
		const int32 NumTris = Triangles.Num() / 3;
		for (int32 T = 0; T < NumTris; ++T)
		{
			const int32 I0 = Triangles[T * 3 + 0];
			const int32 I1 = Triangles[T * 3 + 1];
			const int32 I2 = Triangles[T * 3 + 2];
			if (!Vids.IsValidIndex(I0) || !Vids.IsValidIndex(I1) || !Vids.IsValidIndex(I2))
			{
				OutError = TEXT("Triangle index out of range.");
				return;
			}

			TArray<FVertexInstanceID, TInlineAllocator<3>> Corners;
			const int32 Idx[3] = { I0, I1, I2 };
			for (int32 C = 0; C < 3; ++C)
			{
				const int32 Vi = Idx[C];
				const FVertexInstanceID Inst = MeshDescription.CreateVertexInstance(Vids[Vi]);
				const FVector& P = Vertices[Vi];
				UVs.Set(Inst, 0, FVector2f(static_cast<float>(P.X / UvScale), static_cast<float>(P.Y / UvScale)));
				Normals[Inst] = FVector3f::UpVector;
				Corners.Add(Inst);
			}
			MeshDescription.CreatePolygon(Group, Corners);
		}

		MeshDescription.TriangulateMesh();
		MeshDescription.ReverseAllPolygonFacing();

		const int32 Passes = FMath::Clamp(SmoothShadingPasses, 0, 8);
		const bool bSmooth = Passes > 0;

		TEdgeAttributesRef<bool> Hard = Attributes.GetEdgeHardnesses();
		for (const FEdgeID E : MeshDescription.Edges().GetElementIDs())
		{
			Hard[E] = !bSmooth;
		}

		TMap<FVertexID, FVector3f> Accum;
		TMap<FVertexID, TArray<FVertexID>> Neighbors;
		auto AddNeighbor = [&Neighbors](FVertexID A, FVertexID B)
		{
			if (A != B)
			{
				Neighbors.FindOrAdd(A).AddUnique(B);
				Neighbors.FindOrAdd(B).AddUnique(A);
			}
		};

		for (const FTriangleID TriId : MeshDescription.Triangles().GetElementIDs())
		{
			const TArrayView<const FVertexInstanceID> Corners = MeshDescription.GetTriangleVertexInstances(TriId);
			if (Corners.Num() < 3)
			{
				continue;
			}
			const FVertexID V0 = MeshDescription.GetVertexInstanceVertex(Corners[0]);
			const FVertexID V1 = MeshDescription.GetVertexInstanceVertex(Corners[1]);
			const FVertexID V2 = MeshDescription.GetVertexInstanceVertex(Corners[2]);
			FVector3f N = FVector3f::CrossProduct(Positions[V1] - Positions[V0], Positions[V2] - Positions[V0]).GetSafeNormal();
			if (N.Z < 0.0f)
			{
				N = -N;
			}
			if (N.IsNearlyZero())
			{
				N = FVector3f::UpVector;
			}
			if (!bSmooth)
			{
				Normals[Corners[0]] = N;
				Normals[Corners[1]] = N;
				Normals[Corners[2]] = N;
				continue;
			}
			Accum.FindOrAdd(V0) += N;
			Accum.FindOrAdd(V1) += N;
			Accum.FindOrAdd(V2) += N;
			AddNeighbor(V0, V1);
			AddNeighbor(V1, V2);
			AddNeighbor(V2, V0);
		}

		if (!bSmooth)
		{
			return;
		}

		for (TPair<FVertexID, FVector3f>& Pair : Accum)
		{
			Pair.Value = Pair.Value.GetSafeNormal();
			if (Pair.Value.IsNearlyZero())
			{
				Pair.Value = FVector3f::UpVector;
			}
		}

		for (int32 Extra = 1; Extra < Passes; ++Extra)
		{
			TMap<FVertexID, FVector3f> Next;
			Next.Reserve(Accum.Num());
			for (const TPair<FVertexID, FVector3f>& Pair : Accum)
			{
				FVector3f Sum = Pair.Value;
				int32 Count = 1;
				if (const TArray<FVertexID>* Adj = Neighbors.Find(Pair.Key))
				{
					for (const FVertexID Neighbor : *Adj)
					{
						if (const FVector3f* NeighborN = Accum.Find(Neighbor))
						{
							Sum += *NeighborN;
							++Count;
						}
					}
				}
				FVector3f Blurred = (Sum / static_cast<float>(Count)).GetSafeNormal();
				if (Blurred.IsNearlyZero())
				{
					Blurred = FVector3f::UpVector;
				}
				Next.Add(Pair.Key, Blurred);
			}
			Accum = MoveTemp(Next);
		}

		for (const FTriangleID TriId : MeshDescription.Triangles().GetElementIDs())
		{
			const TArrayView<const FVertexInstanceID> Corners = MeshDescription.GetTriangleVertexInstances(TriId);
			for (const FVertexInstanceID Corner : Corners)
			{
				if (const FVector3f* N = Accum.Find(MeshDescription.GetVertexInstanceVertex(Corner)))
				{
					Normals[Corner] = *N;
				}
			}
		}
	}
}

UStaticMesh* RoadStaticMesh::CreatePersistentStaticMesh(
	const FString& PackageFolder,
	const FString& AssetName,
	const TArray<FVector>& LocalVertices,
	const TArray<int32>& Triangles,
	UMaterialInterface* Material,
	double MetersPerUv,
	int32 SmoothShadingPasses,
	FString& OutError)
{
	OutError.Reset();
	if (LocalVertices.Num() < 3 || Triangles.Num() < 3)
	{
		OutError = TEXT("Invalid road mesh.");
		return nullptr;
	}

	const FString SafeName = SanitizeAssetName(AssetName);
	FString Folder = PackageFolder.TrimStartAndEnd();
	Folder.RemoveFromEnd(TEXT("/"));
	if (Folder.IsEmpty())
	{
		Folder = TEXT("/Game/RoadPlacer/Meshes");
	}
	else if (!Folder.StartsWith(TEXT("/")))
	{
		Folder = TEXT("/") + Folder;
	}

	const FString SafePackagePath = Folder / SafeName;
	UPackage* Package = CreatePackage(*SafePackagePath);
	if (!Package)
	{
		OutError = FString::Printf(TEXT("Failed to create package '%s'."), *SafePackagePath);
		return nullptr;
	}
	Package->FullyLoad();
	ClearRedirectorAt(Package, SafeName);

	UStaticMesh* StaticMesh = FindObject<UStaticMesh>(Package, *SafeName);
	if (!StaticMesh)
	{
		StaticMesh = LoadObject<UStaticMesh>(nullptr, *(SafePackagePath + TEXT(".") + SafeName));
	}
	const bool bCreatedNew = (StaticMesh == nullptr);
	if (!StaticMesh)
	{
		ClearRedirectorAt(Package, SafeName);
		StaticMesh = NewObject<UStaticMesh>(Package, *SafeName, RF_Public | RF_Standalone | RF_Transactional);
	}
	if (!StaticMesh)
	{
		OutError = TEXT("Failed to allocate road UStaticMesh.");
		return nullptr;
	}

	FMeshDescription MeshDescription;
	FillMeshDescription(LocalVertices, Triangles, MetersPerUv, SmoothShadingPasses, MeshDescription, OutError);
	if (!OutError.IsEmpty())
	{
		return nullptr;
	}

	StaticMesh->Modify();
	StaticMesh->NaniteSettings.bEnabled = false;
	StaticMesh->GetStaticMaterials().Reset();
	StaticMesh->GetStaticMaterials().Add(FStaticMaterial(Material));

	UStaticMesh::FBuildMeshDescriptionsParams BuildParams;
	BuildParams.bBuildSimpleCollision = false;
	BuildParams.bFastBuild = false;
	TArray<const FMeshDescription*> Descriptions;
	Descriptions.Add(&MeshDescription);
	StaticMesh->BuildFromMeshDescriptions(Descriptions, BuildParams);

	StaticMesh->GetStaticMaterials().Reset();
	StaticMesh->GetStaticMaterials().Add(FStaticMaterial(Material));
	if (StaticMesh->GetNumSections(0) > 0)
	{
		StaticMesh->GetSectionInfoMap().Set(0, 0, FMeshSectionInfo(0));
	}
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

AStaticMeshActor* RoadStaticMesh::SpawnMeshActor(
	UWorld& World,
	const FVector& Origin,
	UStaticMesh* Mesh,
	UMaterialInterface* Material,
	const FString& Label,
	const FString& FolderPath,
	const FName& Tag,
	bool bEnableCollision)
{
	if (!Mesh)
	{
		return nullptr;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	SpawnParams.ObjectFlags |= RF_Transactional;
	AStaticMeshActor* Actor = World.SpawnActor<AStaticMeshActor>(Origin, FRotator::ZeroRotator, SpawnParams);
	if (!Actor)
	{
		return nullptr;
	}

	Actor->bIsEditorOnlyActor = false;
	Actor->SetActorHiddenInGame(false);
	if (UStaticMeshComponent* Comp = Actor->GetStaticMeshComponent())
	{
		Comp->SetMobility(EComponentMobility::Static);
		Comp->bIsEditorOnly = false;
		Comp->SetStaticMesh(Mesh);
		if (Material)
		{
			Comp->SetMaterial(0, Material);
		}
		Comp->SetCollisionEnabled(bEnableCollision ? ECollisionEnabled::QueryAndPhysics : ECollisionEnabled::NoCollision);
		Comp->SetCastShadow(true);
	}

	Actor->SetActorLabel(Label);
	Actor->Tags.AddUnique(Tag);
	Actor->Tags.AddUnique(FName(TEXT("Road")));
	if (!FolderPath.IsEmpty())
	{
		Actor->SetFolderPath(FName(*FolderPath));
	}
	Actor->Modify();
	return Actor;
}
