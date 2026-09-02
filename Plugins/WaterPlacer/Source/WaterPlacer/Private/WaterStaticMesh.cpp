#include "WaterStaticMesh.h"
#include "WaterPlacerLog.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
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
			Out = TEXT("WaterMesh");
		}
		if (FChar::IsDigit(Out[0]))
		{
			Out = TEXT("W_") + Out;
		}
		return Out;
	}

	void StripClosingDuplicate(TArray<FVector>& Ring)
	{
		if (Ring.Num() >= 2 && Ring[0].Equals(Ring.Last(), 1.0e-3))
		{
			Ring.Pop();
		}
	}

	double SignedArea2XY(const TArray<FVector>& Ring)
	{
		double Area2 = 0.0;
		const int32 N = Ring.Num();
		for (int32 I = 0; I < N; ++I)
		{
			const FVector& A = Ring[I];
			const FVector& B = Ring[(I + 1) % N];
			Area2 += A.X * B.Y - B.X * A.Y;
		}
		return Area2;
	}

	bool PointInTriangleXY(const FVector& P, const FVector& A, const FVector& B, const FVector& C)
	{
		const double D1 = (P.X - B.X) * (A.Y - B.Y) - (A.X - B.X) * (P.Y - B.Y);
		const double D2 = (P.X - C.X) * (B.Y - C.Y) - (B.X - C.X) * (P.Y - C.Y);
		const double D3 = (P.X - A.X) * (C.Y - A.Y) - (C.X - A.X) * (P.Y - A.Y);
		const bool HasNeg = (D1 < 0.0) || (D2 < 0.0) || (D3 < 0.0);
		const bool HasPos = (D1 > 0.0) || (D2 > 0.0) || (D3 > 0.0);
		return !(HasNeg && HasPos);
	}

	bool IsConvexVertexXY(const FVector& Prev, const FVector& Curr, const FVector& Next, bool bCCW)
	{
		const double Cross = (Curr.X - Prev.X) * (Next.Y - Curr.Y) - (Curr.Y - Prev.Y) * (Next.X - Curr.X);
		return bCCW ? (Cross > 0.0) : (Cross < 0.0);
	}

	bool EarClipTriangulate(TArray<FVector> Ring, TArray<int32>& OutIndices, FString& OutError)
	{
		OutIndices.Reset();
		StripClosingDuplicate(Ring);
		const int32 N0 = Ring.Num();
		if (N0 < 3)
		{
			OutError = TEXT("Polygon has fewer than 3 vertices.");
			return false;
		}

		const bool bCCW = SignedArea2XY(Ring) > 0.0;
		TArray<int32> V;
		V.Reserve(N0);
		for (int32 I = 0; I < N0; ++I)
		{
			V.Add(I);
		}

		int32 Guard = 0;
		const int32 GuardMax = N0 * N0 + 8;
		while (V.Num() > 3 && Guard++ < GuardMax)
		{
			bool bClipped = false;
			for (int32 I = 0; I < V.Num(); ++I)
			{
				const int32 IPrev = (I + V.Num() - 1) % V.Num();
				const int32 INext = (I + 1) % V.Num();
				const int32 A = V[IPrev];
				const int32 B = V[I];
				const int32 C = V[INext];

				if (!IsConvexVertexXY(Ring[A], Ring[B], Ring[C], bCCW))
				{
					continue;
				}

				bool bEar = true;
				for (int32 J = 0; J < V.Num(); ++J)
				{
					const int32 P = V[J];
					if (P == A || P == B || P == C)
					{
						continue;
					}
					if (PointInTriangleXY(Ring[P], Ring[A], Ring[B], Ring[C]))
					{
						bEar = false;
						break;
					}
				}
				if (!bEar)
				{
					continue;
				}

				OutIndices.Add(A);
				OutIndices.Add(B);
				OutIndices.Add(C);
				V.RemoveAt(I);
				bClipped = true;
				break;
			}
			if (!bClipped)
			{
				OutError = TEXT("Ear clipping failed (self-intersecting or degenerate polygon).");
				return false;
			}
		}

		if (V.Num() == 3)
		{
			OutIndices.Add(V[0]);
			OutIndices.Add(V[1]);
			OutIndices.Add(V[2]);
			return true;
		}

		OutError = TEXT("Ear clipping did not finish.");
		return false;
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

	void ClearRedirectorAt(UPackage* Package, const FString& AssetName)
	{
		if (!Package)
		{
			return;
		}

		UObjectRedirector* Redirector = FindObject<UObjectRedirector>(Package, *AssetName);
		if (!Redirector)
		{
			const FString ObjectPath = Package->GetName() + TEXT(".") + AssetName;
			Redirector = LoadObject<UObjectRedirector>(nullptr, *ObjectPath);
		}
		if (!Redirector)
		{
			return;
		}

		Redirector->ClearFlags(RF_Public | RF_Standalone);
		Redirector->Rename(
			nullptr,
			GetTransientPackage(),
			REN_DontCreateRedirectors | REN_NonTransactional | REN_DoNotDirty);
		Redirector->MarkAsGarbage();
	}

	void FillMeshDescription(const FWaterFlatMesh& Mesh, FMeshDescription& MeshDescription, FString& OutError)
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

		const FPolygonGroupID Group = MeshDescription.CreatePolygonGroup();
		const int32 NumTris = Mesh.Triangles.Num() / 3;
		for (int32 T = 0; T < NumTris; ++T)
		{
			const int32 I0 = Mesh.Triangles[T * 3 + 0];
			const int32 I1 = Mesh.Triangles[T * 3 + 1];
			const int32 I2 = Mesh.Triangles[T * 3 + 2];
			if (!VertexIds.IsValidIndex(I0) || !VertexIds.IsValidIndex(I1) || !VertexIds.IsValidIndex(I2))
			{
				OutError = TEXT("Triangle index out of range while building water StaticMesh.");
				return;
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

			MeshDescription.CreatePolygon(Group, CornerIds);
		}

		MeshDescription.TriangulateMesh();

		TEdgeAttributesRef<bool> EdgeHardnesses = Attributes.GetEdgeHardnesses();
		for (const FEdgeID EdgeId : MeshDescription.Edges().GetElementIDs())
		{
			EdgeHardnesses[EdgeId] = true;
		}
	}
}

bool WaterStaticMesh::BuildFlatPolygonMesh(
	const TArray<FVector>& LocalRing,
	double MetersPerUv,
	FWaterFlatMesh& OutMesh,
	FString& OutError)
{
	OutMesh = FWaterFlatMesh();
	TArray<FVector> Ring = LocalRing;
	StripClosingDuplicate(Ring);
	if (Ring.Num() < 3)
	{
		OutError = TEXT("Water polygon has fewer than 3 vertices.");
		return false;
	}

	TArray<int32> Indices;
	if (!EarClipTriangulate(Ring, Indices, OutError))
	{
		return false;
	}

	const double UvScale = FMath::Max(MetersPerUv * 100.0, 1.0);
	OutMesh.Vertices = Ring;
	OutMesh.Normals.Init(FVector::UpVector, Ring.Num());
	OutMesh.UVs.Reserve(Ring.Num());
	for (const FVector& P : Ring)
	{
		OutMesh.UVs.Add(FVector2D(P.X / UvScale, P.Y / UvScale));
	}

	OutMesh.Triangles.Reserve(Indices.Num());
	for (int32 T = 0; T + 2 < Indices.Num(); T += 3)
	{
		int32 A = Indices[T];
		int32 B = Indices[T + 1];
		int32 C = Indices[T + 2];
		const FVector N = FVector::CrossProduct(Ring[B] - Ring[A], Ring[C] - Ring[A]);
		if (N.Z < 0.0)
		{
			Swap(B, C);
		}
		OutMesh.Triangles.Add(A);
		OutMesh.Triangles.Add(B);
		OutMesh.Triangles.Add(C);
	}
	return OutMesh.Triangles.Num() >= 3;
}

UStaticMesh* WaterStaticMesh::CreatePersistentStaticMesh(
	const FString& PackageFolder,
	const FString& AssetName,
	const FWaterFlatMesh& Mesh,
	UMaterialInterface* Material,
	FString& OutError)
{
	OutError.Reset();
	if (Mesh.Vertices.Num() < 3 || Mesh.Triangles.Num() < 3 || (Mesh.Triangles.Num() % 3) != 0)
	{
		OutError = TEXT("Invalid mesh for water StaticMesh build.");
		return nullptr;
	}

	const FString SafeName = SanitizeAssetName(AssetName);
	FString Folder = PackageFolder.TrimStartAndEnd();
	Folder.RemoveFromEnd(TEXT("/"));
	if (Folder.IsEmpty())
	{
		Folder = TEXT("/Game/WaterPlacer/Meshes");
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
		StaticMesh = NewObject<UStaticMesh>(
			Package,
			*SafeName,
			RF_Public | RF_Standalone | RF_Transactional);
	}
	if (!StaticMesh)
	{
		OutError = TEXT("Failed to allocate water UStaticMesh.");
		return nullptr;
	}

	FMeshDescription MeshDescription;
	FillMeshDescription(Mesh, MeshDescription, OutError);
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

AStaticMeshActor* WaterStaticMesh::SpawnMeshActor(
	UWorld& World,
	const FVector& Origin,
	UStaticMesh* Mesh,
	UMaterialInterface* Material,
	const FString& Label,
	const FString& FolderPath,
	const FName& Tag)
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
		Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Comp->SetCastShadow(false);
	}

	Actor->SetActorLabel(Label);
	Actor->Tags.AddUnique(Tag);
	Actor->Tags.AddUnique(FName(TEXT("Water")));
	if (!FolderPath.IsEmpty())
	{
		Actor->SetFolderPath(FName(*FolderPath));
	}
	Actor->Modify();
	return Actor;
}
