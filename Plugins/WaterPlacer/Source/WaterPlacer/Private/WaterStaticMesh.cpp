#include "WaterStaticMesh.h"
#include "WaterPlacerLog.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionAdd.h"
#include "Materials/MaterialExpressionComponentMask.h"
#include "Materials/MaterialExpressionConstant3Vector.h"
#include "Materials/MaterialExpressionFresnel.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionSine.h"
#include "Materials/MaterialExpressionTime.h"
#include "Materials/MaterialExpressionWorldPosition.h"
#include "Materials/MaterialInstanceConstant.h"
#include "Materials/MaterialInterface.h"
#include "MeshDescription.h"
#include "Misc/EngineVersionComparison.h"
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

	void FillMeshDescription(
		const FWaterFlatMesh& Mesh,
		int32 SmoothShadingPasses,
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
		// Same as Building Extruder: Unreal treats our XY winding as back-facing.
		MeshDescription.ReverseAllPolygonFacing();

		const int32 Passes = FMath::Clamp(SmoothShadingPasses, 0, 8);
		const bool bSmooth = Passes > 0;

		TEdgeAttributesRef<bool> EdgeHardnesses = Attributes.GetEdgeHardnesses();
		for (const FEdgeID EdgeId : MeshDescription.Edges().GetElementIDs())
		{
			EdgeHardnesses[EdgeId] = !bSmooth;
		}

		TMap<FVertexID, FVector3f> VertexNormals;
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
			const TArrayView<const FVertexInstanceID> Corners =
				MeshDescription.GetTriangleVertexInstances(TriId);
			if (Corners.Num() < 3)
			{
				continue;
			}

			const FVertexID V0 = MeshDescription.GetVertexInstanceVertex(Corners[0]);
			const FVertexID V1 = MeshDescription.GetVertexInstanceVertex(Corners[1]);
			const FVertexID V2 = MeshDescription.GetVertexInstanceVertex(Corners[2]);
			const FVector3f PA = VertexPositions[V0];
			const FVector3f PB = VertexPositions[V1];
			const FVector3f PC = VertexPositions[V2];
			FVector3f Normal = FVector3f::CrossProduct(PB - PA, PC - PA).GetSafeNormal();
			if (Normal.Z < 0.0f)
			{
				Normal = -Normal;
			}
			if (Normal.IsNearlyZero())
			{
				Normal = FVector3f::UpVector;
			}

			if (!bSmooth)
			{
				InstanceNormals[Corners[0]] = Normal;
				InstanceNormals[Corners[1]] = Normal;
				InstanceNormals[Corners[2]] = Normal;
				continue;
			}

			VertexNormals.FindOrAdd(V0) += Normal;
			VertexNormals.FindOrAdd(V1) += Normal;
			VertexNormals.FindOrAdd(V2) += Normal;
			AddNeighbor(V0, V1);
			AddNeighbor(V1, V2);
			AddNeighbor(V2, V0);
		}

		if (!bSmooth)
		{
			return;
		}

		for (TPair<FVertexID, FVector3f>& Pair : VertexNormals)
		{
			Pair.Value = Pair.Value.GetSafeNormal();
			if (Pair.Value.IsNearlyZero())
			{
				Pair.Value = FVector3f::UpVector;
			}
		}

		for (int32 Extra = 1; Extra < Passes; ++Extra)
		{
			TMap<FVertexID, FVector3f> NextNormals;
			NextNormals.Reserve(VertexNormals.Num());
			for (const TPair<FVertexID, FVector3f>& Pair : VertexNormals)
			{
				FVector3f Sum = Pair.Value;
				int32 Count = 1;
				if (const TArray<FVertexID>* Adj = Neighbors.Find(Pair.Key))
				{
					for (const FVertexID Neighbor : *Adj)
					{
						if (const FVector3f* NeighborN = VertexNormals.Find(Neighbor))
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
				NextNormals.Add(Pair.Key, Blurred);
			}
			VertexNormals = MoveTemp(NextNormals);
		}

		for (const FTriangleID TriId : MeshDescription.Triangles().GetElementIDs())
		{
			const TArrayView<const FVertexInstanceID> Corners =
				MeshDescription.GetTriangleVertexInstances(TriId);
			for (const FVertexInstanceID Corner : Corners)
			{
				const FVertexID Vid = MeshDescription.GetVertexInstanceVertex(Corner);
				if (const FVector3f* SmoothNormal = VertexNormals.Find(Vid))
				{
					InstanceNormals[Corner] = *SmoothNormal;
				}
			}
		}
	}

	UMaterialInterface* CreateTwoSidedInstance(UMaterialInterface* Source, const FString& ContentFolder, FString& OutError)
	{
		FString Folder = ContentFolder.TrimStartAndEnd();
		Folder.RemoveFromEnd(TEXT("/"));
		if (Folder.Contains(TEXT("/Meshes")))
		{
			Folder = Folder.Replace(TEXT("/Meshes"), TEXT("/Materials"));
		}
		if (Folder.IsEmpty() || Folder == TEXT("/Game"))
		{
			Folder = TEXT("/Game/WaterPlacer/Materials");
		}
		if (!Folder.StartsWith(TEXT("/")))
		{
			Folder = TEXT("/") + Folder;
		}

		const FString AssetName = TEXT("MI_WaterPlacer_Surface");
		const FString PackagePath = Folder / AssetName;
		UPackage* Package = CreatePackage(*PackagePath);
		if (!Package)
		{
			OutError = TEXT("Failed to create water material package.");
			return Source;
		}
		Package->FullyLoad();
		ClearRedirectorAt(Package, AssetName);

		UMaterialInstanceConstant* MIC = FindObject<UMaterialInstanceConstant>(Package, *AssetName);
		if (!MIC)
		{
			MIC = LoadObject<UMaterialInstanceConstant>(nullptr, *(PackagePath + TEXT(".") + AssetName));
		}
		const bool bCreatedNew = (MIC == nullptr);
		if (!MIC)
		{
			ClearRedirectorAt(Package, AssetName);
			MIC = NewObject<UMaterialInstanceConstant>(
				Package,
				*AssetName,
				RF_Public | RF_Standalone | RF_Transactional);
		}
		if (!MIC)
		{
			OutError = TEXT("Failed to allocate water material instance.");
			return Source;
		}

		MIC->Modify();
		MIC->SetParentEditorOnly(Source);
		MIC->BasePropertyOverrides.bOverride_TwoSided = true;
		MIC->BasePropertyOverrides.TwoSided = true;
		MIC->PostEditChange();

		if (bCreatedNew)
		{
			FAssetRegistryModule::AssetCreated(MIC);
		}
		if (!SaveAssetPackage(Package, MIC, OutError))
		{
			return Source;
		}
		OutError.Reset();
		return MIC;
	}

#if UE_VERSION_OLDER_THAN(5, 2, 0)
	UMaterial* MaterialEditData(UMaterial* Mat)
	{
		return Mat;
	}
#else
	UMaterialEditorOnlyData* MaterialEditData(UMaterial* Mat)
	{
		return Mat ? Mat->GetEditorOnlyData() : nullptr;
	}
#endif

	template <typename TExpr>
	TExpr* NewMatExpr(UMaterial* Mat)
	{
		TExpr* Expr = NewObject<TExpr>(Mat);
		Expr->Material = Mat;
#if UE_VERSION_OLDER_THAN(5, 2, 0)
		Mat->Expressions.Add(Expr);
#else
		Mat->GetExpressionCollection().AddExpression(Expr);
#endif
		return Expr;
	}

	void BuildWavyWaterGraph(UMaterial* Mat)
	{
		Mat->TwoSided = true;
		Mat->BlendMode = BLEND_Translucent;
		Mat->TranslucencyLightingMode = TLM_Surface;
		Mat->MaterialDomain = MD_Surface;

		auto* Edit = MaterialEditData(Mat);
		if (!Edit)
		{
			return;
		}

		UMaterialExpressionConstant3Vector* WaterColor = NewMatExpr<UMaterialExpressionConstant3Vector>(Mat);
		WaterColor->Constant = FLinearColor(0.02f, 0.18f, 0.28f);
		Edit->BaseColor.Expression = WaterColor;

		UMaterialExpressionFresnel* Fresnel = NewMatExpr<UMaterialExpressionFresnel>(Mat);
		UMaterialExpressionAdd* Opacity = NewMatExpr<UMaterialExpressionAdd>(Mat);
		Opacity->A.Expression = Fresnel;
		Opacity->ConstA = 0.0f;
		Opacity->ConstB = 0.35f;
		Edit->Opacity.Expression = Opacity;

		UMaterialExpressionConstant3Vector* Spec = NewMatExpr<UMaterialExpressionConstant3Vector>(Mat);
		Spec->Constant = FLinearColor(0.55f, 0.55f, 0.55f);
		Edit->Specular.Expression = Spec;

		UMaterialExpressionWorldPosition* WorldPos = NewMatExpr<UMaterialExpressionWorldPosition>(Mat);
		UMaterialExpressionTime* Time = NewMatExpr<UMaterialExpressionTime>(Mat);

		UMaterialExpressionComponentMask* X = NewMatExpr<UMaterialExpressionComponentMask>(Mat);
		X->Input.Expression = WorldPos;
		X->R = true;
		X->G = false;
		X->B = false;
		X->A = false;

		UMaterialExpressionComponentMask* Y = NewMatExpr<UMaterialExpressionComponentMask>(Mat);
		Y->Input.Expression = WorldPos;
		Y->R = false;
		Y->G = true;
		Y->B = false;
		Y->A = false;

		UMaterialExpressionMultiply* XFreq = NewMatExpr<UMaterialExpressionMultiply>(Mat);
		XFreq->A.Expression = X;
		XFreq->ConstB = 0.008f;

		UMaterialExpressionMultiply* Time1 = NewMatExpr<UMaterialExpressionMultiply>(Mat);
		Time1->A.Expression = Time;
		Time1->ConstB = 0.7f;

		UMaterialExpressionAdd* Wave1In = NewMatExpr<UMaterialExpressionAdd>(Mat);
		Wave1In->A.Expression = XFreq;
		Wave1In->B.Expression = Time1;

		UMaterialExpressionSine* Wave1 = NewMatExpr<UMaterialExpressionSine>(Mat);
		Wave1->Input.Expression = Wave1In;

		UMaterialExpressionMultiply* YFreq = NewMatExpr<UMaterialExpressionMultiply>(Mat);
		YFreq->A.Expression = Y;
		YFreq->ConstB = 0.011f;

		UMaterialExpressionMultiply* Time2 = NewMatExpr<UMaterialExpressionMultiply>(Mat);
		Time2->A.Expression = Time;
		Time2->ConstB = 0.95f;

		UMaterialExpressionAdd* Wave2In = NewMatExpr<UMaterialExpressionAdd>(Mat);
		Wave2In->A.Expression = YFreq;
		Wave2In->B.Expression = Time2;

		UMaterialExpressionSine* Wave2 = NewMatExpr<UMaterialExpressionSine>(Mat);
		Wave2->Input.Expression = Wave2In;

		UMaterialExpressionAdd* WaveSum = NewMatExpr<UMaterialExpressionAdd>(Mat);
		WaveSum->A.Expression = Wave1;
		WaveSum->B.Expression = Wave2;

		UMaterialExpressionMultiply* WaveHeight = NewMatExpr<UMaterialExpressionMultiply>(Mat);
		WaveHeight->A.Expression = WaveSum;
		WaveHeight->ConstB = 18.0f;

		UMaterialExpressionConstant3Vector* Up = NewMatExpr<UMaterialExpressionConstant3Vector>(Mat);
		Up->Constant = FLinearColor(0.0f, 0.0f, 1.0f);

		UMaterialExpressionMultiply* WPO = NewMatExpr<UMaterialExpressionMultiply>(Mat);
		WPO->A.Expression = Up;
		WPO->B.Expression = WaveHeight;
		Edit->WorldPositionOffset.Expression = WPO;
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
		OutMesh.Triangles.Add(A);
		OutMesh.Triangles.Add(B);
		OutMesh.Triangles.Add(C);
	}
	return OutMesh.Triangles.Num() >= 3;
}

UMaterialInterface* WaterStaticMesh::GetOrCreateWavyWaterMaterial(
	const FString& ContentFolder,
	FString& OutError)
{
	OutError.Reset();
	FString Folder = ContentFolder.TrimStartAndEnd();
	Folder.RemoveFromEnd(TEXT("/"));
	if (Folder.Contains(TEXT("/Meshes")))
	{
		Folder = Folder.Replace(TEXT("/Meshes"), TEXT("/Materials"));
	}
	if (Folder.IsEmpty() || Folder == TEXT("/Game"))
	{
		Folder = TEXT("/Game/WaterPlacer/Materials");
	}
	if (!Folder.StartsWith(TEXT("/")))
	{
		Folder = TEXT("/") + Folder;
	}

	const FString AssetName = TEXT("M_WaterPlacer_Waves");
	const FString PackagePath = Folder / AssetName;
	const FString ObjectPath = PackagePath + TEXT(".") + AssetName;
	if (UMaterial* Existing = LoadObject<UMaterial>(nullptr, *ObjectPath))
	{
		return Existing;
	}

	UPackage* Package = CreatePackage(*PackagePath);
	if (!Package)
	{
		OutError = TEXT("Failed to create wavy water material package.");
		return nullptr;
	}
	Package->FullyLoad();
	ClearRedirectorAt(Package, AssetName);

	UMaterial* Mat = NewObject<UMaterial>(
		Package,
		*AssetName,
		RF_Public | RF_Standalone | RF_Transactional);
	if (!Mat)
	{
		OutError = TEXT("Failed to allocate wavy water material.");
		return nullptr;
	}

	BuildWavyWaterGraph(Mat);
	Mat->PostEditChange();
	FAssetRegistryModule::AssetCreated(Mat);
	if (!SaveAssetPackage(Package, Mat, OutError))
	{
		return nullptr;
	}

	UE_LOG(LogWaterPlacer, Display, TEXT("Created wavy water material '%s'."), *ObjectPath);
	return Mat;
}

UMaterialInterface* WaterStaticMesh::PrepareMaterialForStaticMesh(
	UMaterialInterface* Source,
	const FString& ContentFolder,
	FString& OutError)
{
	OutError.Reset();

	auto IsSingleLayerWaterMaterial = [](UMaterialInterface* Mat) -> bool
	{
		if (!Mat)
		{
			return false;
		}
		const FString Path = Mat->GetPathName();
		if (Path.Contains(TEXT("Water_Material_Ocean")) || Path.Contains(TEXT("Water_Material_Lake")))
		{
			return true;
		}
		return Mat->GetShadingModels().HasShadingModel(MSM_SingleLayerWater);
	};

	if (!Source || IsSingleLayerWaterMaterial(Source))
	{
		if (Source)
		{
			UE_LOG(
				LogWaterPlacer,
				Warning,
				TEXT("Material '%s' is Single Layer Water and will not draw on StaticMeshActors. Using M_WaterPlacer_Waves instead."),
				*Source->GetPathName());
		}
		return GetOrCreateWavyWaterMaterial(ContentFolder, OutError);
	}

	return CreateTwoSidedInstance(Source, ContentFolder, OutError);
}

UStaticMesh* WaterStaticMesh::CreatePersistentStaticMesh(
	const FString& PackageFolder,
	const FString& AssetName,
	const FWaterFlatMesh& Mesh,
	UMaterialInterface* Material,
	int32 SmoothShadingPasses,
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
	FillMeshDescription(Mesh, SmoothShadingPasses, MeshDescription, OutError);
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
