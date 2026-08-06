#include "FootprintGeometryUtils.h"

#include "BuildingFootprintExportSettings.h"
#include "BuildingFootprintFilterSettings.h"
#include "BuildingFootprintExporterLog.h"
#include "FootprintGeoTransform.h"
#include "FootprintSilhouette.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Internationalization/Internationalization.h"
#include "MeshDescription.h"
#include "Misc/ScopedSlowTask.h"
#include "StaticMeshAttributes.h"
#include "StaticMeshResources.h"

namespace
{
	bool StaticMeshUsesNanite(const UStaticMesh& Mesh)
	{
		// UE 5.1 has no IsNaniteEnabled(); use settings + built Nanite data.
		return Mesh.NaniteSettings.bEnabled || Mesh.HasValidNaniteData();
	}

	bool NameContainsAny(const FString& Name, const TArray<FString>& Substrings)
	{
		if (Substrings.Num() == 0)
		{
			return false;
		}

		for (const FString& Sub : Substrings)
		{
			if (!Sub.IsEmpty() && Name.Contains(Sub, ESearchCase::IgnoreCase))
			{
				return true;
			}
		}
		return false;
	}

	bool HasAnyTag(const AActor& Actor, const TArray<FName>& Tags)
	{
		for (const FName& Tag : Tags)
		{
			if (Actor.ActorHasTag(Tag))
			{
				return true;
			}
		}
		return false;
	}

	bool HasAnyTag(const UActorComponent& Component, const TArray<FName>& Tags)
	{
		for (const FName& Tag : Tags)
		{
			if (Component.ComponentHasTag(Tag))
			{
				return true;
			}
		}
		return false;
	}

	bool BoundsOverlap2D(const FBox2D& A, const FBox2D& B, double ExpandCm)
	{
		const FBox2D AE(A.Min - FVector2D(ExpandCm, ExpandCm), A.Max + FVector2D(ExpandCm, ExpandCm));
		return AE.Intersect(B);
	}

	int32 FindRoot(TArray<int32>& Parent, int32 I)
	{
		while (Parent[I] != I)
		{
			Parent[I] = Parent[Parent[I]];
			I = Parent[I];
		}
		return I;
	}

	void UnionIdx(TArray<int32>& Parent, int32 A, int32 B)
	{
		const int32 RA = FindRoot(Parent, A);
		const int32 RB = FindRoot(Parent, B);
		if (RA != RB)
		{
			Parent[RB] = RA;
		}
	}

	FBuildingFootprintPolygon MakeSilhouettePolygon(
		const FSilhouettePolygon2D& Silhouette,
		const FString& ActorLabel,
		const FFootprintGeoTransform& Geo)
	{
		FBuildingFootprintPolygon Poly;
		Poly.SourceActorLabel = ActorLabel;
		Poly.AreaM2 = Silhouette.AreaM2;
		Poly.HeightM = Silhouette.HeightM;
		Poly.LonLatRing.Reserve(Silhouette.OuterRingCm.Num());
		for (const FVector2D& P : Silhouette.OuterRingCm)
		{
			Poly.LonLatRing.Add(Geo.UnrealToLonLat(FVector(P.X, P.Y, 0.0)));
		}
		Poly.HoleLonLatRings.Reserve(Silhouette.HoleRingsCm.Num());
		for (const TArray<FVector2D>& Hole : Silhouette.HoleRingsCm)
		{
			TArray<FVector2D> HoleLonLat;
			HoleLonLat.Reserve(Hole.Num());
			for (const FVector2D& P : Hole)
			{
				HoleLonLat.Add(Geo.UnrealToLonLat(FVector(P.X, P.Y, 0.0)));
			}
			Poly.HoleLonLatRings.Add(MoveTemp(HoleLonLat));
		}
		return Poly;
	}
}

bool FootprintGeometryUtils::ActorPassesFilter(
	const AActor& Actor,
	const UBuildingFootprintExportSettings& ExportSettings,
	const UBuildingFootprintFilterSettings* Filter)
{
	(void)ExportSettings;

	if (Actor.IsHidden())
	{
		return false;
	}

	if (!Filter)
	{
		return true;
	}

	if (Filter->ExcludeActorTags.Num() > 0 && HasAnyTag(Actor, Filter->ExcludeActorTags))
	{
		return false;
	}

	if (Filter->IncludeActorTags.Num() > 0 && !HasAnyTag(Actor, Filter->IncludeActorTags))
	{
		return false;
	}

	return true;
}

bool FootprintGeometryUtils::ComponentPassesFilter(
	const UStaticMeshComponent& Component,
	const UBuildingFootprintExportSettings& ExportSettings,
	const UBuildingFootprintFilterSettings* Filter)
{
	if (!Component.GetStaticMesh())
	{
		return false;
	}

	auto CollectMaterialNames = [&Component]() -> FString
	{
		FString Names;
		const int32 NumMaterials = Component.GetNumMaterials();
		for (int32 Index = 0; Index < NumMaterials; ++Index)
		{
			if (const UMaterialInterface* Material = Component.GetMaterial(Index))
			{
				Names += Material->GetName();
				Names += TEXT(" ");
			}
		}
		return Names;
	};

	const FString MaterialNames = CollectMaterialNames();

	if (!ExportSettings.IncludeMaterialNameContains.IsEmpty())
	{
		if (!MaterialNames.Contains(ExportSettings.IncludeMaterialNameContains, ESearchCase::IgnoreCase))
		{
			return false;
		}
	}

	if (Filter)
	{
		if (Filter->ExcludeMaterialNameContains.Num() > 0 && NameContainsAny(MaterialNames, Filter->ExcludeMaterialNameContains))
		{
			return false;
		}

		if (Filter->IncludeMaterialNameContains.Num() > 0 && !NameContainsAny(MaterialNames, Filter->IncludeMaterialNameContains))
		{
			return false;
		}

		if (Filter->ExcludeComponentTags.Num() > 0 && HasAnyTag(Component, Filter->ExcludeComponentTags))
		{
			return false;
		}

		if (Filter->IncludeComponentTags.Num() > 0 && !HasAnyTag(Component, Filter->IncludeComponentTags))
		{
			return false;
		}

		const FString CompName = Component.GetName();
		const FString MeshName = Component.GetStaticMesh()->GetName();
		const FString Combined = CompName + TEXT(" ") + MeshName;

		if (NameContainsAny(Combined, Filter->ExcludeNameContains))
		{
			return false;
		}

		if (Filter->IncludeNameContains.Num() > 0 && !NameContainsAny(Combined, Filter->IncludeNameContains))
		{
			return false;
		}

		if (Filter->MinComponentBoundsAreaM2 > 0.0)
		{
			const FBox Box = Component.Bounds.GetBox();
			const double ExtX = (Box.Max.X - Box.Min.X) / 100.0;
			const double ExtY = (Box.Max.Y - Box.Min.Y) / 100.0;
			const double AreaM2 = FMath::Abs(ExtX * ExtY);
			if (AreaM2 < Filter->MinComponentBoundsAreaM2)
			{
				return false;
			}
		}
	}

	return true;
}

bool FootprintGeometryUtils::CollectComponentGroundGeometry(
	const UStaticMeshComponent& Component,
	TArray<FVector2D>& OutGroundPointsCm,
	TArray<FGroundTriangle2D>& OutTrianglesCm)
{
	UStaticMesh* Mesh = Component.GetStaticMesh();
	if (!Mesh)
	{
		return false;
	}

	const FTransform& Xform = Component.GetComponentTransform();
	const int32 TrianglesBefore = OutTrianglesCm.Num();

	auto AddWorldTriangle = [&](const FVector& W0, const FVector& W1, const FVector& W2)
	{
		FGroundTriangle2D GroundTri;
		GroundTri.A = FVector2D(W0.X, W0.Y);
		GroundTri.B = FVector2D(W1.X, W1.Y);
		GroundTri.C = FVector2D(W2.X, W2.Y);
		GroundTri.ZA = W0.Z;
		GroundTri.ZB = W1.Z;
		GroundTri.ZC = W2.Z;
		OutTrianglesCm.Add(GroundTri);
	};

	// For Nanite meshes, LOD0 render data is a coarse fallback (caused H→C footprints).
	// Only then read the source MeshDescription. Non-Nanite keeps LOD0 (faster, same quality).
	if (StaticMeshUsesNanite(*Mesh))
	{
		FMeshDescription MeshDesc;
		if (Mesh->CloneMeshDescription(0, MeshDesc))
		{
			FStaticMeshAttributes Attributes(MeshDesc);
			TVertexAttributesRef<FVector3f> VertexPositions = Attributes.GetVertexPositions();
			if (VertexPositions.IsValid() && MeshDesc.Triangles().Num() > 0)
			{
				OutTrianglesCm.Reserve(OutTrianglesCm.Num() + MeshDesc.Triangles().Num());

				for (const FTriangleID TriangleID : MeshDesc.Triangles().GetElementIDs())
				{
					const FVertexID V0 = MeshDesc.GetVertexInstanceVertex(MeshDesc.GetTriangleVertexInstance(TriangleID, 0));
					const FVertexID V1 = MeshDesc.GetVertexInstanceVertex(MeshDesc.GetTriangleVertexInstance(TriangleID, 1));
					const FVertexID V2 = MeshDesc.GetVertexInstanceVertex(MeshDesc.GetTriangleVertexInstance(TriangleID, 2));

					const FVector3f& P0 = VertexPositions[V0];
					const FVector3f& P1 = VertexPositions[V1];
					const FVector3f& P2 = VertexPositions[V2];
					const FVector W0 = Xform.TransformPosition(FVector(P0.X, P0.Y, P0.Z));
					const FVector W1 = Xform.TransformPosition(FVector(P1.X, P1.Y, P1.Z));
					const FVector W2 = Xform.TransformPosition(FVector(P2.X, P2.Y, P2.Z));
					AddWorldTriangle(W0, W1, W2);
				}

				const int32 NumVerts = MeshDesc.Vertices().Num();
				if (NumVerts > 0)
				{
					const int32 Stride = FMath::Max(1, NumVerts / 8000);
					int32 Count = 0;
					OutGroundPointsCm.Reserve(OutGroundPointsCm.Num() + (NumVerts + Stride - 1) / Stride);
					for (const FVertexID VertexID : MeshDesc.Vertices().GetElementIDs())
					{
						if ((Count++ % Stride) != 0)
						{
							continue;
						}
						const FVector3f& P = VertexPositions[VertexID];
						const FVector W = Xform.TransformPosition(FVector(P.X, P.Y, P.Z));
						OutGroundPointsCm.Add(FVector2D(W.X, W.Y));
					}
				}

				if (OutTrianglesCm.Num() > TrianglesBefore)
				{
					UE_LOG(
						LogBuildingFootprintExporter,
						Verbose,
						TEXT("Nanite MeshDescription OK: mesh='%s' tris=%d"),
						*Mesh->GetName(),
						OutTrianglesCm.Num() - TrianglesBefore);
					return true;
				}
			}
		}
		UE_LOG(
			LogBuildingFootprintExporter,
			Warning,
			TEXT("Nanite mesh '%s' had no usable MeshDescription; falling back to LOD0."),
			*Mesh->GetName());
	}

	const FStaticMeshRenderData* RenderData = Mesh->GetRenderData();
	if (!RenderData || RenderData->LODResources.Num() == 0)
	{
		const FBox Box = Component.Bounds.GetBox();
		const FVector2D P00(Box.Min.X, Box.Min.Y);
		const FVector2D P01(Box.Min.X, Box.Max.Y);
		const FVector2D P10(Box.Max.X, Box.Min.Y);
		const FVector2D P11(Box.Max.X, Box.Max.Y);
		OutGroundPointsCm.Add(P00);
		OutGroundPointsCm.Add(P01);
		OutGroundPointsCm.Add(P10);
		OutGroundPointsCm.Add(P11);
		OutTrianglesCm.Add({P00, P10, P11, Box.Min.Z, Box.Min.Z, Box.Max.Z});
		OutTrianglesCm.Add({P00, P11, P01, Box.Min.Z, Box.Max.Z, Box.Max.Z});
		return true;
	}

	const FStaticMeshLODResources& LOD = RenderData->LODResources[0];
	const FPositionVertexBuffer& VertexBuffer = LOD.VertexBuffers.PositionVertexBuffer;
	const uint32 NumVerts = VertexBuffer.GetNumVertices();
	if (NumVerts == 0)
	{
		return false;
	}

	TArray<FVector> WorldPoints;
	WorldPoints.SetNumUninitialized(static_cast<int32>(NumVerts));
	for (uint32 Index = 0; Index < NumVerts; ++Index)
	{
		WorldPoints[static_cast<int32>(Index)] = Xform.TransformPosition(FVector(VertexBuffer.VertexPosition(Index)));
	}

	const FIndexArrayView Indices = LOD.IndexBuffer.GetArrayView();
	const uint32 NumTriangles = LOD.GetNumTriangles();
	OutTrianglesCm.Reserve(OutTrianglesCm.Num() + static_cast<int32>(NumTriangles));
	for (uint32 Tri = 0; Tri < NumTriangles; ++Tri)
	{
		const uint32 I0 = Indices[Tri * 3 + 0];
		const uint32 I1 = Indices[Tri * 3 + 1];
		const uint32 I2 = Indices[Tri * 3 + 2];
		if (I0 >= NumVerts || I1 >= NumVerts || I2 >= NumVerts)
		{
			continue;
		}

		AddWorldTriangle(
			WorldPoints[static_cast<int32>(I0)],
			WorldPoints[static_cast<int32>(I1)],
			WorldPoints[static_cast<int32>(I2)]);
	}

	const uint32 Stride = FMath::Max<uint32>(1, NumVerts / 8000);
	OutGroundPointsCm.Reserve(OutGroundPointsCm.Num() + static_cast<int32>((NumVerts + Stride - 1) / Stride));
	for (uint32 Index = 0; Index < NumVerts; Index += Stride)
	{
		const FVector& W = WorldPoints[static_cast<int32>(Index)];
		OutGroundPointsCm.Add(FVector2D(W.X, W.Y));
	}

	return OutTrianglesCm.Num() > TrianglesBefore;
}

FFootprintExtractionResult FootprintGeometryUtils::ExtractFootprints(
	UWorld* World,
	const UBuildingFootprintExportSettings& ExportSettings,
	const UBuildingFootprintFilterSettings* Filter,
	const FFootprintGeoTransform& Geo)
{
	const double StartTime = FPlatformTime::Seconds();
	FFootprintExtractionResult Result;

	if (!World)
	{
		Result.ErrorMessage = TEXT("World is null.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingFootprintExporter, Error, TEXT("ExtractFootprints aborted: world is null."));
		return Result;
	}

	// 0 = request finest grid; silhouette clamps to 1 cm (maximum practical accuracy).
	constexpr double HardCodedCellSizeCm = 0.0;

	UE_LOG(
		LogBuildingFootprintExporter,
		Display,
		TEXT("Extract start | map='%s' material='%s' mergeCm=%.1f simplifyCm=%.1f maxGrid=%d cellCm=%.1f(->1) filterAsset=%s"),
		*ExportSettings.MapName,
		*ExportSettings.IncludeMaterialNameContains,
		ExportSettings.ClusterMergeDistanceCm,
		ExportSettings.SilhouetteSimplifyToleranceCm,
		ExportSettings.SilhouetteMaxGridDimension,
		HardCodedCellSizeCm,
		Filter ? *Filter->GetName() : TEXT("(none)"));

	struct FMeshPiece
	{
		TArray<FGroundTriangle2D> TrianglesCm;
		FBox2D Bounds;
		FString ActorLabel;
	};

	auto BoundsFromTriangles = [](const TArray<FGroundTriangle2D>& Tris) -> FBox2D
	{
		FBox2D Box(Tris[0].A, Tris[0].A);
		for (const FGroundTriangle2D& Tri : Tris)
		{
			Box += Tri.A;
			Box += Tri.B;
			Box += Tri.C;
		}
		return Box;
	};

	TArray<FMeshPiece> AllPieces;
	int32 NaniteMeshesUsed = 0;
	int32 Lod0MeshesUsed = 0;
	int32 TotalTriangles = 0;
	int32 ActorsSkippedHiddenOrFilter = 0;

	TArray<AActor*> ActorsToProcess;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor)
		{
			continue;
		}
		if (ActorPassesFilter(*Actor, ExportSettings, Filter))
		{
			ActorsToProcess.Add(Actor);
		}
		else
		{
			++ActorsSkippedHiddenOrFilter;
		}
	}

	UE_LOG(
		LogBuildingFootprintExporter,
		Display,
		TEXT("Actor scan done | toProcess=%d skippedHiddenOrFilter=%d"),
		ActorsToProcess.Num(),
		ActorsSkippedHiddenOrFilter);

	FScopedSlowTask SlowTask(
		static_cast<float>(FMath::Max(ActorsToProcess.Num(), 1)) + 1.0f,
		NSLOCTEXT("BuildingFootprintExporter", "ExportProgress", "Exporting building footprints..."));
	SlowTask.MakeDialog(true);

	for (int32 ActorIndex = 0; ActorIndex < ActorsToProcess.Num(); ++ActorIndex)
	{
		AActor* Actor = ActorsToProcess[ActorIndex];
		const FString ActorLabel = Actor->GetActorNameOrLabel();

		SlowTask.EnterProgressFrame(
			1.0f,
			FText::Format(
				NSLOCTEXT("BuildingFootprintExporter", "ExportProgressActor", "Scan {0}/{1}: {2}  |  Pieces: {3}"),
				FText::AsNumber(ActorIndex + 1),
				FText::AsNumber(ActorsToProcess.Num()),
				FText::FromString(ActorLabel),
				FText::AsNumber(AllPieces.Num())));

		if (SlowTask.ShouldCancel())
		{
			Result.bCancelled = true;
			Result.ErrorMessage = TEXT("Export cancelled by user.");
			UE_LOG(LogBuildingFootprintExporter, Warning, TEXT("Cancelled by user at actor %d/%d ('%s')."), ActorIndex + 1, ActorsToProcess.Num(), *ActorLabel);
			break;
		}

		++Result.ActorsScanned;

		TArray<UStaticMeshComponent*> MeshComponents;
		Actor->GetComponents<UStaticMeshComponent>(MeshComponents);
		if (MeshComponents.Num() == 0)
		{
			UE_LOG(LogBuildingFootprintExporter, Verbose, TEXT("Actor '%s' has no StaticMeshComponents."), *ActorLabel);
			continue;
		}

		int32 ActorAccepted = 0;
		int32 ActorRejected = 0;
		int32 ActorTris = 0;

		for (UStaticMeshComponent* Comp : MeshComponents)
		{
			if (!Comp || !ComponentPassesFilter(*Comp, ExportSettings, Filter))
			{
				++Result.ComponentsRejected;
				++ActorRejected;
				continue;
			}

			UStaticMesh* StaticMesh = Comp->GetStaticMesh();
			const bool bNanite = StaticMesh && StaticMeshUsesNanite(*StaticMesh);

			TArray<FVector2D> PointsCm;
			TArray<FGroundTriangle2D> TrianglesCm;
			if (!CollectComponentGroundGeometry(*Comp, PointsCm, TrianglesCm))
			{
				++Result.ComponentsRejected;
				++ActorRejected;
				UE_LOG(
					LogBuildingFootprintExporter,
					Warning,
					TEXT("Failed to collect geometry | actor='%s' comp='%s' mesh='%s' nanite=%d"),
					*ActorLabel,
					*Comp->GetName(),
					StaticMesh ? *StaticMesh->GetName() : TEXT("(null)"),
					bNanite ? 1 : 0);
				continue;
			}

			++Result.ComponentsAccepted;
			++ActorAccepted;
			if (bNanite)
			{
				++NaniteMeshesUsed;
			}
			else
			{
				++Lod0MeshesUsed;
			}

			FMeshPiece Piece;
			Piece.TrianglesCm = MoveTemp(TrianglesCm);
			Piece.ActorLabel = ActorLabel;
			if (Piece.TrianglesCm.Num() == 0)
			{
				continue;
			}
			ActorTris += Piece.TrianglesCm.Num();
			TotalTriangles += Piece.TrianglesCm.Num();
			Piece.Bounds = BoundsFromTriangles(Piece.TrianglesCm);
			AllPieces.Add(MoveTemp(Piece));
		}

		if (ActorAccepted > 0 || (ActorIndex < 20) || ((ActorIndex + 1) % 25 == 0) || (ActorIndex + 1 == ActorsToProcess.Num()))
		{
			UE_LOG(
				LogBuildingFootprintExporter,
				Display,
				TEXT("Actor %d/%d '%s' | comps accepted=%d rejected=%d tris=%d piecesTotal=%d"),
				ActorIndex + 1,
				ActorsToProcess.Num(),
				*ActorLabel,
				ActorAccepted,
				ActorRejected,
				ActorTris,
				AllPieces.Num());
		}
	}

	UE_LOG(
		LogBuildingFootprintExporter,
		Display,
		TEXT("Geometry collect done | actorsScanned=%d compsAccepted=%d compsRejected=%d pieces=%d tris=%d naniteMeshes=%d lod0Meshes=%d elapsed=%.2fs"),
		Result.ActorsScanned,
		Result.ComponentsAccepted,
		Result.ComponentsRejected,
		AllPieces.Num(),
		TotalTriangles,
		NaniteMeshesUsed,
		Lod0MeshesUsed,
		FPlatformTime::Seconds() - StartTime);

	if (!Result.bCancelled && AllPieces.Num() > 0)
	{
		SlowTask.EnterProgressFrame(
			1.0f,
			FText::Format(
				NSLOCTEXT("BuildingFootprintExporter", "ExportProgressMerge", "Size-limited merge of {0} mesh pieces, then silhouettes..."),
				FText::AsNumber(AllPieces.Num())));

		const int32 PieceCount = AllPieces.Num();
		const double CellCm = FMath::Max(HardCodedCellSizeCm, 1.0);
		const int32 MaxDim = FMath::Max(ExportSettings.SilhouetteMaxGridDimension, 64);
		const double MergeDist = ExportSettings.ClusterMergeDistanceCm;

		auto BoundsFitInMaxGrid = [CellCm, MaxDim](const FBox2D& Box) -> bool
		{
			// Match silhouette padding (~1 cell each side).
			const double Pad = CellCm;
			const int32 Width = FMath::Max(1, FMath::CeilToInt((Box.Max.X - Box.Min.X + 2.0 * Pad) / CellCm));
			const int32 Height = FMath::Max(1, FMath::CeilToInt((Box.Max.Y - Box.Min.Y + 2.0 * Pad) / CellCm));
			return Width <= MaxDim && Height <= MaxDim;
		};

		UE_LOG(
			LogBuildingFootprintExporter,
			Display,
			TEXT("Size-limited clustering | pieces=%d mergeCm=%.1f cellCm=%.1f maxGrid=%d (merge only if combined AABB fits fine grid)"),
			PieceCount,
			MergeDist,
			CellCm,
			MaxDim);

		TArray<int32> Parent;
		TArray<FBox2D> RootBounds;
		Parent.SetNum(PieceCount);
		RootBounds.SetNum(PieceCount);
		for (int32 I = 0; I < PieceCount; ++I)
		{
			Parent[I] = I;
			RootBounds[I] = AllPieces[I].Bounds;
			if (!BoundsFitInMaxGrid(AllPieces[I].Bounds))
			{
				UE_LOG(
					LogBuildingFootprintExporter,
					Warning,
					TEXT("Piece '%s' alone exceeds max grid at %.1f cm cell — its silhouette may coarsen."),
					*AllPieces[I].ActorLabel,
					CellCm);
			}
		}

		// Agglomerative merge: join nearby groups only while the combined AABB still fits MaxGrid @ 1cm.
		// Prevents whole-tile mega-clusters that forced coarse cells and stair footprints.
		bool bMergedAny = true;
		int32 MergePasses = 0;
		int32 MergeCount = 0;
		while (bMergedAny)
		{
			bMergedAny = false;
			++MergePasses;
			for (int32 I = 0; I < PieceCount; ++I)
			{
				for (int32 J = I + 1; J < PieceCount; ++J)
				{
					const int32 RI = FindRoot(Parent, I);
					const int32 RJ = FindRoot(Parent, J);
					if (RI == RJ)
					{
						continue;
					}
					if (!BoundsOverlap2D(RootBounds[RI], RootBounds[RJ], MergeDist))
					{
						continue;
					}

					FBox2D Combined = RootBounds[RI];
					Combined += RootBounds[RJ].Min;
					Combined += RootBounds[RJ].Max;
					if (!BoundsFitInMaxGrid(Combined))
					{
						continue;
					}

					UnionIdx(Parent, RI, RJ);
					const int32 NewRoot = FindRoot(Parent, RI);
					RootBounds[NewRoot] = Combined;
					bMergedAny = true;
					++MergeCount;
				}
			}
		}

		TMap<int32, TArray<int32>> Clusters;
		for (int32 I = 0; I < PieceCount; ++I)
		{
			Clusters.FindOrAdd(FindRoot(Parent, I)).Add(I);
		}

		UE_LOG(
			LogBuildingFootprintExporter,
			Display,
			TEXT("Size-limited clustering done | groups=%d merges=%d passes=%d. Building per-group silhouettes..."),
			Clusters.Num(),
			MergeCount,
			MergePasses);

		int32 ClusterIndex = 0;
		for (const TPair<int32, TArray<int32>>& Pair : Clusters)
		{
			++ClusterIndex;
			if (SlowTask.ShouldCancel())
			{
				Result.bCancelled = true;
				Result.ErrorMessage = TEXT("Export cancelled by user.");
				UE_LOG(LogBuildingFootprintExporter, Warning, TEXT("Cancelled during silhouette at group %d/%d."), ClusterIndex, Clusters.Num());
				break;
			}

			const TArray<int32>& MemberIds = Pair.Value;

			TArray<FGroundTriangle2D> ClusterTris;
			FString MergedLabel = AllPieces[MemberIds[0]].ActorLabel;
			TSet<FString> Labels;
			int32 TotalTris = 0;
			for (const int32 Idx : MemberIds)
			{
				TotalTris += AllPieces[Idx].TrianglesCm.Num();
				Labels.Add(AllPieces[Idx].ActorLabel);
			}
			ClusterTris.Reserve(TotalTris);
			for (const int32 Idx : MemberIds)
			{
				ClusterTris.Append(AllPieces[Idx].TrianglesCm);
			}
			if (Labels.Num() > 1)
			{
				MergedLabel = FString::Join(Labels.Array(), TEXT("+"));
			}

			const double SilhouetteStart = FPlatformTime::Seconds();
			const TArray<FSilhouettePolygon2D> Silhouettes = FootprintSilhouette::BuildSilhouettesFromTriangles(
				ClusterTris,
				HardCodedCellSizeCm,
				ExportSettings.SilhouetteSimplifyToleranceCm,
				ExportSettings.SilhouetteMaxGridDimension,
				100.0);
			const double SilhouetteSeconds = FPlatformTime::Seconds() - SilhouetteStart;

			int32 Added = 0;
			for (const FSilhouettePolygon2D& Silhouette : Silhouettes)
			{
				if (Silhouette.OuterRingCm.Num() < 3)
				{
					continue;
				}
				Result.Footprints.Add(MakeSilhouettePolygon(Silhouette, MergedLabel, Geo));
				++Added;
			}

			if (Added > 0 || ClusterIndex <= 15 || (ClusterIndex % 20 == 0) || ClusterIndex == Clusters.Num())
			{
				const FBox2D& GB = RootBounds[FindRoot(Parent, MemberIds[0])];
				UE_LOG(
					LogBuildingFootprintExporter,
					Display,
					TEXT("Group %d/%d '%s' | members=%d tris=%d aabb=%.1fx%.1fm silhouettes=%d kept=%d time=%.2fs"),
					ClusterIndex,
					Clusters.Num(),
					*MergedLabel,
					MemberIds.Num(),
					ClusterTris.Num(),
					(GB.Max.X - GB.Min.X) / 100.0,
					(GB.Max.Y - GB.Min.Y) / 100.0,
					Silhouettes.Num(),
					Added,
					SilhouetteSeconds);
			}
		}
	}
	else if (!Result.bCancelled)
	{
		UE_LOG(LogBuildingFootprintExporter, Warning, TEXT("No mesh pieces collected — nothing to silhouette."));
	}

	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	UE_LOG(
		LogBuildingFootprintExporter,
		Display,
		TEXT("Extract finished | footprints=%d cancelled=%d error='%s' elapsed=%.2fs"),
		Result.Footprints.Num(),
		Result.bCancelled ? 1 : 0,
		*Result.ErrorMessage,
		Result.ElapsedSeconds);
	return Result;
}
