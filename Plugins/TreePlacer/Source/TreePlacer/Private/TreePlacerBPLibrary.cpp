#include "TreePlacerBPLibrary.h"

#include "TreeCesiumPlacement.h"
#include "TreeMeshFolderLoader.h"
#include "TreePlacerLog.h"
#include "TreeShapefileReader.h"

#include "CesiumGeoreference.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Editor.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "FoliageType.h"
#include "FoliageType_InstancedStaticMesh.h"
#include "HAL/PlatformTime.h"
#include "InstancedFoliage.h"
#include "InstancedFoliageActor.h"
#include "Internationalization/Internationalization.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Materials/MaterialInstanceConstant.h"
#include "Materials/MaterialInterface.h"
#include "MaterialTypes.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
#include "Misc/ScopedSlowTask.h"
#include "UObject/SoftObjectPath.h"

namespace
{
	FString SanitizeFilePath(const FString& InPath)
	{
		FString Path = InPath.TrimStartAndEnd();
		while ((Path.StartsWith(TEXT("\"")) && Path.EndsWith(TEXT("\"")) && Path.Len() >= 2)
			|| (Path.StartsWith(TEXT("'")) && Path.EndsWith(TEXT("'")) && Path.Len() >= 2))
		{
			Path = Path.Mid(1, Path.Len() - 2).TrimStartAndEnd();
		}
		return Path;
	}

	UWorld* ResolveEditorWorld(UObject* WorldContextObject)
	{
		UWorld* World = nullptr;
		if (WorldContextObject)
		{
			World = GEngine ? GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::ReturnNull) : nullptr;
		}
		if (!World && GEditor)
		{
			World = GEditor->GetEditorWorldContext().World();
		}
		return World;
	}

	void ChooseSquareTileGrid(
		double MinLon,
		double MaxLon,
		double MinLat,
		double MaxLat,
		int32 TargetTileCount,
		int32& OutTilesX,
		int32& OutTilesY)
	{
		const int32 Target = FMath::Clamp(TargetTileCount <= 0 ? 64 : TargetTileCount, 1, 4096);
		const double LonSpan = FMath::Max(MaxLon - MinLon, 1.0e-9);
		const double LatSpan = FMath::Max(MaxLat - MinLat, 1.0e-9);
		const double MidLatRad = FMath::DegreesToRadians(0.5 * (MinLat + MaxLat));
		constexpr double MetersPerDegLat = 110540.0;
		const double MetersPerDegLon = FMath::Max(111320.0 * FMath::Cos(MidLatRad), 1.0);
		const double WidthM = LonSpan * MetersPerDegLon;
		const double HeightM = LatSpan * MetersPerDegLat;

		int32 BestX = Target;
		int32 BestY = 1;
		double BestAspectError = TNumericLimits<double>::Max();

		for (int32 Y = 1; Y <= Target; ++Y)
		{
			if (Target % Y != 0)
			{
				continue;
			}
			const int32 X = Target / Y;
			const double CellW = WidthM / static_cast<double>(X);
			const double CellH = HeightM / static_cast<double>(Y);
			const double CellAspect = CellW / FMath::Max(CellH, 1.0);
			const double AspectError = FMath::Abs(FMath::Loge(CellAspect));
			if (AspectError < BestAspectError)
			{
				BestAspectError = AspectError;
				BestX = X;
				BestY = Y;
			}
		}

		OutTilesX = BestX;
		OutTilesY = BestY;
	}

	bool ParseTileIndicesFilter(const FString& TileIndices, TSet<int32>& OutIndices, FString& OutError)
	{
		OutIndices.Reset();
		FString Trimmed = TileIndices.TrimStartAndEnd();
		if (Trimmed.IsEmpty())
		{
			return true;
		}

		TArray<FString> Parts;
		Trimmed.ParseIntoArray(Parts, TEXT(","), /*bCullEmpty*/ true);
		for (FString& Part : Parts)
		{
			Part.TrimStartAndEndInline();
			if (Part.IsEmpty())
			{
				continue;
			}
			int32 Index = INDEX_NONE;
			if (!LexTryParseString(Index, *Part) || Index < 0)
			{
				OutError = FString::Printf(
					TEXT("TileIndices entry '%s' is not a non-negative integer. Use e.g. \"0,6,12\"."),
					*Part);
				return false;
			}
			OutIndices.Add(Index);
		}
		return true;
	}

	/** Convert meters to Unreal cull cm. <= 0 means no distance culling (UE uses 0). */
	int32 MetersToCullDistanceCm(float DistanceMeters)
	{
		if (DistanceMeters <= 0.0f)
		{
			return 0;
		}
		return FMath::Max(FMath::CeilToInt(DistanceMeters * 100.0f), 1);
	}

	void ConfigureHismCullDistance(UHierarchicalInstancedStaticMeshComponent* HISM, int32 EndCullDistanceCm)
	{
		if (!HISM)
		{
			return;
		}
		const int32 EndDist = FMath::Max(EndCullDistanceCm, 0);
		// Fade starts at 35% of the end distance before full cull (min 1 m band).
		const int32 StartDist = (EndDist > 0)
			? FMath::Max(0, EndDist - FMath::Max((EndDist * 35) / 100, 100))
			: 0;
		HISM->InstanceStartCullDistance = StartDist;
		HISM->InstanceEndCullDistance = EndDist;
	}

	void ApplyFoliageCullDistance(
		UFoliageType* FoliageType,
		FFoliageInfo* FoliageInfo,
		int32 TreeCullCm)
	{
		if (!FoliageType)
		{
			return;
		}

		const int32 EndDist = FMath::Max(TreeCullCm, 0);
		const int32 StartDist = (EndDist > 0)
			? FMath::Max(0, EndDist - FMath::Max((EndDist * 35) / 100, 100))
			: 0;
		FoliageType->CullDistance.Min = StartDist;
		FoliageType->CullDistance.Max = EndDist;
		FoliageType->CastShadow = true;
		FoliageType->AlignToNormal = false;
		FoliageType->RandomYaw = false;
		FoliageType->Modify();

		if (FoliageInfo)
		{
			if (UHierarchicalInstancedStaticMeshComponent* HISM =
					Cast<UHierarchicalInstancedStaticMeshComponent>(FoliageInfo->GetComponent()))
			{
				ConfigureHismCullDistance(HISM, TreeCullCm);
				HISM->SetCastShadow(true);
				if (const UFoliageType_InstancedStaticMesh* ISMType =
						Cast<UFoliageType_InstancedStaticMesh>(FoliageType))
				{
					for (int32 Slot = 0; Slot < ISMType->OverrideMaterials.Num(); ++Slot)
					{
						if (ISMType->OverrideMaterials[Slot])
						{
							HISM->SetMaterial(Slot, ISMType->OverrideMaterials[Slot]);
						}
					}
				}
				HISM->MarkRenderStateDirty();
			}
		}
	}

	struct FTreeFoliageSlot
	{
		UFoliageType* Type = nullptr;
		FFoliageInfo* Info = nullptr;
	};

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

	UFoliageType* FindFoliageTypeForMeshAndLeaf(
		AInstancedFoliageActor& IFA,
		const UStaticMesh* Mesh,
		const UMaterialInterface* LeafMI,
		int32 LeafSlotIndex)
	{
		UFoliageType* Found = nullptr;
		IFA.ForEachFoliageInfo([&](UFoliageType* Type, FFoliageInfo& /*Info*/)
		{
			const UFoliageType_InstancedStaticMesh* ISMType = Cast<UFoliageType_InstancedStaticMesh>(Type);
			if (!ISMType || !Mesh || ISMType->GetStaticMesh() != Mesh)
			{
				return true;
			}
			if (!LeafMI)
			{
				return true;
			}
			if (ISMType->OverrideMaterials.IsValidIndex(LeafSlotIndex)
				&& ISMType->OverrideMaterials[LeafSlotIndex] == LeafMI)
			{
				Found = Type;
				return false;
			}
			return true;
		});
		return Found;
	}

	void ApplyLeafOverride(
		UFoliageType_InstancedStaticMesh& Type,
		UStaticMesh* Mesh,
		int32 LeafSlotIndex,
		UMaterialInterface* LeafMI)
	{
		if (!Mesh || !LeafMI)
		{
			return;
		}

		const int32 NumSlots = Mesh->GetStaticMaterials().Num();
		Type.OverrideMaterials.SetNum(FMath::Max(NumSlots, 0));
		for (int32 I = 0; I < NumSlots; ++I)
		{
			Type.OverrideMaterials[I] = Mesh->GetMaterial(I);
		}

		const int32 Slot = (NumSlots > 0)
			? FMath::Clamp(LeafSlotIndex, 0, NumSlots - 1)
			: FMath::Max(LeafSlotIndex, 0);
		if (!Type.OverrideMaterials.IsValidIndex(Slot))
		{
			Type.OverrideMaterials.SetNum(Slot + 1);
		}
		Type.OverrideMaterials[Slot] = LeafMI;
	}

	bool GetOrCreateFoliageSlot(
		AInstancedFoliageActor& IFA,
		UStaticMesh* Mesh,
		int32 TreeCullCm,
		FTreeFoliageSlot& OutSlot,
		FString& OutError,
		UMaterialInterface* LeafOverride = nullptr,
		int32 LeafSlotIndex = 1)
	{
		OutSlot = FTreeFoliageSlot();
		if (!Mesh)
		{
			OutError = TEXT("Null tree mesh.");
			return false;
		}

		UFoliageType* Type = nullptr;
		FFoliageInfo* Info = nullptr;

		if (LeafOverride)
		{
			Type = FindFoliageTypeForMeshAndLeaf(IFA, Mesh, LeafOverride, LeafSlotIndex);
			Info = Type ? IFA.FindInfo(Type) : nullptr;
			if (!Type || !Info)
			{
				UFoliageType_InstancedStaticMesh* NewType =
					NewObject<UFoliageType_InstancedStaticMesh>(&IFA, NAME_None, RF_Transactional);
				NewType->SetStaticMesh(Mesh);
				ApplyLeafOverride(*NewType, Mesh, LeafSlotIndex, LeafOverride);
				Type = IFA.AddFoliageType(NewType, &Info);
				if (Type && !Info)
				{
					Info = IFA.FindInfo(Type);
				}
				if (UFoliageType_InstancedStaticMesh* AddedISM = Cast<UFoliageType_InstancedStaticMesh>(Type))
				{
					ApplyLeafOverride(*AddedISM, Mesh, LeafSlotIndex, LeafOverride);
					AddedISM->Modify();
				}
			}
		}
		else
		{
			Type = FindFoliageTypeForMesh(IFA, Mesh);
			Info = Type ? IFA.FindInfo(Type) : nullptr;
			if (!Type || !Info)
			{
				Type = nullptr;
				Info = IFA.AddMesh(Mesh, &Type);
			}
		}

		if (!Type || !Info)
		{
			OutError = FString::Printf(TEXT("Failed to add foliage type for mesh '%s'."), *Mesh->GetName());
			return false;
		}

		ApplyFoliageCullDistance(Type, Info, TreeCullCm);
		OutSlot.Type = Type;
		OutSlot.Info = Info;
		return true;
	}

	FFoliageInstance MakeFoliageInstance(const FTransform& Xform)
	{
		FFoliageInstance Instance;
		Instance.Location = Xform.GetLocation();
		Instance.Rotation = Xform.Rotator();
		Instance.DrawScale3D = FVector3f(Xform.GetScale3D());
		return Instance;
	}

	bool AddFoliageInstances(
		FTreeFoliageSlot& Slot,
		const TArray<FTransform>& WorldTransforms)
	{
		if (!Slot.Type || !Slot.Info || WorldTransforms.Num() == 0)
		{
			return WorldTransforms.Num() == 0;
		}

		for (const FTransform& Xform : WorldTransforms)
		{
			Slot.Info->AddInstance(Slot.Type, MakeFoliageInstance(Xform));
		}
		return true;
	}

	void RefreshFoliageSlots(TArray<FTreeFoliageSlot>& Slots, int32 TreeCullCm)
	{
		for (FTreeFoliageSlot& Slot : Slots)
		{
			if (!Slot.Info)
			{
				continue;
			}
			Slot.Info->Refresh(/*Async*/ false, /*Force*/ true);
			ApplyFoliageCullDistance(Slot.Type, Slot.Info, TreeCullCm);
		}
	}

	struct FLabColor
	{
		double L = 0.0;
		double A = 0.0;
		double B = 0.0;
	};

	double LabF(double T)
	{
		constexpr double Delta = 6.0 / 29.0;
		constexpr double Delta3 = Delta * Delta * Delta;
		if (T > Delta3)
		{
			return FMath::Pow(T, 1.0 / 3.0);
		}
		return T / (3.0 * Delta * Delta) + 4.0 / 29.0;
	}

	double LabFInv(double T)
	{
		constexpr double Delta = 6.0 / 29.0;
		if (T > Delta)
		{
			return T * T * T;
		}
		return 3.0 * Delta * Delta * (T - 4.0 / 29.0);
	}

	FLabColor LinearToLab(const FLinearColor& Color)
	{
		const double R = FMath::Max(static_cast<double>(Color.R), 0.0);
		const double G = FMath::Max(static_cast<double>(Color.G), 0.0);
		const double B = FMath::Max(static_cast<double>(Color.B), 0.0);
		const double X = 0.4124564 * R + 0.3575761 * G + 0.1804375 * B;
		const double Y = 0.2126729 * R + 0.7151522 * G + 0.0721750 * B;
		const double Z = 0.0193339 * R + 0.1191920 * G + 0.9503041 * B;
		const double Fx = LabF(X / 0.95047);
		const double Fy = LabF(Y / 1.00000);
		const double Fz = LabF(Z / 1.08883);
		FLabColor Lab;
		Lab.L = 116.0 * Fy - 16.0;
		Lab.A = 500.0 * (Fx - Fy);
		Lab.B = 200.0 * (Fy - Fz);
		return Lab;
	}

	FLinearColor LabToLinear(const FLabColor& Lab)
	{
		const double Fy = (Lab.L + 16.0) / 116.0;
		const double Fx = Fy + Lab.A / 500.0;
		const double Fz = Fy - Lab.B / 200.0;
		const double X = 0.95047 * LabFInv(Fx);
		const double Y = 1.00000 * LabFInv(Fy);
		const double Z = 1.08883 * LabFInv(Fz);
		FLinearColor Color;
		Color.R = static_cast<float>(3.2404542 * X - 1.5371385 * Y - 0.4985314 * Z);
		Color.G = static_cast<float>(-0.9692660 * X + 1.8760108 * Y + 0.0415560 * Z);
		Color.B = static_cast<float>(0.0556434 * X - 0.2040259 * Y + 1.0572252 * Z);
		Color.A = 1.0f;
		Color.R = FMath::Clamp(Color.R, 0.0f, 1.0f);
		Color.G = FMath::Clamp(Color.G, 0.0f, 1.0f);
		Color.B = FMath::Clamp(Color.B, 0.0f, 1.0f);
		return Color;
	}

	FLabColor Srgb8ToLab(uint8 R, uint8 G, uint8 B)
	{
		return LinearToLab(FLinearColor::FromSRGBColor(FColor(R, G, B, 255)));
	}

	double LabDistSq(const FLabColor& A, const FLabColor& B)
	{
		const double DL = A.L - B.L;
		const double DA = A.A - B.A;
		const double DB = A.B - B.B;
		return DL * DL + DA * DA + DB * DB;
	}

	int32 NearestCentroid(const FLabColor& Sample, const TArray<FLabColor>& Centroids)
	{
		int32 Best = 0;
		double BestDist = LabDistSq(Sample, Centroids[0]);
		for (int32 I = 1; I < Centroids.Num(); ++I)
		{
			const double Dist = LabDistSq(Sample, Centroids[I]);
			if (Dist < BestDist)
			{
				BestDist = Dist;
				Best = I;
			}
		}
		return Best;
	}

	bool ClusterLeafColors(
		const TArray<FTreeShapefilePoint>& Points,
		int32 RequestedK,
		FRandomStream& Rng,
		TArray<int32>& OutClusterOfPoint,
		TArray<FLinearColor>& OutPalette,
		int32& OutDefaultCluster,
		FString& OutError)
	{
		OutClusterOfPoint.SetNum(Points.Num());
		for (int32& Id : OutClusterOfPoint)
		{
			Id = INDEX_NONE;
		}
		OutPalette.Reset();
		OutDefaultCluster = 0;

		TArray<int32> RgbIndices;
		RgbIndices.Reserve(Points.Num());
		for (int32 I = 0; I < Points.Num(); ++I)
		{
			if (Points[I].bHasRgb)
			{
				RgbIndices.Add(I);
			}
		}
		if (RgbIndices.Num() == 0)
		{
			OutError = TEXT("Leaf tint is enabled but no points have valid R/G/B values.");
			return false;
		}

		const int32 SampleCount = FMath::Min(RgbIndices.Num(), 10000);
		TArray<FLabColor> Samples;
		Samples.Reserve(SampleCount);
		for (int32 I = 0; I < SampleCount; ++I)
		{
			const int32 J = Rng.RandRange(I, RgbIndices.Num() - 1);
			RgbIndices.Swap(I, J);
			const FTreeShapefilePoint& Point = Points[RgbIndices[I]];
			Samples.Add(Srgb8ToLab(Point.R, Point.G, Point.B));
		}

		int32 K = FMath::Clamp(RequestedK, 1, 64);
		K = FMath::Min(K, Samples.Num());

		TArray<FLabColor> Centroids;
		Centroids.Reserve(K);
		Centroids.Add(Samples[Rng.RandRange(0, Samples.Num() - 1)]);
		TArray<double> MinDistSq;
		MinDistSq.SetNum(Samples.Num());
		while (Centroids.Num() < K)
		{
			double DistSum = 0.0;
			for (int32 I = 0; I < Samples.Num(); ++I)
			{
				MinDistSq[I] = LabDistSq(Samples[I], Centroids[0]);
				for (int32 C = 1; C < Centroids.Num(); ++C)
				{
					MinDistSq[I] = FMath::Min(MinDistSq[I], LabDistSq(Samples[I], Centroids[C]));
				}
				DistSum += MinDistSq[I];
			}
			if (DistSum <= KINDA_SMALL_NUMBER)
			{
				break;
			}
			double Pick = Rng.FRandRange(0.0, DistSum);
			int32 Chosen = Samples.Num() - 1;
			for (int32 I = 0; I < Samples.Num(); ++I)
			{
				Pick -= MinDistSq[I];
				if (Pick <= 0.0)
				{
					Chosen = I;
					break;
				}
			}
			Centroids.Add(Samples[Chosen]);
		}
		K = Centroids.Num();

		TArray<int32> Assignment;
		Assignment.SetNum(Samples.Num());
		for (int32 Iter = 0; Iter < 32; ++Iter)
		{
			for (int32 I = 0; I < Samples.Num(); ++I)
			{
				Assignment[I] = NearestCentroid(Samples[I], Centroids);
			}

			TArray<FLabColor> Sums;
			TArray<int32> Counts;
			Sums.SetNum(K);
			Counts.Init(0, K);
			for (int32 I = 0; I < Samples.Num(); ++I)
			{
				const int32 C = Assignment[I];
				Sums[C].L += Samples[I].L;
				Sums[C].A += Samples[I].A;
				Sums[C].B += Samples[I].B;
				++Counts[C];
			}

			double MaxMoveSq = 0.0;
			for (int32 C = 0; C < K; ++C)
			{
				FLabColor NewCentroid = Centroids[C];
				if (Counts[C] > 0)
				{
					const double Inv = 1.0 / static_cast<double>(Counts[C]);
					NewCentroid.L = Sums[C].L * Inv;
					NewCentroid.A = Sums[C].A * Inv;
					NewCentroid.B = Sums[C].B * Inv;
				}
				else
				{
					NewCentroid = Samples[Rng.RandRange(0, Samples.Num() - 1)];
				}
				MaxMoveSq = FMath::Max(MaxMoveSq, LabDistSq(Centroids[C], NewCentroid));
				Centroids[C] = NewCentroid;
			}
			if (MaxMoveSq < 1.0e-4)
			{
				break;
			}
		}

		TArray<int32> ClusterCounts;
		ClusterCounts.Init(0, K);
		for (int32 I = 0; I < Points.Num(); ++I)
		{
			if (!Points[I].bHasRgb)
			{
				continue;
			}
			const FLabColor Lab = Srgb8ToLab(Points[I].R, Points[I].G, Points[I].B);
			const int32 Cluster = NearestCentroid(Lab, Centroids);
			OutClusterOfPoint[I] = Cluster;
			++ClusterCounts[Cluster];
		}

		OutDefaultCluster = 0;
		for (int32 C = 1; C < K; ++C)
		{
			if (ClusterCounts[C] > ClusterCounts[OutDefaultCluster])
			{
				OutDefaultCluster = C;
			}
		}
		for (int32 I = 0; I < Points.Num(); ++I)
		{
			if (OutClusterOfPoint[I] == INDEX_NONE)
			{
				OutClusterOfPoint[I] = OutDefaultCluster;
			}
		}

		OutPalette.SetNum(K);
		for (int32 C = 0; C < K; ++C)
		{
			OutPalette[C] = LabToLinear(Centroids[C]);
		}

		UE_LOG(
			LogTreePlacer,
			Display,
			TEXT("Leaf tint k-means: rgbPoints=%d samples=%d K=%d defaultCluster=%d (missing RGB -> largest cluster)"),
			RgbIndices.Num(),
			SampleCount,
			K,
			OutDefaultCluster);
		for (int32 C = 0; C < K; ++C)
		{
			const FColor Srgb = OutPalette[C].ToFColor(true);
			UE_LOG(
				LogTreePlacer,
				Display,
				TEXT("  cluster %d: sRGB(%d,%d,%d) count=%d"),
				C,
				Srgb.R,
				Srgb.G,
				Srgb.B,
				ClusterCounts[C]);
		}

		return true;
	}

	FString NormalizeLeafTintObjectPath(const FString& InPath)
	{
		FString Path = SanitizeFilePath(InPath);
		Path.ReplaceInline(TEXT("\\"), TEXT("/"));

		// Content Browser "Copy Reference": MaterialInstanceConstant'/Plugin/Path/Asset.Asset'
		const int32 QuoteStart = Path.Find(TEXT("'"));
		const int32 QuoteEnd = Path.Find(TEXT("'"), ESearchCase::IgnoreCase, ESearchDir::FromEnd);
		if (QuoteStart != INDEX_NONE && QuoteEnd > QuoteStart)
		{
			Path = Path.Mid(QuoteStart + 1, QuoteEnd - QuoteStart - 1);
		}

		if (Path.StartsWith(TEXT("/All/")))
		{
			Path = Path.RightChop(4);
		}
		else if (Path.StartsWith(TEXT("All/")))
		{
			Path = TEXT("/") + Path.RightChop(4);
		}

		if (Path.EndsWith(TEXT(".uasset"), ESearchCase::IgnoreCase))
		{
			Path.LeftChopInline(7);
		}
		else if (Path.EndsWith(TEXT(".umap"), ESearchCase::IgnoreCase))
		{
			Path.LeftChopInline(5);
		}

		if (Path.Len() >= 2 && Path[1] == TEXT(':'))
		{
			FString PackageName;
			if (FPackageName::TryConvertFilenameToLongPackageName(Path, PackageName))
			{
				Path = PackageName;
			}
		}

		Path.RemoveFromEnd(TEXT("/"));
		return Path;
	}

	UMaterialInterface* TryLoadMaterialAtPath(const FString& ObjectPath)
	{
		if (ObjectPath.IsEmpty())
		{
			return nullptr;
		}
		if (UMaterialInterface* Mat = LoadObject<UMaterialInterface>(nullptr, *ObjectPath))
		{
			return Mat;
		}
		return Cast<UMaterialInterface>(FSoftObjectPath(ObjectPath).TryLoad());
	}

	UMaterialInterface* LoadLeafTintMaterial(const FString& InPath, FString& OutError)
	{
		const FString Path = NormalizeLeafTintObjectPath(InPath);
		if (Path.IsEmpty())
		{
			OutError = TEXT("LeafTintMaterialPath is empty.");
			return nullptr;
		}

		TArray<FString> Candidates;
		Candidates.Add(Path);
		const FString PackagePath = Path.Contains(TEXT("."))
			? FPackageName::ObjectPathToPackageName(Path)
			: Path;
		const FString ObjectPath = PackagePath + TEXT(".") + FPackageName::GetShortName(PackagePath);
		Candidates.AddUnique(PackagePath);
		Candidates.AddUnique(ObjectPath);

		for (const FString& Candidate : Candidates)
		{
			if (UMaterialInterface* Mat = TryLoadMaterialAtPath(Candidate))
			{
				UE_LOG(LogTreePlacer, Display, TEXT("Loaded leaf tint material '%s'."), *Mat->GetPathName());
				return Mat;
			}
		}

		IAssetRegistry& AssetRegistry =
			FModuleManager::LoadModuleChecked<FAssetRegistryModule>(TEXT("AssetRegistry")).Get();
		TArray<FAssetData> Assets;
		AssetRegistry.GetAssetsByPackageName(*PackagePath, Assets);
		for (const FAssetData& Asset : Assets)
		{
			if (UMaterialInterface* Mat = Cast<UMaterialInterface>(Asset.GetAsset()))
			{
				UE_LOG(LogTreePlacer, Display, TEXT("Loaded leaf tint material '%s'."), *Mat->GetPathName());
				return Mat;
			}
		}

		OutError = FString::Printf(
			TEXT("Could not load leaf tint material '%s' (normalized '%s'). Use a Content path such as /Plugin/Folder/MI_Leaves, with or without .uasset."),
			*InPath,
			*Path);
		return nullptr;
	}

	bool CreateLeafTintInstances(
		AInstancedFoliageActor& IFA,
		UMaterialInterface* Master,
		const FName ParameterName,
		const TArray<FLinearColor>& Palette,
		TArray<UMaterialInterface*>& OutMIs,
		FString& OutError)
	{
		OutMIs.Reset();
		if (!Master)
		{
			OutError = TEXT("Leaf tint master material is null.");
			return false;
		}

		TArray<FMaterialParameterInfo> ParamInfos;
		TArray<FGuid> ParamIds;
		Master->GetAllVectorParameterInfo(ParamInfos, ParamIds);
		bool bFoundParam = false;
		for (const FMaterialParameterInfo& Info : ParamInfos)
		{
			if (Info.Name == ParameterName)
			{
				bFoundParam = true;
				break;
			}
		}
		if (!bFoundParam)
		{
			FString FoundNames;
			const int32 MaxListed = 16;
			for (int32 I = 0; I < ParamInfos.Num() && I < MaxListed; ++I)
			{
				if (!FoundNames.IsEmpty())
				{
					FoundNames += TEXT(", ");
				}
				FoundNames += ParamInfos[I].Name.ToString();
			}
			UE_LOG(
				LogTreePlacer,
				Warning,
				TEXT("Leaf tint parameter '%s' was not found on '%s'. Vector params: %s%s"),
				*ParameterName.ToString(),
				*Master->GetName(),
				FoundNames.IsEmpty() ? TEXT("(none)") : *FoundNames,
				ParamInfos.Num() > MaxListed ? TEXT(", ...") : TEXT(""));
		}

		OutMIs.Reserve(Palette.Num());
		for (int32 I = 0; I < Palette.Num(); ++I)
		{
			const FName MicName(*FString::Printf(TEXT("TP_LeafTint_%d"), I));
			UMaterialInstanceConstant* MIC = FindObject<UMaterialInstanceConstant>(&IFA, *MicName.ToString());
			if (!MIC)
			{
				MIC = NewObject<UMaterialInstanceConstant>(&IFA, MicName, RF_Public | RF_Transactional);
			}
			if (!MIC)
			{
				OutError = FString::Printf(TEXT("Failed to create leaf tint material instance %d."), I);
				return false;
			}

			MIC->SetParentEditorOnly(Master);
			MIC->SetVectorParameterValueEditorOnly(ParameterName, Palette[I]);
			MIC->PostEditChange();
			MIC->MarkPackageDirty();
			OutMIs.Add(MIC);
		}
		return true;
	}
}

FTreePlaceResult UTreePlacerBPLibrary::PlaceTreesFromShapefile(
	UObject* WorldContextObject,
	const FString& ShapefilePath,
	const FString& TreeMeshFolder,
	const FString& AltitudeFieldName,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath,
	int32 TargetTileCount,
	const FString& TileIndices,
	int32 RandomSeed,
	float TreeCullDistanceMeters,
	const FString& RedFieldName,
	const FString& GreenFieldName,
	const FString& BlueFieldName,
	const FString& LeafTintMaterialPath,
	const FString& LeafTintParameterName,
	int32 LeafMaterialSlotIndex,
	int32 ColorClusterCount)
{
	FTreePlaceResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString AltitudeField = AltitudeFieldName.IsEmpty() ? TEXT("altitude") : AltitudeFieldName;
	const FString CleanInputPath = SanitizeFilePath(ShapefilePath);
	const FString CleanTintPath = SanitizeFilePath(LeafTintMaterialPath);
	const bool bTintLeaves = !CleanTintPath.IsEmpty();
	const FString RedField = RedFieldName.IsEmpty() ? TEXT("R") : RedFieldName;
	const FString GreenField = GreenFieldName.IsEmpty() ? TEXT("G") : GreenFieldName;
	const FString BlueField = BlueFieldName.IsEmpty() ? TEXT("B") : BlueFieldName;
	const FName TintParamName = LeafTintParameterName.IsEmpty()
		? FName(TEXT("LeafTint"))
		: FName(*LeafTintParameterName);
	const int32 RequestedK = FMath::Clamp(ColorClusterCount <= 0 ? 8 : ColorClusterCount, 1, 64);

	UE_LOG(LogTreePlacer, Display, TEXT("========== Tree Place START =========="));
	UE_LOG(
		LogTreePlacer,
		Display,
		TEXT("shp='%s' meshes='%s' altitudeField='%s' targetTiles=%d tileFilter='%s' seed=%d treeCullM=%.1f tint='%s' K=%d"),
		*CleanInputPath,
		*TreeMeshFolder,
		*AltitudeField,
		TargetTileCount,
		*TileIndices,
		RandomSeed,
		TreeCullDistanceMeters,
		bTintLeaves ? *CleanTintPath : TEXT("(off)"),
		RequestedK);

	UWorld* World = ResolveEditorWorld(WorldContextObject);
	if (!World)
	{
		Result.Message = TEXT("Could not resolve an editor world. Open a map first.");
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	ACesiumGeoreference* Georeference = TreeCesiumPlacement::FindGeoreference(World);
	if (!Georeference)
	{
		Result.Message = TEXT("No ACesiumGeoreference found in the level.");
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (CleanInputPath.IsEmpty())
	{
		Result.Message = TEXT("ShapefilePath is empty (provide a .shp path).");
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<UStaticMesh*> TreeMeshes;
	FString MeshError;
	if (!TreeMeshFolderLoader::LoadTreeMeshesFromFolder(TreeMeshFolder, TreeMeshes, MeshError))
	{
		Result.Message = MeshError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<FTreeShapefilePoint> Points;
	FString ReadError;
	const FString RgbRed = bTintLeaves ? RedField : FString();
	const FString RgbGreen = bTintLeaves ? GreenField : FString();
	const FString RgbBlue = bTintLeaves ? BlueField : FString();
	if (!TreeShapefileReader::ReadPoints(CleanInputPath, AltitudeField, Points, ReadError, RgbRed, RgbGreen, RgbBlue))
	{
		Result.Message = ReadError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	const int32 PointCount = Points.Num();
	double MinLon = TNumericLimits<double>::Max();
	double MaxLon = TNumericLimits<double>::Lowest();
	double MinLat = TNumericLimits<double>::Max();
	double MaxLat = TNumericLimits<double>::Lowest();
	for (const FTreeShapefilePoint& Point : Points)
	{
		MinLon = FMath::Min(MinLon, Point.LonDeg);
		MaxLon = FMath::Max(MaxLon, Point.LonDeg);
		MinLat = FMath::Min(MinLat, Point.LatDeg);
		MaxLat = FMath::Max(MaxLat, Point.LatDeg);
	}
	const double LonSpan = FMath::Max(MaxLon - MinLon, 1.0e-9);
	const double LatSpan = FMath::Max(MaxLat - MinLat, 1.0e-9);

	int32 TilesX = 1;
	int32 TilesY = 1;
	ChooseSquareTileGrid(MinLon, MaxLon, MinLat, MaxLat, TargetTileCount, TilesX, TilesY);

	TSet<int32> SelectedTileIndices;
	FString TileFilterError;
	if (!ParseTileIndicesFilter(TileIndices, SelectedTileIndices, TileFilterError))
	{
		Result.Message = TileFilterError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	const int32 TileSlotCount = TilesX * TilesY;
	for (const int32 Idx : SelectedTileIndices)
	{
		if (Idx >= TileSlotCount)
		{
			Result.Message = FString::Printf(
				TEXT("Tile index %d is out of range for grid %dx%d (%d slots, indices 0..%d)."),
				Idx,
				TilesX,
				TilesY,
				TileSlotCount,
				TileSlotCount - 1);
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
			return Result;
		}
	}

	TArray<TArray<int32>> TilePointIndices;
	TilePointIndices.SetNum(TileSlotCount);
	for (int32 I = 0; I < PointCount; ++I)
	{
		const FTreeShapefilePoint& Point = Points[I];
		int32 TX = FMath::FloorToInt(static_cast<float>((Point.LonDeg - MinLon) / LonSpan * TilesX));
		int32 TY = FMath::FloorToInt(static_cast<float>((Point.LatDeg - MinLat) / LatSpan * TilesY));
		TX = FMath::Clamp(TX, 0, TilesX - 1);
		TY = FMath::Clamp(TY, 0, TilesY - 1);
		const int32 LinearIndex = TY * TilesX + TX;
		if (SelectedTileIndices.Num() > 0 && !SelectedTileIndices.Contains(LinearIndex))
		{
			continue;
		}
		TilePointIndices[LinearIndex].Add(I);
	}

	int32 NonEmptyTiles = 0;
	int32 PointsInSelectedTiles = 0;
	for (const TArray<int32>& Bucket : TilePointIndices)
	{
		if (Bucket.Num() > 0)
		{
			++NonEmptyTiles;
			PointsInSelectedTiles += Bucket.Num();
		}
	}

	UE_LOG(
		LogTreePlacer,
		Display,
		TEXT("Tiling: points=%d target=%d chosenGrid=%dx%d (%d slots) processingNonEmpty=%d pointsInTiles=%d"),
		PointCount,
		TargetTileCount,
		TilesX,
		TilesY,
		TileSlotCount,
		NonEmptyTiles,
		PointsInSelectedTiles);

	if (NonEmptyTiles == 0)
	{
		Result.Message = SelectedTileIndices.Num() > 0
			? TEXT("No points found in the selected TileIndices.")
			: TEXT("No points assigned to any tile.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	int32 EffectiveSeed = RandomSeed;
	if (EffectiveSeed == 0)
	{
		EffectiveSeed = FMath::Rand();
		if (EffectiveSeed == 0)
		{
			EffectiveSeed = 1;
		}
	}
	FRandomStream PlaceRng;
	FRandomStream ClusterRng;
	PlaceRng.Initialize(EffectiveSeed);
	ClusterRng.Initialize(EffectiveSeed);

	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("TreeTile") : ActorLabelPrefix;
	const FString FolderPath = EditorFolderPath;

	AInstancedFoliageActor* IFA = AInstancedFoliageActor::GetInstancedFoliageActorForCurrentLevel(World, /*bCreateIfNone*/ true);
	if (!IFA)
	{
		Result.Message = TEXT("Could not get or create the level InstancedFoliageActor.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	IFA->Modify();
	if (!FolderPath.IsEmpty())
	{
		IFA->SetFolderPath(FName(*FolderPath));
	}

	const int32 TreeCullCm = MetersToCullDistanceCm(TreeCullDistanceMeters);

	TArray<int32> ClusterOfPoint;
	TArray<FLinearColor> Palette;
	TArray<UMaterialInterface*> LeafMIs;
	int32 DefaultCluster = 0;
	int32 PaletteK = 1;
	if (bTintLeaves)
	{
		UMaterialInterface* Master = nullptr;
		FString TintError;
		Master = LoadLeafTintMaterial(CleanTintPath, TintError);
		if (!Master)
		{
			Result.Message = TintError;
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
			return Result;
		}

		if (!ClusterLeafColors(Points, RequestedK, ClusterRng, ClusterOfPoint, Palette, DefaultCluster, TintError))
		{
			Result.Message = TintError;
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
			return Result;
		}

		if (!CreateLeafTintInstances(*IFA, Master, TintParamName, Palette, LeafMIs, TintError))
		{
			Result.Message = TintError;
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
			return Result;
		}
		PaletteK = LeafMIs.Num();
	}

	const int32 SlotCount = bTintLeaves ? (TreeMeshes.Num() * PaletteK) : TreeMeshes.Num();
	TArray<FTreeFoliageSlot> FoliageSlots;
	FoliageSlots.SetNum(SlotCount);

	if (!bTintLeaves)
	{
		for (int32 MeshIndex = 0; MeshIndex < TreeMeshes.Num(); ++MeshIndex)
		{
			FString SlotError;
			if (!GetOrCreateFoliageSlot(*IFA, TreeMeshes[MeshIndex], TreeCullCm, FoliageSlots[MeshIndex], SlotError))
			{
				Result.Message = SlotError;
				Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
				UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
				return Result;
			}
		}
	}

	FScopedSlowTask SlowTask(
		static_cast<float>(NonEmptyTiles),
		NSLOCTEXT("TreePlacer", "PlaceProgress", "Placing trees on InstancedFoliageActor..."));
	SlowTask.MakeDialog(true);

	int32 TreesPlaced = 0;
	int32 TreesSkipped = 0;
	int32 TilesSpawned = 0;

	for (int32 TileY = 0; TileY < TilesY; ++TileY)
	{
		for (int32 TileX = 0; TileX < TilesX; ++TileX)
		{
			const TArray<int32>& Bucket = TilePointIndices[TileY * TilesX + TileX];
			if (Bucket.Num() == 0)
			{
				continue;
			}

			const FString TileLabel = FString::Printf(TEXT("%s_%d_%d"), *LabelPrefix, TileX, TileY);
			SlowTask.EnterProgressFrame(
				1.0f,
				FText::Format(
					NSLOCTEXT("TreePlacer", "PlaceProgressTile", "Tile {0} ({1} trees)"),
					FText::FromString(TileLabel),
					FText::AsNumber(Bucket.Num())));

			if (SlowTask.ShouldCancel())
			{
				RefreshFoliageSlots(FoliageSlots, TreeCullCm);
				IFA->Modify();
				World->MarkPackageDirty();
				Result.bCancelled = true;
				Result.TreesPlaced = TreesPlaced;
				Result.TreesSkipped = TreesSkipped;
				Result.TilesSpawned = TilesSpawned;
				Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
				Result.Message = FString::Printf(
					TEXT("Cancelled. Tiles=%d trees=%d on InstancedFoliageActor. Elapsed: %.2fs."),
					TilesSpawned,
					TreesPlaced,
					Result.ElapsedSeconds);
				UE_LOG(LogTreePlacer, Warning, TEXT("%s"), *Result.Message);
				return Result;
			}

			TArray<TArray<FTransform>> TransformsPerSlot;
			TransformsPerSlot.SetNum(SlotCount);

			for (const int32 PointIndex : Bucket)
			{
				const FTreeShapefilePoint& Point = Points[PointIndex];
				const FVector WorldPos = TreeCesiumPlacement::LonLatHeightToUnreal(
					*Georeference,
					Point.LonDeg,
					Point.LatDeg,
					Point.AltitudeM);

				const int32 MeshIndex = PlaceRng.RandRange(0, TreeMeshes.Num() - 1);
				const int32 ClusterIndex = bTintLeaves
					? FMath::Clamp(ClusterOfPoint[PointIndex], 0, PaletteK - 1)
					: 0;
				const int32 SlotIndex = bTintLeaves
					? (MeshIndex * PaletteK + ClusterIndex)
					: MeshIndex;
				const float YawDeg = PlaceRng.FRandRange(0.0f, 360.0f);
				TransformsPerSlot[SlotIndex].Add(FTransform(
					FRotator(0.0, YawDeg, 0.0),
					WorldPos,
					FVector::OneVector));
			}

			bool bTileAdded = false;
			for (int32 SlotIndex = 0; SlotIndex < SlotCount; ++SlotIndex)
			{
				const TArray<FTransform>& Transforms = TransformsPerSlot[SlotIndex];
				if (Transforms.Num() == 0)
				{
					continue;
				}

				if (!FoliageSlots[SlotIndex].Type)
				{
					const int32 MeshIndex = bTintLeaves ? (SlotIndex / PaletteK) : SlotIndex;
					UMaterialInterface* LeafMI = bTintLeaves ? LeafMIs[SlotIndex % PaletteK] : nullptr;
					FString SlotError;
					if (!GetOrCreateFoliageSlot(
							*IFA,
							TreeMeshes[MeshIndex],
							TreeCullCm,
							FoliageSlots[SlotIndex],
							SlotError,
							LeafMI,
							LeafMaterialSlotIndex))
					{
						UE_LOG(LogTreePlacer, Error, TEXT("Tile %s: %s"), *TileLabel, *SlotError);
						TreesSkipped += Transforms.Num();
						continue;
					}
				}

				if (!AddFoliageInstances(FoliageSlots[SlotIndex], Transforms))
				{
					const int32 MeshIndex = bTintLeaves ? (SlotIndex / PaletteK) : SlotIndex;
					UE_LOG(LogTreePlacer, Error, TEXT("Tile %s failed to add instances for mesh '%s'."),
						*TileLabel,
						*TreeMeshes[MeshIndex]->GetName());
					TreesSkipped += Transforms.Num();
					continue;
				}
				TreesPlaced += Transforms.Num();
				bTileAdded = true;
			}

			if (bTileAdded)
			{
				++TilesSpawned;
			}
		}
	}

	RefreshFoliageSlots(FoliageSlots, TreeCullCm);
	IFA->Modify();
	World->MarkPackageDirty();

	if (TilesSpawned <= 0)
	{
		Result.TreesPlaced = 0;
		Result.TreesSkipped = TreesSkipped;
		Result.TilesSpawned = 0;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(
			TEXT("No trees added to InstancedFoliageActor (%d skipped). Elapsed: %.2fs."),
			TreesSkipped,
			Result.ElapsedSeconds);
		UE_LOG(LogTreePlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	Result.bSuccess = true;
	Result.TreesPlaced = TreesPlaced;
	Result.TreesSkipped = TreesSkipped;
	Result.TilesSpawned = TilesSpawned;
	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	if (bTintLeaves)
	{
		Result.Message = FString::Printf(
			TEXT("Added %d trees (%d skipped) from %d tiles onto InstancedFoliageActor using %d meshes x %d leaf colors from '%s'. Elapsed: %.2fs."),
			TreesPlaced,
			TreesSkipped,
			TilesSpawned,
			TreeMeshes.Num(),
			PaletteK,
			*TreeMeshFolder,
			Result.ElapsedSeconds);
	}
	else
	{
		Result.Message = FString::Printf(
			TEXT("Added %d trees (%d skipped) from %d tiles onto InstancedFoliageActor using %d mesh types from '%s'. Elapsed: %.2fs."),
			TreesPlaced,
			TreesSkipped,
			TilesSpawned,
			TreeMeshes.Num(),
			*TreeMeshFolder,
			Result.ElapsedSeconds);
	}

	UE_LOG(LogTreePlacer, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogTreePlacer, Display, TEXT("========== Tree Place END =========="));
	return Result;
}
