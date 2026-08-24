#include "BuildingExtruderBPLibrary.h"

#include "BuildingCesiumPlacement.h"
#include "BuildingExtrudeUtils.h"
#include "BuildingExtruderLog.h"
#include "BuildingExtruderTileActor.h"
#include "BuildingFoliagePlacement.h"
#include "BuildingMeshFolderLoader.h"
#include "BuildingRoofObjectPlacement.h"
#include "BuildingShapefileReader.h"
#include "BuildingStaticMeshUtils.h"

#include "CesiumGeoreference.h"
#include "Components/ActorComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Editor.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "InstancedFoliageActor.h"
#include "Internationalization/Internationalization.h"
#include "Materials/MaterialInterface.h"
#include "Misc/Paths.h"
#include "Misc/ScopedSlowTask.h"

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

	FVector2D FeatureCentroidLonLat(const FBuildingShapefileFeature& Feature)
	{
		FVector2D Sum(0, 0);
		for (const FVector2D& P : Feature.OuterRingLonLat)
		{
			Sum += P;
		}
		return Sum / static_cast<double>(FMath::Max(Feature.OuterRingLonLat.Num(), 1));
	}

	/**
	 * Chooses TXxTY with TX*TY == TargetTileCount (exact factor pair).
	 * Picks the pair whose cells are closest to square in meters.
	 */
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

	void AssignEvenRandomSlots(
		const TArray<int32>& Bucket,
		int32 SlotCount,
		FRandomStream& Rng,
		TMap<int32, int32>& OutFeatureToSlot)
	{
		OutFeatureToSlot.Reset();
		const int32 Slots = FMath::Max(SlotCount, 1);
		TArray<int32> Order = Bucket;
		for (int32 I = 0; I < Order.Num(); ++I)
		{
			const int32 J = Rng.RandRange(I, Order.Num() - 1);
			Order.Swap(I, J);
		}
		for (int32 I = 0; I < Order.Num(); ++I)
		{
			OutFeatureToSlot.Add(Order[I], I % Slots);
		}
	}

	int32 CommitRoofFoliage(
		AInstancedFoliageActor* IFA,
		TArray<FBuildingFoliageSlot>& Slots,
		TArray<TArray<FTransform>>& Transforms,
		UWorld* World)
	{
		int32 Count = 0;
		if (!IFA)
		{
			return 0;
		}
		for (int32 I = 0; I < Slots.Num(); ++I)
		{
			if (Transforms.IsValidIndex(I))
			{
				BuildingFoliagePlacement::AddInstances(Slots[I], Transforms[I]);
				Count += Transforms[I].Num();
				Transforms[I].Reset();
			}
		}
		BuildingFoliagePlacement::RefreshSlots(*IFA, Slots);
		IFA->Modify();
		if (World)
		{
			World->MarkPackageDirty();
		}
		return Count;
	}

	void AppendWorldMesh(
		FExtrudedPrismMesh& Combined,
		const FExtrudedPrismMesh& LocalMesh,
		const FVector& WorldOrigin)
	{
		const int32 VertexOffset = Combined.Vertices.Num();
		Combined.Vertices.Reserve(Combined.Vertices.Num() + LocalMesh.Vertices.Num());
		Combined.Normals.Reserve(Combined.Normals.Num() + LocalMesh.Normals.Num());
		Combined.UVs.Reserve(Combined.UVs.Num() + LocalMesh.UVs.Num());
		Combined.Triangles.Reserve(Combined.Triangles.Num() + LocalMesh.Triangles.Num());
		Combined.TriangleMaterialIndices.Reserve(
			Combined.TriangleMaterialIndices.Num() + LocalMesh.TriangleMaterialIndices.Num());

		for (const FVector& V : LocalMesh.Vertices)
		{
			Combined.Vertices.Add(V + WorldOrigin);
		}
		Combined.Normals.Append(LocalMesh.Normals);
		Combined.UVs.Append(LocalMesh.UVs);
		Combined.TriangleMaterialIndices.Append(LocalMesh.TriangleMaterialIndices);
		for (const int32 Index : LocalMesh.Triangles)
		{
			Combined.Triangles.Add(Index + VertexOffset);
		}
	}

	bool BuildFeaturePartsWorld(
		ACesiumGeoreference& Georeference,
		const FBuildingShapefileFeature& Feature,
		double BaseAltitudeM,
		double MetersPerUv,
		bool bUseRoofTypes,
		int32 FlatRoofIndex,
		int32 HippedRoofIndex,
		int32 ParapetRoofIndex,
		double ParapetHeightMeters,
		double ParapetWidthMeters,
		double HippedHeightMeters,
		FExtrudedPrismMesh& OutWallsAndFloorWorld,
		FExtrudedPrismMesh& OutRoofWorld,
		FVector& OutCentroid,
		TArray<FRoofPlaceTriangle>* OutPlaceTris,
		TArray<FVector2D>* OutFootprintXY,
		FString& OutError)
	{
		const TArray<FVector2D>& Ring = Feature.OuterRingLonLat;
		if (Ring.Num() < 3)
		{
			OutError = TEXT("Ring too small.");
			return false;
		}

		const double TopHeightM = BaseAltitudeM + Feature.HeightM;

		TArray<FVector> BaseWorld;
		TArray<FVector> TopWorld;
		BaseWorld.Reserve(Ring.Num());
		TopWorld.Reserve(Ring.Num());

		FVector Origin(0, 0, 0);
		for (const FVector2D& LonLat : Ring)
		{
			const FVector BasePos = BuildingCesiumPlacement::LonLatHeightToUnreal(
				Georeference, LonLat.X, LonLat.Y, BaseAltitudeM);
			const FVector TopPos = BuildingCesiumPlacement::LonLatHeightToUnreal(
				Georeference, LonLat.X, LonLat.Y, TopHeightM);
			BaseWorld.Add(BasePos);
			TopWorld.Add(TopPos);
			Origin += BasePos;
		}
		Origin /= static_cast<double>(Ring.Num());
		OutCentroid = Origin;

		TArray<FVector> BaseLocal;
		TArray<FVector> TopLocal;
		BaseLocal.Reserve(Ring.Num());
		TopLocal.Reserve(Ring.Num());
		for (int32 I = 0; I < Ring.Num(); ++I)
		{
			BaseLocal.Add(BaseWorld[I] - Origin);
			TopLocal.Add(TopWorld[I] - Origin);
		}

		const EBuildingRoofType RoofType = bUseRoofTypes
			? BuildingExtrudeUtils::ResolveRoofType(
				Feature.RoofTypeCode,
				FlatRoofIndex,
				HippedRoofIndex,
				ParapetRoofIndex)
			: EBuildingRoofType::Flat;

		FExtrudedPrismMesh LocalWalls;
		FExtrudedPrismMesh LocalRoof;
		if (!BuildingExtrudeUtils::BuildRoofPartsFromRings(
				RoofType,
				BaseLocal,
				TopLocal,
				MetersPerUv,
				ParapetHeightMeters,
				ParapetWidthMeters,
				HippedHeightMeters,
				LocalWalls,
				LocalRoof,
				OutError))
		{
			return false;
		}

		OutWallsAndFloorWorld = FExtrudedPrismMesh();
		OutRoofWorld = FExtrudedPrismMesh();
		AppendWorldMesh(OutWallsAndFloorWorld, LocalWalls, Origin);
		AppendWorldMesh(OutRoofWorld, LocalRoof, Origin);

		if (OutFootprintXY)
		{
			OutFootprintXY->Reset();
			OutFootprintXY->Reserve(TopWorld.Num());
			for (const FVector& P : TopWorld)
			{
				OutFootprintXY->Add(FVector2D(P.X, P.Y));
			}
		}

		if (OutPlaceTris)
		{
			OutPlaceTris->Reset();
			TArray<FRoofPlaceTriangle> LocalPlace;
			if (BuildingExtrudeUtils::BuildRoofPlacementTriangles(
					RoofType,
					BaseLocal,
					TopLocal,
					ParapetHeightMeters,
					ParapetWidthMeters,
					HippedHeightMeters,
					LocalPlace))
			{
				OutPlaceTris->Reserve(LocalPlace.Num());
				for (const FRoofPlaceTriangle& Tri : LocalPlace)
				{
					FRoofPlaceTriangle WorldTri;
					WorldTri.A = Tri.A + Origin;
					WorldTri.B = Tri.B + Origin;
					WorldTri.C = Tri.C + Origin;
					WorldTri.AlignDirXY = Tri.AlignDirXY;
					OutPlaceTris->Add(WorldTri);
				}
			}
		}
		return true;
	}

	void AppendPathTags(UActorComponent& Comp, const TArray<FName>& PathTags)
	{
		for (const FName& Tag : PathTags)
		{
			Comp.ComponentTags.AddUnique(Tag);
		}
	}

	USceneComponent* MakeSceneFolder(
		AActor& Owner,
		USceneComponent& Parent,
		const FName Name,
		const TArray<FName>& PathTags)
	{
		USceneComponent* Folder = NewObject<USceneComponent>(&Owner, Name, RF_Transactional);
		if (!Folder)
		{
			return nullptr;
		}
		Folder->SetMobility(EComponentMobility::Static);
		Folder->SetupAttachment(&Parent);
		Owner.AddInstanceComponent(Folder);
		Folder->RegisterComponent();
		AppendPathTags(*Folder, PathTags);
		return Folder;
	}

	bool AttachSlotMesh(
		AActor& Owner,
		USceneComponent& Parent,
		const FName CompName,
		UStaticMesh& Mesh,
		const TArray<FName>& PathTags,
		FString& OutError)
	{
		UStaticMeshComponent* Comp = NewObject<UStaticMeshComponent>(&Owner, CompName, RF_Transactional);
		if (!Comp)
		{
			OutError = TEXT("Failed to create StaticMeshComponent.");
			return false;
		}
		Comp->SetMobility(EComponentMobility::Static);
		Comp->SetupAttachment(&Parent);
		Owner.AddInstanceComponent(Comp);
		Comp->RegisterComponent();
		Comp->SetStaticMesh(&Mesh);
		Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		AppendPathTags(*Comp, PathTags);
		return true;
	}

	void ShiftMeshToLocal(FExtrudedPrismMesh& Mesh, const FVector& Origin)
	{
		for (FVector& V : Mesh.Vertices)
		{
			V -= Origin;
		}
	}

	FVector CombinedVertexCentroid(const TArray<FExtrudedPrismMesh*>& Meshes)
	{
		FVector Sum(0, 0, 0);
		int32 Count = 0;
		for (const FExtrudedPrismMesh* Mesh : Meshes)
		{
			if (!Mesh)
			{
				continue;
			}
			for (const FVector& V : Mesh->Vertices)
			{
				Sum += V;
				++Count;
			}
		}
		return Count > 0 ? (Sum / static_cast<double>(Count)) : FVector::ZeroVector;
	}

	bool SpawnBuildingTileActor(
		UWorld& World,
		const FExtrudedPrismMesh& WorldWallsMesh,
		const FExtrudedPrismMesh& WorldRoofMesh,
		const FString& ActorLabel,
		const FString& EditorFolderPath,
		UMaterialInterface* Material,
		int32 NumWallSlots,
		int32 NumRoofSlots,
		AActor*& OutActor,
		FString& OutError)
	{
		OutActor = nullptr;

		TArray<FExtrudedPrismMesh> WallParts;
		TArray<FExtrudedPrismMesh> RoofParts;
		TArray<FExtrudedPrismMesh*> Used;
		WallParts.SetNum(NumWallSlots);
		RoofParts.SetNum(NumRoofSlots);
		for (int32 Slot = 0; Slot < NumWallSlots; ++Slot)
		{
			if (BuildingExtrudeUtils::ExtractMaterialSlot(WorldWallsMesh, Slot, WallParts[Slot]))
			{
				Used.Add(&WallParts[Slot]);
			}
		}
		for (int32 Slot = 0; Slot < NumRoofSlots; ++Slot)
		{
			if (BuildingExtrudeUtils::ExtractMaterialSlot(WorldRoofMesh, Slot, RoofParts[Slot]))
			{
				Used.Add(&RoofParts[Slot]);
			}
		}
		if (Used.Num() == 0)
		{
			OutError = TEXT("Tile mesh empty.");
			return false;
		}

		const FVector Origin = CombinedVertexCentroid(Used);
		for (FExtrudedPrismMesh* Mesh : Used)
		{
			ShiftMeshToLocal(*Mesh, Origin);
		}

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		SpawnParams.ObjectFlags |= RF_Transactional;

		ABuildingExtruderTileActor* Actor =
			World.SpawnActor<ABuildingExtruderTileActor>(Origin, FRotator::ZeroRotator, SpawnParams);
		if (!Actor || !Actor->GetRootComponent())
		{
			OutError = TEXT("Failed to spawn tile actor.");
			return false;
		}

		Actor->SetActorLabel(ActorLabel);
		Actor->Tags.Add(FName(TEXT("BuildingExtruder")));
		Actor->Tags.Add(FName(TEXT("BuildingExtruderTile")));
		Actor->Tags.Add(FName(TEXT("Building")));
		if (!EditorFolderPath.IsEmpty())
		{
			Actor->SetFolderPath(FName(*EditorFolderPath));
		}

		const TArray<FName> BuildingTags = {FName(TEXT("Building"))};
		const TArray<FName> WallTags = {FName(TEXT("Building")), FName(TEXT("Wall"))};
		const TArray<FName> RoofPathTags = {FName(TEXT("Building")), FName(TEXT("Roof"))};

		USceneComponent* BuildingFolder = MakeSceneFolder(
			*Actor, *Actor->GetRootComponent(), TEXT("Building"), BuildingTags);
		USceneComponent* WallFolder = BuildingFolder
			? MakeSceneFolder(*Actor, *BuildingFolder, TEXT("Wall"), WallTags)
			: nullptr;
		USceneComponent* RoofFolder = BuildingFolder
			? MakeSceneFolder(*Actor, *BuildingFolder, TEXT("Roof"), RoofPathTags)
			: nullptr;
		if (!BuildingFolder || !WallFolder || !RoofFolder)
		{
			OutError = TEXT("Failed to create tile component folders.");
			Actor->Destroy();
			return false;
		}

		auto SpawnSlotComponents = [&](
			TArray<FExtrudedPrismMesh>& Parts,
			USceneComponent& Parent,
			const TCHAR* Kind,
			const TArray<FName>& PathTags) -> bool
		{
			for (int32 Slot = 0; Slot < Parts.Num(); ++Slot)
			{
				if (Parts[Slot].Triangles.Num() < 3)
				{
					continue;
				}
				const FString AssetName = FString::Printf(TEXT("%s_%s_%d"), *ActorLabel, Kind, Slot);
				const FName CompName(*FString::Printf(TEXT("%s_Material_%d"), Kind, Slot));
				UStaticMesh* StaticMesh = BuildingStaticMeshUtils::CreatePersistentStaticMesh(
					TEXT("/Game/BuildingExtruder/Meshes"),
					AssetName,
					Parts[Slot],
					Material,
					/*NumMaterialSlots*/ 1,
					OutError);
				if (!StaticMesh)
				{
					return false;
				}
				if (!AttachSlotMesh(*Actor, Parent, CompName, *StaticMesh, PathTags, OutError))
				{
					return false;
				}
			}
			return true;
		};

		if (!SpawnSlotComponents(WallParts, *WallFolder, TEXT("Wall"), WallTags)
			|| !SpawnSlotComponents(RoofParts, *RoofFolder, TEXT("Roof"), RoofPathTags))
		{
			Actor->Destroy();
			return false;
		}

		Actor->Modify();
		Actor->MarkPackageDirty();
		OutActor = Actor;
		return true;
	}
}

FBuildingExtrudeResult UBuildingExtruderBPLibrary::ImportAndExtrudeBuildingsFromShapefile(
	UObject* WorldContextObject,
	const FString& ShapefilePath,
	const FString& AltitudeFieldName,
	const FString& HeightFieldName,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath,
	int32 TargetTileCount,
	int32 WallMaterialSlotCount,
	int32 RoofMaterialSlotCount,
	float MetersPerUv,
	int32 MaterialRandomSeed,
	bool bUseRoofTypes,
	const FString& RoofTypeFieldName,
	int32 FlatRoofIndex,
	int32 HippedRoofIndex,
	int32 ParapetRoofIndex,
	float ParapetHeightMeters,
	float ParapetWidthMeters,
	float HippedHeightMeters,
	bool bPlaceRoofObjects,
	const FString& RoofObjectMeshFolder)
{
	FBuildingExtrudeResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString AltitudeField = AltitudeFieldName.IsEmpty() ? TEXT("altitude") : AltitudeFieldName;
	const FString HeightField = HeightFieldName.IsEmpty() ? TEXT("height") : HeightFieldName;
	const FString RoofTypeField = (!bUseRoofTypes)
		? FString()
		: (RoofTypeFieldName.IsEmpty() ? TEXT("roof_type") : RoofTypeFieldName);
	const double UvMeters = FMath::Max(static_cast<double>(MetersPerUv), 0.01);
	const double ParapetHeightM = FMath::Max(static_cast<double>(ParapetHeightMeters), 0.0);
	const double ParapetWidthM = FMath::Max(static_cast<double>(ParapetWidthMeters), 0.0);
	const double HippedHeightM = FMath::Max(static_cast<double>(HippedHeightMeters), 0.0);
	const int32 WallSlots = FMath::Clamp(WallMaterialSlotCount, 1, 64);
	const int32 RoofSlots = FMath::Clamp(RoofMaterialSlotCount, 1, 64);

	const FString CleanInputPath = SanitizeFilePath(ShapefilePath);

	UE_LOG(LogBuildingExtruder, Display, TEXT("========== Extrude START =========="));
	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("shp='%s' altitudeField='%s' heightField='%s' useRoofTypes=%s roofTypeField='%s' "
			 "flatIdx=%d hippedIdx=%d parapetIdx=%d parapetH=%.2fm parapetW=%.2fm hippedH=%.2fm "
			 "targetTiles=%d metersPerUv=%.3f wallSlots=%d roofSlots=%d matSeed=%d "
			 "roofObjects=%s folder='%s'"),
		*CleanInputPath,
		*AltitudeField,
		*HeightField,
		bUseRoofTypes ? TEXT("on") : TEXT("off"),
		bUseRoofTypes ? *RoofTypeField : TEXT("(unused)"),
		FlatRoofIndex,
		HippedRoofIndex,
		ParapetRoofIndex,
		ParapetHeightM,
		ParapetWidthM,
		HippedHeightM,
		TargetTileCount,
		UvMeters,
		WallSlots,
		RoofSlots,
		MaterialRandomSeed,
		bPlaceRoofObjects ? TEXT("on") : TEXT("off"),
		*RoofObjectMeshFolder);

	UWorld* World = ResolveEditorWorld(WorldContextObject);
	if (!World)
	{
		Result.Message = TEXT("Could not resolve an editor world. Open a map first.");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	ACesiumGeoreference* Georeference = BuildingCesiumPlacement::FindGeoreference(World);
	if (!Georeference)
	{
		Result.Message = TEXT("No ACesiumGeoreference found in the level. Add Cesium Georeference (same one used by tilesets).");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (CleanInputPath.IsEmpty())
	{
		Result.Message = TEXT("ShapefilePath is empty (provide a .shp path).");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<UStaticMesh*> RoofMeshes;
	TArray<FRoofObjectFootprint> RoofFeet;
	TArray<FBuildingFoliageSlot> RoofFoliageSlots;
	TArray<TArray<FTransform>> RoofTransforms;
	AInstancedFoliageActor* RoofIFA = nullptr;
	int32 RoofObjectsPlaced = 0;
	if (bPlaceRoofObjects)
	{
		FString MeshError;
		if (!BuildingMeshFolderLoader::LoadStaticMeshesFromFolder(RoofObjectMeshFolder, RoofMeshes, MeshError))
		{
			Result.Message = MeshError;
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
			return Result;
		}
		RoofIFA = AInstancedFoliageActor::GetInstancedFoliageActorForCurrentLevel(World, /*bCreateIfNone*/ true);
		if (!RoofIFA)
		{
			Result.Message = TEXT("Could not get or create the level InstancedFoliageActor for roof objects.");
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
			return Result;
		}
		RoofIFA->Modify();
		RoofFeet.SetNum(RoofMeshes.Num());
		RoofFoliageSlots.SetNum(RoofMeshes.Num());
		RoofTransforms.SetNum(RoofMeshes.Num());
		for (int32 MeshIndex = 0; MeshIndex < RoofMeshes.Num(); ++MeshIndex)
		{
			RoofFeet[MeshIndex] = BuildingRoofObjectPlacement::MakeFootprint(*RoofMeshes[MeshIndex]);
			FString SlotError;
			if (!BuildingFoliagePlacement::GetOrCreateSlot(
					*RoofIFA, RoofMeshes[MeshIndex], RoofFoliageSlots[MeshIndex], SlotError))
			{
				Result.Message = SlotError;
				Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
				UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
				return Result;
			}
		}
	}

	if (FPaths::GetExtension(CleanInputPath).Equals(TEXT("gpkg"), ESearchCase::IgnoreCase))
	{
		Result.Message = TEXT("GeoPackage (.gpkg) is not supported. Export to ESRI Shapefile (.shp/.dbf) and use that path.");
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (bUseRoofTypes
		&& (FlatRoofIndex == HippedRoofIndex
			|| FlatRoofIndex == ParapetRoofIndex
			|| HippedRoofIndex == ParapetRoofIndex))
	{
		Result.Message = FString::Printf(
			TEXT("Roof type indices must be unique (flat=%d, hipped=%d, parapet=%d)."),
			FlatRoofIndex,
			HippedRoofIndex,
			ParapetRoofIndex);
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<FBuildingShapefileFeature> Features;
	FString ReadError;
	if (!BuildingShapefileReader::ReadPolygonBuildings(
			CleanInputPath, HeightField, AltitudeField, RoofTypeField, Features, ReadError))
	{
		Result.Message = ReadError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		UE_LOG(LogBuildingExtruder, Display, TEXT("========== Extrude END (read failed) =========="));
		return Result;
	}

	constexpr double MaxValidHeightM = 50.0;
	double ValidSum = 0.0;
	int32 ValidCount = 0;
	for (const FBuildingShapefileFeature& Feature : Features)
	{
		if (Feature.HeightM > 0.0 && Feature.HeightM <= MaxValidHeightM)
		{
			ValidSum += Feature.HeightM;
			++ValidCount;
		}
	}

	if (ValidCount == 0)
	{
		Result.Message = FString::Printf(
			TEXT("No valid %s values in (0, %.0f] m; cannot compute replacement average."),
			*HeightField,
			MaxValidHeightM);
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	const double AverageHeightM = ValidSum / static_cast<double>(ValidCount);
	int32 AnomalyCount = 0;
	for (FBuildingShapefileFeature& Feature : Features)
	{
		if (Feature.HeightM <= 0.0 || Feature.HeightM > MaxValidHeightM)
		{
			Feature.HeightM = AverageHeightM;
			++AnomalyCount;
		}
	}

	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("Height sanitize (%s): valid=%d anomalies=%d average=%.3f m"),
		*HeightField,
		ValidCount,
		AnomalyCount,
		AverageHeightM);

	const int32 ToImport = Features.Num();
	UE_LOG(LogBuildingExtruder, Display, TEXT("Will process %d footprints"), ToImport);

	double MinLon = TNumericLimits<double>::Max();
	double MaxLon = TNumericLimits<double>::Lowest();
	double MinLat = TNumericLimits<double>::Max();
	double MaxLat = TNumericLimits<double>::Lowest();
	int32 BoundsCount = 0;
	for (int32 I = 0; I < ToImport; ++I)
	{
		if (Features[I].OuterRingLonLat.Num() < 3)
		{
			continue;
		}
		const FVector2D C = FeatureCentroidLonLat(Features[I]);
		MinLon = FMath::Min(MinLon, C.X);
		MaxLon = FMath::Max(MaxLon, C.X);
		MinLat = FMath::Min(MinLat, C.Y);
		MaxLat = FMath::Max(MaxLat, C.Y);
		++BoundsCount;
	}

	if (BoundsCount == 0)
	{
		Result.Message = TEXT("No buildings with valid footprints to place.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}
	const double LonSpan = FMath::Max(MaxLon - MinLon, 1.0e-9);
	const double LatSpan = FMath::Max(MaxLat - MinLat, 1.0e-9);

	int32 TilesX = 1;
	int32 TilesY = 1;
	ChooseSquareTileGrid(MinLon, MaxLon, MinLat, MaxLat, TargetTileCount, TilesX, TilesY);

	const int32 TileSlotCount = TilesX * TilesY;
	TArray<TArray<int32>> TileFeatureIndices;
	TileFeatureIndices.SetNum(TileSlotCount);
	for (int32 I = 0; I < ToImport; ++I)
	{
		if (Features[I].OuterRingLonLat.Num() < 3)
		{
			continue;
		}
		const FVector2D C = FeatureCentroidLonLat(Features[I]);
		int32 TX = FMath::FloorToInt(static_cast<float>((C.X - MinLon) / LonSpan * TilesX));
		int32 TY = FMath::FloorToInt(static_cast<float>((C.Y - MinLat) / LatSpan * TilesY));
		TX = FMath::Clamp(TX, 0, TilesX - 1);
		TY = FMath::Clamp(TY, 0, TilesY - 1);
		TileFeatureIndices[TY * TilesX + TX].Add(I);
	}

	int32 NonEmptyTiles = 0;
	int32 BuildingsInTiles = 0;
	for (const TArray<int32>& Bucket : TileFeatureIndices)
	{
		if (Bucket.Num() > 0)
		{
			++NonEmptyTiles;
			BuildingsInTiles += Bucket.Num();
		}
	}

	UE_LOG(
		LogBuildingExtruder,
		Display,
		TEXT("Tiling: buildings=%d target=%d chosenGrid=%dx%d (%d slots) processingNonEmpty=%d placed=%d bounds lon[%.6f,%.6f] lat[%.6f,%.6f]"),
		ToImport,
		TargetTileCount,
		TilesX,
		TilesY,
		TileSlotCount,
		NonEmptyTiles,
		BuildingsInTiles,
		MinLon,
		MaxLon,
		MinLat,
		MaxLat);

	if (NonEmptyTiles == 0)
	{
		Result.Message = TEXT("No buildings assigned to any tile.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	{
		int32 AltOk = 0;
		double AltMin = TNumericLimits<double>::Max();
		double AltMax = TNumericLimits<double>::Lowest();
		for (int32 TileIndex = 0; TileIndex < TileFeatureIndices.Num(); ++TileIndex)
		{
			for (const int32 FeatureIndex : TileFeatureIndices[TileIndex])
			{
				const double Z = Features[FeatureIndex].ElevationM;
				AltMin = FMath::Min(AltMin, Z);
				AltMax = FMath::Max(AltMax, Z);
				++AltOk;
			}
		}
		UE_LOG(
			LogBuildingExtruder,
			Display,
			TEXT("Using shapefile altitude field '%s' for %d buildings. Z range [%.3f, %.3f] m"),
			*AltitudeField,
			AltOk,
			AltOk > 0 ? AltMin : 0.0,
			AltOk > 0 ? AltMax : 0.0);
	}

	const float SpawnTileWork = static_cast<float>(FMath::Max(NonEmptyTiles, 1));
	FScopedSlowTask SlowTask(
		SpawnTileWork,
		NSLOCTEXT("BuildingExtruder", "ImportProgress", "Extruding tiled buildings..."));
	SlowTask.MakeDialog(true);

	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("BldgTile") : ActorLabelPrefix;
	const FString FolderPath = EditorFolderPath;

	int32 BuildingsMeshed = 0;
	int32 BuildingsSkipped = 0;
	int32 TilesSpawned = 0;
	UMaterialInterface* CorrectMaterial = nullptr;
	{
		FString MaterialError;
		CorrectMaterial = BuildingStaticMeshUtils::GetOrCreateBuildingMaterial(MaterialError);
		if (!CorrectMaterial)
		{
			Result.Message = MaterialError.IsEmpty()
				? TEXT("Failed to create /Game/BuildingExtruder building material.")
				: MaterialError;
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
			return Result;
		}
	}
	}

	FRandomStream MaterialRng;
	if (MaterialRandomSeed == 0)
	{
		MaterialRng.GenerateNewSeed();
	}
	else
	{
		MaterialRng.Initialize(MaterialRandomSeed);
	}

	for (int32 TileY = 0; TileY < TilesY; ++TileY)
	{
		for (int32 TileX = 0; TileX < TilesX; ++TileX)
		{
			const TArray<int32>& Bucket = TileFeatureIndices[TileY * TilesX + TileX];
			if (Bucket.Num() == 0)
			{
				continue;
			}

			const FString TileLabel = FString::Printf(TEXT("%s_%d_%d"), *LabelPrefix, TileX, TileY);
			SlowTask.EnterProgressFrame(
				1.0f,
				FText::Format(
					NSLOCTEXT("BuildingExtruder", "ImportProgressTile", "Tile {0} ({1} buildings)"),
					FText::FromString(TileLabel),
					FText::AsNumber(Bucket.Num())));

			if (SlowTask.ShouldCancel())
			{
				Result.bCancelled = true;
				Result.BuildingsSpawned = BuildingsMeshed;
				Result.BuildingsSkipped = BuildingsSkipped + (ToImport - BuildingsMeshed - BuildingsSkipped);
				Result.TilesSpawned = TilesSpawned;
				Result.RoofObjectsPlaced = CommitRoofFoliage(RoofIFA, RoofFoliageSlots, RoofTransforms, World);
				Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
				Result.Message = FString::Printf(
					TEXT("Cancelled. Tiles=%d buildings=%d roofObjects=%d. Elapsed: %.2fs."),
					TilesSpawned,
					BuildingsMeshed,
					Result.RoofObjectsPlaced,
					Result.ElapsedSeconds);
				UE_LOG(LogBuildingExtruder, Warning, TEXT("%s"), *Result.Message);
				return Result;
			}

			FExtrudedPrismMesh TileWallsMesh;
			FExtrudedPrismMesh TileRoofMesh;
			int32 TileBuildingCount = 0;
			TArray<TArray<FTransform>> TileRoofPending;
			if (bPlaceRoofObjects)
			{
				TileRoofPending.SetNum(RoofMeshes.Num());
			}

			TMap<int32, int32> WallSlotByFeature;
			TMap<int32, int32> RoofSlotByFeature;
			AssignEvenRandomSlots(Bucket, WallSlots, MaterialRng, WallSlotByFeature);
			AssignEvenRandomSlots(Bucket, RoofSlots, MaterialRng, RoofSlotByFeature);

			for (const int32 FeatureIndex : Bucket)
			{
				FExtrudedPrismMesh BuildingWalls;
				FExtrudedPrismMesh BuildingRoof;
				FVector Centroid;
				FString ExtrudeError;
				TArray<FRoofPlaceTriangle> PlaceTris;
				TArray<FVector2D> FootprintXY;
				if (!BuildFeaturePartsWorld(
						*Georeference,
						Features[FeatureIndex],
						Features[FeatureIndex].ElevationM,
						UvMeters,
						bUseRoofTypes,
						FlatRoofIndex,
						HippedRoofIndex,
						ParapetRoofIndex,
						ParapetHeightM,
						ParapetWidthM,
						HippedHeightM,
						BuildingWalls,
						BuildingRoof,
						Centroid,
						bPlaceRoofObjects ? &PlaceTris : nullptr,
						bPlaceRoofObjects ? &FootprintXY : nullptr,
						ExtrudeError))
				{
					++BuildingsSkipped;
					UE_LOG(
						LogBuildingExtruder,
						Warning,
						TEXT("Skipped record %d: %s"),
						Features[FeatureIndex].RecordIndex,
						*ExtrudeError);
					continue;
				}

				const int32 WallSlot = WallSlotByFeature.FindRef(FeatureIndex);
				const int32 RoofSlot = RoofSlotByFeature.FindRef(FeatureIndex);
				BuildingExtrudeUtils::AssignAllTrianglesMaterialSlot(BuildingWalls, WallSlot);
				BuildingExtrudeUtils::AssignAllTrianglesMaterialSlot(BuildingRoof, RoofSlot);

				AppendWorldMesh(TileWallsMesh, BuildingWalls, FVector::ZeroVector);
				AppendWorldMesh(TileRoofMesh, BuildingRoof, FVector::ZeroVector);

				if (bPlaceRoofObjects && PlaceTris.Num() > 0 && RoofMeshes.Num() > 0)
				{
					TArray<FPlacedRoofObject2D> Occupied;
					TArray<int32> Order;
					Order.SetNum(RoofMeshes.Num());
					for (int32 I = 0; I < RoofMeshes.Num(); ++I)
					{
						Order[I] = I;
					}
					for (int32 I = 0; I < Order.Num(); ++I)
					{
						const int32 J = MaterialRng.RandRange(I, Order.Num() - 1);
						Order.Swap(I, J);
					}
					for (const int32 MeshIndex : Order)
					{
						if (MaterialRng.FRand() >= 0.5f)
						{
							continue;
						}
						FTransform Xform;
						FPlacedRoofObject2D Placed;
						if (!BuildingRoofObjectPlacement::TryPlace(
								PlaceTris,
								FootprintXY,
								RoofFeet[MeshIndex],
								Occupied,
								MaterialRng,
								Xform,
								Placed))
						{
							continue;
						}
						Occupied.Add(Placed);
						TileRoofPending[MeshIndex].Add(Xform);
					}
				}

				++BuildingsMeshed;
				++TileBuildingCount;
			}

			if (TileBuildingCount == 0)
			{
				continue;
			}

			AActor* TileActor = nullptr;
			FString SpawnError;
			if (!SpawnBuildingTileActor(
					*World,
					TileWallsMesh,
					TileRoofMesh,
					TileLabel,
					FolderPath,
					CorrectMaterial,
					WallSlots,
					RoofSlots,
					TileActor,
					SpawnError))
			{
				UE_LOG(LogBuildingExtruder, Error, TEXT("Tile %s failed: %s"), *TileLabel, *SpawnError);
				BuildingsSkipped += TileBuildingCount;
				BuildingsMeshed -= TileBuildingCount;
				continue;
			}

			++TilesSpawned;
			for (int32 MeshIndex = 0; MeshIndex < TileRoofPending.Num(); ++MeshIndex)
			{
				RoofTransforms[MeshIndex].Append(TileRoofPending[MeshIndex]);
			}
		}
	}

	RoofObjectsPlaced = CommitRoofFoliage(RoofIFA, RoofFoliageSlots, RoofTransforms, World);

	if (TilesSpawned <= 0)
	{
		Result.BuildingsSpawned = 0;
		Result.BuildingsSkipped = BuildingsSkipped;
		Result.TilesSpawned = 0;
		Result.RoofObjectsPlaced = RoofObjectsPlaced;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(
			TEXT("No tiles spawned (%d buildings skipped). Elapsed: %.2fs."),
			BuildingsSkipped,
			Result.ElapsedSeconds);
		UE_LOG(LogBuildingExtruder, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	Result.bSuccess = true;
	Result.BuildingsSpawned = BuildingsMeshed;
	Result.BuildingsSkipped = BuildingsSkipped;
	Result.TilesSpawned = TilesSpawned;
	Result.RoofObjectsPlaced = RoofObjectsPlaced;
	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	Result.Message = FString::Printf(
		TEXT("Spawned %d tiles (%d buildings, %d skipped, %d roof objects), "
			 "saved per-material meshes under /Game/BuildingExtruder/Meshes. Elapsed: %.2fs."),
		TilesSpawned,
		BuildingsMeshed,
		BuildingsSkipped,
		RoofObjectsPlaced,
		Result.ElapsedSeconds);

	World->MarkPackageDirty();
	UE_LOG(LogBuildingExtruder, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogBuildingExtruder, Display, TEXT("========== Extrude END =========="));
	return Result;
}
