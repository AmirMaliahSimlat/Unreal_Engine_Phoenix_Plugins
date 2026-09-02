#include "WaterPlacerBPLibrary.h"

#include "WaterCesiumPlacement.h"
#include "WaterPlacerLog.h"
#include "WaterShapefileReader.h"
#include "WaterStaticMesh.h"

#include "Cesium3DTileset.h"
#include "CesiumCartographicPolygon.h"
#include "CesiumGeoreference.h"
#include "CesiumPolygonRasterOverlay.h"
#include "Components/SplineComponent.h"
#include "Editor.h"
#include "Engine/Engine.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Internationalization/Internationalization.h"
#include "Materials/MaterialInterface.h"
#include "Misc/PackageName.h"
#include "Misc/ScopedSlowTask.h"
#include "UObject/SoftObjectPath.h"

namespace
{
	const FName WaterPlacerTag(TEXT("WaterPlacer"));
	const FName WaterPlacerOverlayName(TEXT("WaterPlacerClip"));
	constexpr double DuplicateEpsDeg = 1.0e-10;
	constexpr int32 MinOutlineVertices = 32;
	constexpr int32 HardMaxOutlineVertices = 16384;
	constexpr double MetersPerUv = 50.0;

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

	double PerpDistSq(const FVector2D& Point, const FVector2D& A, const FVector2D& B)
	{
		const FVector2D AB = B - A;
		const double LenSq = AB.SizeSquared();
		if (LenSq < 1.0e-30)
		{
			return FVector2D::DistSquared(Point, A);
		}
		const double T = FMath::Clamp(FVector2D::DotProduct(Point - A, AB) / LenSq, 0.0, 1.0);
		return FVector2D::DistSquared(Point, A + AB * T);
	}

	void RdpKeep(const TArray<FVector2D>& Pts, int32 Start, int32 End, double EpsSq, TArray<uint8>& Keep)
	{
		double MaxD = -1.0;
		int32 MaxI = Start;
		for (int32 I = Start + 1; I < End; ++I)
		{
			const double D = PerpDistSq(Pts[I], Pts[Start], Pts[End]);
			if (D > MaxD)
			{
				MaxD = D;
				MaxI = I;
			}
		}
		if (MaxD > EpsSq && MaxI > Start && MaxI < End)
		{
			RdpKeep(Pts, Start, MaxI, EpsSq, Keep);
			RdpKeep(Pts, MaxI, End, EpsSq, Keep);
		}
		else
		{
			Keep[Start] = 1;
			Keep[End] = 1;
		}
	}

	void UniformSample(const TArray<FVector2D>& In, int32 MaxPoints, TArray<FVector2D>& Out)
	{
		Out.Reset();
		if (In.Num() == 0 || MaxPoints < 3)
		{
			return;
		}
		if (In.Num() <= MaxPoints)
		{
			Out = In;
			return;
		}
		Out.Reserve(MaxPoints);
		for (int32 I = 0; I < MaxPoints; ++I)
		{
			const int32 Src = (I * (In.Num() - 1)) / (MaxPoints - 1);
			Out.Add(In[Src]);
		}
		if (Out.Num() >= 2 && Out[0].Equals(Out.Last(), DuplicateEpsDeg))
		{
			Out.Pop();
		}
	}

	void ApplyRdp(const TArray<FVector2D>& Unique, double EpsDeg, TArray<FVector2D>& Out)
	{
		Out.Reset();
		if (Unique.Num() < 3)
		{
			Out = Unique;
			return;
		}
		if (EpsDeg <= DuplicateEpsDeg)
		{
			Out = Unique;
			return;
		}

		TArray<uint8> Keep;
		Keep.Init(0, Unique.Num());
		RdpKeep(Unique, 0, Unique.Num() - 1, EpsDeg * EpsDeg, Keep);
		Keep[0] = 1;
		Keep.Last() = 1;
		Out.Reserve(Unique.Num());
		for (int32 I = 0; I < Unique.Num(); ++I)
		{
			if (Keep[I])
			{
				Out.Add(Unique[I]);
			}
		}
		if (Out.Num() < 3)
		{
			Out = Unique;
		}
	}

	void ChaikinClosed(const TArray<FVector2D>& In, TArray<FVector2D>& Out)
	{
		Out.Reset();
		const int32 N = In.Num();
		if (N < 3)
		{
			Out = In;
			return;
		}
		Out.Reserve(N * 2);
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D& A = In[I];
			const FVector2D& B = In[(I + 1) % N];
			Out.Add(A * 0.75 + B * 0.25);
			Out.Add(A * 0.25 + B * 0.75);
		}
	}

	void DecimateRing(const TArray<FVector2D>& In, int32 MaxPoints, double SmoothMeters, TArray<FVector2D>& Out)
	{
		Out.Reset();
		if (In.Num() == 0)
		{
			return;
		}

		TArray<FVector2D> Unique;
		Unique.Reserve(In.Num());
		for (const FVector2D& P : In)
		{
			if (Unique.Num() == 0 || !Unique.Last().Equals(P, DuplicateEpsDeg))
			{
				Unique.Add(P);
			}
		}
		if (Unique.Num() >= 2 && Unique[0].Equals(Unique.Last(), DuplicateEpsDeg))
		{
			Unique.Pop();
		}

		TArray<FVector2D> Working;
		if (SmoothMeters > 0.0)
		{
			const double EpsDeg = SmoothMeters / 111320.0;
			ApplyRdp(Unique, EpsDeg, Working);
			TArray<FVector2D> Rounded;
			ChaikinClosed(Working, Rounded);
			ChaikinClosed(Rounded, Working);
		}
		else
		{
			Working = MoveTemp(Unique);
		}

		if (Working.Num() <= MaxPoints)
		{
			Out = MoveTemp(Working);
			return;
		}

		ApplyRdp(Working, FMath::Max(SmoothMeters, 1.0) / 111320.0, Out);
		if (Out.Num() > MaxPoints)
		{
			UniformSample(Out, MaxPoints, Working);
			Out = MoveTemp(Working);
		}
	}

	void DensifyClosedRing(const TArray<FVector2D>& In, double MaxEdgeMeters, int32 MaxPoints, TArray<FVector2D>& Out)
	{
		Out.Reset();
		if (In.Num() < 3)
		{
			Out = In;
			return;
		}

		double SpacingM = FMath::Max(MaxEdgeMeters, 5.0);
		for (int32 Attempt = 0; Attempt < 8; ++Attempt)
		{
			Out.Reset();
			const double MaxDeg = SpacingM / 111320.0;
			const int32 N = In.Num();
			Out.Reserve(N * 2);
			for (int32 I = 0; I < N; ++I)
			{
				const FVector2D A = In[I];
				const FVector2D B = In[(I + 1) % N];
				Out.Add(A);
				const double DistDeg = FVector2D::Distance(A, B);
				const int32 Segments = FMath::Max(
					1,
					FMath::CeilToInt(static_cast<float>(DistDeg / FMath::Max(MaxDeg, 1.0e-12))));
				for (int32 S = 1; S < Segments; ++S)
				{
					Out.Add(A + (B - A) * (static_cast<double>(S) / Segments));
				}
			}
			if (Out.Num() <= MaxPoints)
			{
				return;
			}
			SpacingM *= 1.6;
		}
	}

	FString NormalizeUnrealAssetPath(const FString& InPath)
	{
		FString Path = SanitizeFilePath(InPath);
		int32 FirstQuote = INDEX_NONE;
		if (Path.FindChar(TCHAR('\''), FirstQuote))
		{
			const int32 LastQuote = Path.Find(TEXT("'"), ESearchCase::IgnoreCase, ESearchDir::FromEnd);
			if (LastQuote > FirstQuote)
			{
				Path = Path.Mid(FirstQuote + 1, LastQuote - FirstQuote - 1);
			}
		}
		return Path;
	}

	UMaterialInterface* LoadWaterMaterialFromPath(const FString& InPath)
	{
		const FString Path = NormalizeUnrealAssetPath(InPath);
		if (Path.IsEmpty())
		{
			UE_LOG(LogWaterPlacer, Warning, TEXT("WaterMaterialPath is empty."));
			return nullptr;
		}

		auto TryLoad = [](const FString& Candidate) -> UMaterialInterface*
		{
			if (Candidate.IsEmpty())
			{
				return nullptr;
			}
			if (UMaterialInterface* Material = LoadObject<UMaterialInterface>(nullptr, *Candidate))
			{
				return Material;
			}
			return Cast<UMaterialInterface>(FSoftObjectPath(Candidate).TryLoad());
		};

		if (UMaterialInterface* Material = TryLoad(Path))
		{
			UE_LOG(LogWaterPlacer, Display, TEXT("Loaded water material '%s'."), *Path);
			return Material;
		}

		if (!Path.Contains(TEXT(".")))
		{
			const FString WithObject = Path + TEXT(".") + FPackageName::GetShortName(Path);
			if (UMaterialInterface* Material = TryLoad(WithObject))
			{
				UE_LOG(LogWaterPlacer, Display, TEXT("Loaded water material '%s'."), *WithObject);
				return Material;
			}
		}

		UE_LOG(
			LogWaterPlacer,
			Error,
			TEXT("Could not load water material from '%s'. Copy Reference from the Content Browser (not a Windows file path)."),
			*InPath);
		return nullptr;
	}

	void ApplyWorldClosedSpline(USplineComponent& Spline, const TArray<FVector>& WorldPoints)
	{
		Spline.ClearSplinePoints(false);
		Spline.SetClosedLoop(false, false);
		for (const FVector& P : WorldPoints)
		{
			Spline.AddSplinePoint(P, ESplineCoordinateSpace::World, false);
		}
		Spline.SetClosedLoop(true, false);
		const int32 Num = Spline.GetNumberOfSplinePoints();
		for (int32 I = 0; I < Num; ++I)
		{
			Spline.SetSplinePointType(I, ESplinePointType::Linear, false);
		}
		Spline.UpdateSpline();
	}

	void RemovePreviousWaterPlacer(UWorld& World)
	{
		for (TActorIterator<ACesium3DTileset> It(&World); It; ++It)
		{
			ACesium3DTileset* Tileset = *It;
			if (!Tileset)
			{
				continue;
			}

			TArray<UCesiumPolygonRasterOverlay*> Overlays;
			Tileset->GetComponents<UCesiumPolygonRasterOverlay>(Overlays);
			for (UCesiumPolygonRasterOverlay* Overlay : Overlays)
			{
				if (!Overlay)
				{
					continue;
				}
				if (Overlay->GetFName().ToString().StartsWith(WaterPlacerOverlayName.ToString()))
				{
					Overlay->RemoveFromTileset();
					Overlay->DestroyComponent();
					Tileset->RefreshTileset();
				}
			}
		}

		TArray<AActor*> ToDestroy;
		for (TActorIterator<AActor> It(&World); It; ++It)
		{
			AActor* Actor = *It;
			if (Actor && Actor->ActorHasTag(WaterPlacerTag))
			{
				ToDestroy.Add(Actor);
			}
		}
		for (AActor* Actor : ToDestroy)
		{
			World.DestroyActor(Actor);
		}
	}

	void ApplyClipOverlayToTileset(
		ACesium3DTileset& Tileset,
		const TArray<ACesiumCartographicPolygon*>& ClipPolygons)
	{
		UCesiumPolygonRasterOverlay* Overlay = NewObject<UCesiumPolygonRasterOverlay>(
			&Tileset,
			WaterPlacerOverlayName,
			RF_Transactional);
		if (!Overlay)
		{
			return;
		}

		Overlay->bAutoActivate = false;
		Tileset.AddInstanceComponent(Overlay);
		Overlay->RegisterComponent();
		Overlay->Polygons.Reset();
		for (ACesiumCartographicPolygon* Poly : ClipPolygons)
		{
			if (Poly)
			{
				Overlay->Polygons.Add(Poly);
			}
		}
		Overlay->InvertSelection = false;
		Overlay->ExcludeSelectedTiles = true;
		Tileset.Modify();
		Overlay->Activate(true);
		Overlay->Refresh();
		Tileset.RefreshTileset();
	}

	ACesiumCartographicPolygon* SpawnClipPolygon(
		UWorld& World,
		const TArray<FVector>& WorldPoints,
		const FString& Label,
		const FString& FolderPath)
	{
		if (WorldPoints.Num() < 3)
		{
			return nullptr;
		}

		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		ACesiumCartographicPolygon* Poly = World.SpawnActor<ACesiumCartographicPolygon>(
			WorldPoints[0],
			FRotator::ZeroRotator,
			Params);
		if (!Poly || !Poly->Polygon)
		{
			return nullptr;
		}

		ApplyWorldClosedSpline(*Poly->Polygon, WorldPoints);
		Poly->SetActorLabel(Label);
		Poly->Tags.AddUnique(WaterPlacerTag);
		if (!FolderPath.IsEmpty())
		{
			Poly->SetFolderPath(FName(*FolderPath));
		}
		Poly->Modify();
		return Poly;
	}

	AStaticMeshActor* SpawnWaterMesh(
		UWorld& World,
		const TArray<FVector>& WorldPoints,
		UMaterialInterface* WaterMaterial,
		const FString& MeshFolder,
		const FString& Label,
		const FString& FolderPath,
		int32 SmoothShadingPasses)
	{
		if (WorldPoints.Num() < 3)
		{
			return nullptr;
		}

		FVector Origin = FVector::ZeroVector;
		for (const FVector& P : WorldPoints)
		{
			Origin += P;
		}
		Origin /= static_cast<double>(WorldPoints.Num());

		TArray<FVector> LocalPoints;
		LocalPoints.Reserve(WorldPoints.Num());
		for (const FVector& P : WorldPoints)
		{
			LocalPoints.Add(P - Origin);
		}

		FWaterFlatMesh Mesh;
		FString MeshError;
		if (!WaterStaticMesh::BuildFlatPolygonMesh(LocalPoints, MetersPerUv, Mesh, MeshError))
		{
			UE_LOG(LogWaterPlacer, Warning, TEXT("Failed to triangulate '%s': %s"), *Label, *MeshError);
			return nullptr;
		}

		UStaticMesh* StaticMesh = WaterStaticMesh::CreatePersistentStaticMesh(
			MeshFolder, Label, Mesh, WaterMaterial, SmoothShadingPasses, MeshError);
		if (!StaticMesh)
		{
			UE_LOG(LogWaterPlacer, Warning, TEXT("Failed to save water mesh '%s': %s"), *Label, *MeshError);
			return nullptr;
		}

		return WaterStaticMesh::SpawnMeshActor(
			World, Origin, StaticMesh, WaterMaterial, Label, FolderPath, WaterPlacerTag);
	}
}

FWaterPlaceResult UWaterPlacerBPLibrary::PlaceWaterFromShapefile(
	UObject* WorldContextObject,
	const FString& ShapefilePath,
	const FString& AltitudeFieldName,
	const FString& WaterMaterialPath,
	const FString& MeshContentFolder,
	bool bClipGroundUnderWater,
	int32 MaxOutlineVertices,
	float OutlineSmoothMeters,
	bool bDrapeOnCesiumTerrain,
	float DrapeHeightOffsetMeters,
	int32 SmoothShadingPasses,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath)
{
	FWaterPlaceResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString CleanInputPath = SanitizeFilePath(ShapefilePath);
	const FString AltitudeField = AltitudeFieldName;
	const FString CleanMaterialPath = SanitizeFilePath(WaterMaterialPath);
	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("Water") : ActorLabelPrefix;
	const FString FolderPath = EditorFolderPath;
	const FString MeshFolder = MeshContentFolder.IsEmpty() ? TEXT("/Game/WaterPlacer/Meshes") : MeshContentFolder;
	const int32 OutlineVertexCap = FMath::Clamp(MaxOutlineVertices, MinOutlineVertices, HardMaxOutlineVertices);
	const double SmoothMeters = FMath::Max(static_cast<double>(OutlineSmoothMeters), 0.0);
	const bool bDrape = bDrapeOnCesiumTerrain;
	const double DrapeOffsetM = FMath::Max(static_cast<double>(DrapeHeightOffsetMeters), 0.0);
	constexpr int32 MaxSmoothShadingPasses = 8;
	const int32 ShadingPasses = FMath::Clamp(SmoothShadingPasses, 0, MaxSmoothShadingPasses);
	constexpr double DrapeSampleSpacingM = 25.0;

	UE_LOG(LogWaterPlacer, Display, TEXT("========== Water Place START =========="));
	UE_LOG(
		LogWaterPlacer,
		Display,
		TEXT("shp='%s' altitudeField='%s' waterMaterial='%s' meshFolder='%s' clipGround=%s maxOutline=%d smoothMeters=%.1f drape=%s drapeOffsetM=%.2f smoothShadingPasses=%d"),
		*CleanInputPath,
		AltitudeField.IsEmpty() ? TEXT("(0 ellipsoid)") : *AltitudeField,
		CleanMaterialPath.IsEmpty() ? TEXT("(empty, wavy default)") : *CleanMaterialPath,
		*MeshFolder,
		bClipGroundUnderWater ? TEXT("on") : TEXT("off"),
		OutlineVertexCap,
		SmoothMeters,
		bDrape ? TEXT("on") : TEXT("off"),
		DrapeOffsetM,
		ShadingPasses);

	UWorld* World = ResolveEditorWorld(WorldContextObject);
	if (!World)
	{
		Result.Message = TEXT("Could not resolve an editor world. Open a map first.");
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	ACesiumGeoreference* Georeference = WaterCesiumPlacement::FindGeoreference(World);
	if (!Georeference)
	{
		Result.Message = TEXT("No ACesiumGeoreference found in the level.");
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (CleanInputPath.IsEmpty())
	{
		Result.Message = TEXT("ShapefilePath is empty (provide a .shp path).");
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<FWaterShapefilePolygon> Polygons;
	FString ReadError;
	if (!WaterShapefileReader::ReadWaterPolygons(CleanInputPath, AltitudeField, Polygons, ReadError))
	{
		Result.Message = ReadError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	Result.PolygonsRead = Polygons.Num();
	for (const FWaterShapefilePolygon& Poly : Polygons)
	{
		Result.HoleRingsIgnored += Poly.HoleRingCount;
	}

	TArray<ACesium3DTileset*> Tilesets;
	if (bClipGroundUnderWater)
	{
		for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
		{
			if (*It)
			{
				Tilesets.Add(*It);
			}
		}
		if (Tilesets.Num() == 0)
		{
			UE_LOG(
				LogWaterPlacer,
				Warning,
				TEXT("Clip Ground Under Water is on but no ACesium3DTileset is in the level. Water meshes will still spawn."));
		}
	}

	RemovePreviousWaterPlacer(*World);

	ACesium3DTileset* TerrainTileset = nullptr;
	if (bDrape)
	{
		TerrainTileset = WaterCesiumPlacement::FindTerrainTileset(World);
		if (TerrainTileset)
		{
			WaterCesiumPlacement::EnsureTilesetQueryCollision(*TerrainTileset);
			UE_LOG(
				LogWaterPlacer,
				Display,
				TEXT("Draping water onto tileset '%s'."),
				*TerrainTileset->GetActorLabel());
		}
		else
		{
			UE_LOG(
				LogWaterPlacer,
				Warning,
				TEXT("Drape On Cesium Terrain is on but no terrain tileset was found. Shore vertices will use the shapefile altitude field."));
		}
	}

	UMaterialInterface* WaterMaterial = nullptr;
	if (!CleanMaterialPath.IsEmpty())
	{
		WaterMaterial = LoadWaterMaterialFromPath(CleanMaterialPath);
	}
	{
		FString MaterialError;
		if (UMaterialInterface* Prepared = WaterStaticMesh::PrepareMaterialForStaticMesh(
				WaterMaterial, MeshFolder, MaterialError))
		{
			WaterMaterial = Prepared;
		}
		else if (!MaterialError.IsEmpty())
		{
			UE_LOG(LogWaterPlacer, Warning, TEXT("%s"), *MaterialError);
		}
	}

	TArray<TArray<FVector>> LakeWorldPoints;
	LakeWorldPoints.Reserve(Polygons.Num());
	TArray<int32> LakeRecordIds;

	FScopedSlowTask SlowTask(
		static_cast<float>(Polygons.Num() + 1),
		NSLOCTEXT("WaterPlacer", "PlaceProgress", "Placing water meshes..."));
	SlowTask.MakeDialog(true);

	for (int32 Index = 0; Index < Polygons.Num(); ++Index)
	{
		const FWaterShapefilePolygon& Feature = Polygons[Index];
		SlowTask.EnterProgressFrame(1.0f, FText::FromString(FString::Printf(TEXT("Polygon %d / %d"), Index + 1, Polygons.Num())));
		if (SlowTask.ShouldCancel())
		{
			Result.bCancelled = true;
			break;
		}

		TArray<FVector2D> Ring;
		DecimateRing(Feature.OuterRingLonLat, OutlineVertexCap, SmoothMeters, Ring);
		if (bDrape)
		{
			TArray<FVector2D> Densified;
			DensifyClosedRing(Ring, DrapeSampleSpacingM, OutlineVertexCap, Densified);
			Ring = MoveTemp(Densified);
		}
		if (Ring.Num() < 3)
		{
			++Result.PolygonsSkipped;
			continue;
		}

		TArray<FVector> WorldPoints;
		WorldPoints.Reserve(Ring.Num());
		for (const FVector2D& LonLat : Ring)
		{
			if (bDrape)
			{
				bool bHit = false;
				WorldPoints.Add(WaterCesiumPlacement::DrapeLonLatToUnreal(
					*World,
					*Georeference,
					TerrainTileset,
					LonLat.X,
					LonLat.Y,
					Feature.AltitudeM,
					DrapeOffsetM,
					bHit));
				if (bHit)
				{
					++Result.TerrainSamplesHit;
				}
				else
				{
					++Result.TerrainSamplesMissed;
				}
			}
			else
			{
				WorldPoints.Add(WaterCesiumPlacement::LonLatHeightToUnreal(
					*Georeference, LonLat.X, LonLat.Y, Feature.AltitudeM));
			}
		}
		LakeWorldPoints.Add(MoveTemp(WorldPoints));
		LakeRecordIds.Add(Feature.RecordIndex);
	}

	if (Result.bCancelled)
	{
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(TEXT("Cancelled. Elapsed: %.2fs."), Result.ElapsedSeconds);
		UE_LOG(LogWaterPlacer, Warning, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<ACesiumCartographicPolygon*> ClipActors;
	if (bClipGroundUnderWater)
	{
		ClipActors.Reserve(LakeWorldPoints.Num());
	}

	for (int32 I = 0; I < LakeWorldPoints.Num(); ++I)
	{
		const FString MeshLabel = FString::Printf(TEXT("%s_Mesh_%d"), *LabelPrefix, LakeRecordIds[I]);
		if (SpawnWaterMesh(
				*World, LakeWorldPoints[I], WaterMaterial, MeshFolder, MeshLabel, FolderPath, ShadingPasses))
		{
			++Result.WaterMeshesSpawned;
		}
		else
		{
			++Result.PolygonsSkipped;
			UE_LOG(LogWaterPlacer, Warning, TEXT("Failed to spawn water mesh for record %d."), LakeRecordIds[I]);
		}

		if (bClipGroundUnderWater)
		{
			const FString ClipLabel = FString::Printf(TEXT("%s_Clip_%d"), *LabelPrefix, LakeRecordIds[I]);
			if (ACesiumCartographicPolygon* ClipActor = SpawnClipPolygon(
					*World, LakeWorldPoints[I], ClipLabel, FolderPath))
			{
				ClipActors.Add(ClipActor);
				++Result.ClipPolygonsSpawned;
			}
		}
	}

	SlowTask.EnterProgressFrame(1.0f, NSLOCTEXT("WaterPlacer", "ClipTilesets", "Clipping tilesets..."));
	if (bClipGroundUnderWater && ClipActors.Num() > 0)
	{
		for (ACesium3DTileset* Tileset : Tilesets)
		{
			if (!Tileset)
			{
				continue;
			}
			ApplyClipOverlayToTileset(*Tileset, ClipActors);
			++Result.TilesetsClipped;
		}
	}

	World->MarkPackageDirty();

	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	Result.bSuccess = Result.WaterMeshesSpawned > 0;

	if (!Result.bSuccess)
	{
		Result.Message = TEXT("No water StaticMeshActors were spawned.");
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	FString Extra;
	if (bDrape)
	{
		Extra += FString::Printf(
			TEXT(" Draped %d shoreline sample(s) onto Cesium terrain (%d miss)."),
			Result.TerrainSamplesHit,
			Result.TerrainSamplesMissed);
		if (Result.TerrainSamplesMissed > 0 && Result.TerrainSamplesHit == 0)
		{
			Extra += TEXT(" No terrain hits: look at the water in the viewport so Cesium tiles load, enable Create Physics Meshes on Cesium World Terrain, then Place Water again.");
		}
		else if (Result.TerrainSamplesMissed > Result.TerrainSamplesHit)
		{
			Extra += TEXT(" Many draping misses: frame the water so terrain tiles are loaded, then re-run.");
		}
	}
	if (bClipGroundUnderWater)
	{
		Extra += FString::Printf(
			TEXT(" Clipped ground under water on %d tileset(s) (%d clip polygons)."),
			Result.TilesetsClipped,
			Result.ClipPolygonsSpawned);
	}
	if (Result.HoleRingsIgnored > 0)
	{
		Extra += FString::Printf(TEXT(" Ignored %d inner rings (islands in lakes)."), Result.HoleRingsIgnored);
	}
	if (!WaterMaterial)
	{
		Extra += TEXT(" Water material was not loaded; meshes have an empty material slot.");
	}

	Result.Message = FString::Printf(
		TEXT("Spawned %d water StaticMeshActor(s) from %d polygon(s).%s Elapsed: %.2fs."),
		Result.WaterMeshesSpawned,
		Result.PolygonsRead,
		*Extra,
		Result.ElapsedSeconds);
	UE_LOG(LogWaterPlacer, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogWaterPlacer, Display, TEXT("========== Water Place END =========="));
	return Result;
}
