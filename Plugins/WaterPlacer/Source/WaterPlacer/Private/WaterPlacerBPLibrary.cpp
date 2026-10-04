#include "WaterPlacerPrivatePCH.h"
#include "WaterPlacerBPLibrary.h"

#include "WaterCesiumPlacement.h"
#include "WaterPlacerLog.h"
#include "WaterShapefileReader.h"
#include "WaterStaticMesh.h"
#include "WaterSurface.h"

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

	AStaticMeshActor* SpawnWaterSurfaceMesh(
		UWorld& World,
		const FWaterTin& Tin,
		ACesiumGeoreference& Georeference,
		double HeightOffsetM,
		UMaterialInterface* WaterMaterial,
		const FString& MeshFolder,
		const FString& Label,
		const FString& FolderPath,
		int32 SmoothShadingPasses)
	{
		if (Tin.Vertices.Num() < 3 || Tin.Triangles.Num() < 3)
		{
			return nullptr;
		}

		TArray<FVector> WorldVerts;
		WorldVerts.Reserve(Tin.Vertices.Num());
		FVector Origin = FVector::ZeroVector;
		for (const FWaterSample& S : Tin.Vertices)
		{
			const FVector P = WaterCesiumPlacement::LonLatHeightToUnreal(
				Georeference, S.Lon, S.Lat, S.HeightM + HeightOffsetM);
			WorldVerts.Add(P);
			Origin += P;
		}
		Origin /= static_cast<double>(WorldVerts.Num());

		FWaterFlatMesh Mesh;
		Mesh.Vertices.Reserve(WorldVerts.Num());
		Mesh.UVs.Reserve(WorldVerts.Num());
		Mesh.Normals.Init(FVector::UpVector, WorldVerts.Num());
		const double UvScale = FMath::Max(MetersPerUv * 100.0, 1.0);
		for (const FVector& P : WorldVerts)
		{
			const FVector Local = P - Origin;
			Mesh.Vertices.Add(Local);
			Mesh.UVs.Add(FVector2D(Local.X / UvScale, Local.Y / UvScale));
		}
		Mesh.Triangles = Tin.Triangles;

		FString MeshError;
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
	const FString& MaskShapefilePath,
	const FString& ElevationPointsPath,
	const FString& OptionalAltitudeFieldName,
	const FString& WaterMaterialPath,
	bool bApplyWaterMaterial,
	const FString& MeshContentFolder,
	bool bClipGroundUnderWater,
	int32 MaxOutlineVertices,
	float OutlineSmoothMeters,
	float HeightOffsetMeters,
	float InteriorSpacingMeters,
	float ControlSpacingMeters,
	int32 SmoothShadingPasses,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath)
{
	FWaterPlaceResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString CleanMaskPath = SanitizeFilePath(MaskShapefilePath);
	const FString CleanElevPath = SanitizeFilePath(ElevationPointsPath);
	const FString AltitudeField = OptionalAltitudeFieldName;
	const FString CleanMaterialPath = SanitizeFilePath(WaterMaterialPath);
	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("Water") : ActorLabelPrefix;
	const FString FolderPath = EditorFolderPath;
	const FString MeshFolder = MeshContentFolder.IsEmpty() ? TEXT("/Game/WaterPlacer/Meshes") : MeshContentFolder;
	const double HeightOff = static_cast<double>(HeightOffsetMeters);
	const double InteriorSpacing = static_cast<double>(InteriorSpacingMeters);
	const double ControlSpacing = static_cast<double>(ControlSpacingMeters);
	constexpr int32 MaxSmoothShadingPasses = 8;
	const int32 ShadingPasses = FMath::Clamp(SmoothShadingPasses, 0, MaxSmoothShadingPasses);

	UE_LOG(LogWaterPlacer, Display, TEXT("========== Water Place START =========="));
	UE_LOG(
		LogWaterPlacer,
		Display,
		TEXT("mask='%s' elev='%s' altitudeField='%s' applyMaterial=%s waterMaterial='%s' meshFolder='%s' clipGround=%s maxOutline=%d smoothMeters=%.1f heightOff=%.2f interiorM=%.1f controlM=%.1f smoothShadingPasses=%d"),
		*CleanMaskPath,
		*CleanElevPath,
		AltitudeField.IsEmpty() ? TEXT("(PointZ)") : *AltitudeField,
		bApplyWaterMaterial ? TEXT("on") : TEXT("off"),
		!bApplyWaterMaterial
			? TEXT("(none)")
			: (CleanMaterialPath.IsEmpty() ? TEXT("(empty, wavy default)") : *CleanMaterialPath),
		*MeshFolder,
		bClipGroundUnderWater ? TEXT("on") : TEXT("off"),
		MaxOutlineVertices,
		OutlineSmoothMeters,
		HeightOff,
		InteriorSpacing,
		ControlSpacing,
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

	if (CleanMaskPath.IsEmpty())
	{
		Result.Message = TEXT("MaskShapefilePath is empty (provide a polygon .shp path).");
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}
	if (CleanElevPath.IsEmpty())
	{
		Result.Message = TEXT("ElevationPointsPath is empty (provide a PointZ .shp path).");
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<FWaterShapefilePolygon> Polygons;
	FString ReadError;
	if (!WaterShapefileReader::ReadWaterPolygons(CleanMaskPath, FString(), Polygons, ReadError))
	{
		Result.Message = ReadError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	TArray<FWaterShapefilePoint> ElevationPoints;
	if (!WaterShapefileReader::ReadElevationPoints(CleanElevPath, AltitudeField, ElevationPoints, ReadError))
	{
		Result.Message = ReadError;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	Result.MaskPolygonsRead = Polygons.Num();
	Result.ElevationPointsRead = ElevationPoints.Num();
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

	UMaterialInterface* WaterMaterial = nullptr;
	if (bApplyWaterMaterial)
	{
		if (!CleanMaterialPath.IsEmpty())
		{
			WaterMaterial = LoadWaterMaterialFromPath(CleanMaterialPath);
		}
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
	else
	{
		UE_LOG(LogWaterPlacer, Display, TEXT("Apply Water Material is off; meshes will have an empty material slot."));
	}

	FScopedSlowTask SlowTask(
		static_cast<float>(Polygons.Num() + 1),
		NSLOCTEXT("WaterPlacer", "PlaceProgress", "Placing water meshes..."));
	SlowTask.MakeDialog(true);

	TArray<ACesiumCartographicPolygon*> ClipActors;
	if (bClipGroundUnderWater)
	{
		ClipActors.Reserve(Polygons.Num());
	}

	for (int32 Index = 0; Index < Polygons.Num(); ++Index)
	{
		const FWaterShapefilePolygon& Feature = Polygons[Index];
		SlowTask.EnterProgressFrame(
			1.0f,
			FText::FromString(FString::Printf(TEXT("Lake %d / %d"), Index + 1, Polygons.Num())));
		if (SlowTask.ShouldCancel())
		{
			Result.bCancelled = true;
			break;
		}

		FWaterSurfaceResult Surface;
		FString SurfaceError;
		const bool bBuilt = WaterSurface::BuildInterpolatedLake(
			Feature,
			ElevationPoints,
			MaxOutlineVertices,
			static_cast<double>(OutlineSmoothMeters),
			InteriorSpacing,
			ControlSpacing,
			Surface,
			SurfaceError,
			[&SlowTask](float, const TCHAR* Stage) -> bool
			{
				if (Stage && *Stage)
				{
					SlowTask.EnterProgressFrame(0.0f, FText::FromString(FString(Stage)));
				}
				return !SlowTask.ShouldCancel();
			});
		if (!bBuilt)
		{
			++Result.PolygonsSkipped;
			UE_LOG(
				LogWaterPlacer,
				Warning,
				TEXT("Skipped water mask record %d: %s"),
				Feature.RecordIndex,
				*SurfaceError);
			if (SurfaceError == TEXT("Cancelled."))
			{
				Result.bCancelled = true;
				break;
			}
			continue;
		}

		Result.SurfaceVertices += Surface.Tin.Vertices.Num();
		Result.SurfaceTriangles += Surface.Tin.Triangles.Num() / 3;

		const FString MeshLabel = FString::Printf(TEXT("%s_Mesh_%d"), *LabelPrefix, Feature.RecordIndex);
		if (SpawnWaterSurfaceMesh(
				*World,
				Surface.Tin,
				*Georeference,
				HeightOff,
				WaterMaterial,
				MeshFolder,
				MeshLabel,
				FolderPath,
				ShadingPasses))
		{
			++Result.WaterMeshesSpawned;
		}
		else
		{
			++Result.PolygonsSkipped;
			UE_LOG(LogWaterPlacer, Warning, TEXT("Failed to spawn water mesh for record %d."), Feature.RecordIndex);
		}

		if (bClipGroundUnderWater && Surface.Outline.Num() >= 3)
		{
			TArray<FVector> ClipWorld;
			ClipWorld.Reserve(Surface.Outline.Num());
			for (const FWaterSample& S : Surface.Outline)
			{
				ClipWorld.Add(WaterCesiumPlacement::LonLatHeightToUnreal(
					*Georeference, S.Lon, S.Lat, S.HeightM + HeightOff));
			}
			const FString ClipLabel = FString::Printf(TEXT("%s_Clip_%d"), *LabelPrefix, Feature.RecordIndex);
			if (ACesiumCartographicPolygon* ClipActor = SpawnClipPolygon(
					*World, ClipWorld, ClipLabel, FolderPath))
			{
				ClipActors.Add(ClipActor);
				++Result.ClipPolygonsSpawned;
			}
		}
	}

	if (Result.bCancelled)
	{
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(TEXT("Cancelled. Elapsed: %.2fs."), Result.ElapsedSeconds);
		UE_LOG(LogWaterPlacer, Warning, TEXT("%s"), *Result.Message);
		return Result;
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
	Extra += FString::Printf(
		TEXT(" Surface %d vert(s), %d tri(s) from %d elevation point(s)."),
		Result.SurfaceVertices,
		Result.SurfaceTriangles,
		Result.ElevationPointsRead);
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
	if (!bApplyWaterMaterial)
	{
		Extra += TEXT(" No water material assigned (Apply Water Material off).");
	}
	else if (!WaterMaterial)
	{
		Extra += TEXT(" Water material was not loaded; meshes have an empty material slot.");
	}

	Result.Message = FString::Printf(
		TEXT("Spawned %d water StaticMeshActor(s) from %d mask polygon(s).%s Elapsed: %.2fs."),
		Result.WaterMeshesSpawned,
		Result.MaskPolygonsRead,
		*Extra,
		Result.ElapsedSeconds);
	UE_LOG(LogWaterPlacer, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogWaterPlacer, Display, TEXT("========== Water Place END =========="));
	return Result;
}
