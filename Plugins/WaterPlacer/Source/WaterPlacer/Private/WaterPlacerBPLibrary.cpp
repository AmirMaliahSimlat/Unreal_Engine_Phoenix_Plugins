#include "WaterPlacerBPLibrary.h"

#include "WaterCesiumPlacement.h"
#include "WaterPlacerLog.h"
#include "WaterShapefileReader.h"

#include "Cesium3DTileset.h"
#include "CesiumCartographicPolygon.h"
#include "CesiumGeoreference.h"
#include "CesiumPolygonRasterOverlay.h"
#include "Components/SplineComponent.h"
#include "Editor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Internationalization/Internationalization.h"
#include "Misc/ScopedSlowTask.h"
#include "WaterBodyComponent.h"
#include "WaterBodyLakeActor.h"
#include "WaterSplineComponent.h"
#include "WaterZoneActor.h"

namespace
{
	const FName WaterPlacerTag(TEXT("WaterPlacer"));
	const FName WaterPlacerOverlayName(TEXT("WaterPlacerClip"));
	constexpr int32 MaxSplinePoints = 256;
	constexpr double DuplicateEpsDeg = 1.0e-10;

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

	void DecimateRing(const TArray<FVector2D>& In, TArray<FVector2D>& Out)
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

		if (Unique.Num() <= MaxSplinePoints)
		{
			Out = MoveTemp(Unique);
			return;
		}

		Out.Reserve(MaxSplinePoints);
		for (int32 I = 0; I < MaxSplinePoints; ++I)
		{
			const int32 Src = (I * (Unique.Num() - 1)) / (MaxSplinePoints - 1);
			Out.Add(Unique[Src]);
		}
		if (Out.Num() >= 2 && Out[0].Equals(Out.Last(), DuplicateEpsDeg))
		{
			Out.Pop();
		}
	}

	void ApplyLinearClosedSpline(USplineComponent& Spline, const TArray<FVector>& WorldPoints)
	{
		Spline.ClearSplinePoints(false);
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
		const TArray<ACesiumCartographicPolygon*>& ClipPolygons,
		bool bInvertSelection)
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
		Overlay->InvertSelection = bInvertSelection;
		Overlay->ExcludeSelectedTiles = true;
		Tileset.Modify();
		Overlay->Activate(true);
		Overlay->Refresh();
		Tileset.RefreshTileset();
	}

	ACesiumCartographicPolygon* SpawnClipPolygon(
		UWorld& World,
		ACesiumGeoreference& Georeference,
		const TArray<FVector2D>& RingLonLat,
		double AltitudeM,
		const FString& Label,
		const FString& FolderPath)
	{
		TArray<FVector> WorldPoints;
		WorldPoints.Reserve(RingLonLat.Num());
		for (const FVector2D& LonLat : RingLonLat)
		{
			WorldPoints.Add(WaterCesiumPlacement::LonLatHeightToUnreal(
				Georeference, LonLat.X, LonLat.Y, AltitudeM));
		}
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

		ApplyLinearClosedSpline(*Poly->Polygon, WorldPoints);
		Poly->SetActorLabel(Label);
		Poly->Tags.AddUnique(WaterPlacerTag);
		if (!FolderPath.IsEmpty())
		{
			Poly->SetFolderPath(FName(*FolderPath));
		}
		Poly->Modify();
		return Poly;
	}

	AWaterBodyLake* SpawnLake(
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
		AWaterBodyLake* Lake = World.SpawnActor<AWaterBodyLake>(
			WorldPoints[0],
			FRotator::ZeroRotator,
			Params);
		if (!Lake)
		{
			return nullptr;
		}

		UWaterSplineComponent* Spline = Lake->GetWaterSpline();
		if (!Spline)
		{
			World.DestroyActor(Lake);
			return nullptr;
		}

		ApplyLinearClosedSpline(*Spline, WorldPoints);
		if (UWaterBodyComponent* Body = Lake->GetWaterBodyComponent())
		{
			Body->OnWaterBodyChanged(true);
		}

		Lake->SetActorLabel(Label);
		Lake->Tags.AddUnique(WaterPlacerTag);
		if (!FolderPath.IsEmpty())
		{
			Lake->SetFolderPath(FName(*FolderPath));
		}
		Lake->Modify();
		return Lake;
	}

	void EnsureWaterZone(UWorld& World, const TArray<FVector>& AllWaterPoints, const FString& FolderPath)
	{
		if (AllWaterPoints.Num() == 0)
		{
			return;
		}

		FBox Bounds(ForceInit);
		for (const FVector& P : AllWaterPoints)
		{
			Bounds += P;
		}
		const FVector Center = Bounds.GetCenter();
		const FVector Extent = Bounds.GetExtent();
		constexpr double PadCm = 100000.0;
		const FVector2D ZoneExtent(
			FMath::Max(Extent.X * 2.0 + PadCm, PadCm),
			FMath::Max(Extent.Y * 2.0 + PadCm, PadCm));

		AWaterZone* Zone = nullptr;
		for (TActorIterator<AWaterZone> It(&World); It; ++It)
		{
			if (*It && (*It)->ActorHasTag(WaterPlacerTag))
			{
				Zone = *It;
				break;
			}
		}
		if (!Zone)
		{
			for (TActorIterator<AWaterZone> It(&World); It; ++It)
			{
				if (*It)
				{
					Zone = *It;
					break;
				}
			}
		}
		if (!Zone)
		{
			FActorSpawnParameters Params;
			Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
			Zone = World.SpawnActor<AWaterZone>(Center, FRotator::ZeroRotator, Params);
			if (Zone)
			{
				Zone->Tags.AddUnique(WaterPlacerTag);
				Zone->SetActorLabel(TEXT("WaterPlacer_WaterZone"));
				if (!FolderPath.IsEmpty())
				{
					Zone->SetFolderPath(FName(*FolderPath));
				}
			}
		}
		if (!Zone)
		{
			UE_LOG(LogWaterPlacer, Warning, TEXT("Could not spawn or find a Water Zone. Enable the Water plugin and add a Water Zone if lakes do not render."));
			return;
		}

		Zone->SetActorLocation(Center);
		Zone->SetZoneExtent(ZoneExtent);
		Zone->Modify();
	}
}

FWaterPlaceResult UWaterPlacerBPLibrary::PlaceWaterFromShapefile(
	UObject* WorldContextObject,
	const FString& ShapefilePath,
	const FString& AltitudeFieldName,
	bool bInvertSelection,
	bool bPlaceWaterBodies,
	const FString& ActorLabelPrefix,
	const FString& EditorFolderPath)
{
	FWaterPlaceResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString CleanInputPath = SanitizeFilePath(ShapefilePath);
	const FString AltitudeField = AltitudeFieldName;
	const bool bSpawnLakes = bPlaceWaterBodies && !bInvertSelection;
	const FString LabelPrefix = ActorLabelPrefix.IsEmpty() ? TEXT("Water") : ActorLabelPrefix;
	const FString FolderPath = EditorFolderPath;

	UE_LOG(LogWaterPlacer, Display, TEXT("========== Water Place START =========="));
	UE_LOG(
		LogWaterPlacer,
		Display,
		TEXT("shp='%s' altitudeField='%s' invert=%s placeLakes=%s"),
		*CleanInputPath,
		AltitudeField.IsEmpty() ? TEXT("(0 ellipsoid)") : *AltitudeField,
		bInvertSelection ? TEXT("on (keep inside / clip sea)") : TEXT("off (clip lakes)"),
		bSpawnLakes ? TEXT("on") : TEXT("off"));

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
	for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
	{
		if (*It)
		{
			Tilesets.Add(*It);
		}
	}
	if (Tilesets.Num() == 0)
	{
		Result.Message = TEXT("No ACesium3DTileset in the level. Add Cesium World Terrain (and any 3D tiles) first.");
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	RemovePreviousWaterPlacer(*World);

	FScopedSlowTask SlowTask(
		static_cast<float>(Polygons.Num()),
		NSLOCTEXT("WaterPlacer", "PlaceProgress", "Placing water clip polygons..."));
	SlowTask.MakeDialog(true);

	TArray<ACesiumCartographicPolygon*> ClipActors;
	ClipActors.Reserve(Polygons.Num());
	TArray<FVector> AllLakePoints;

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
		DecimateRing(Feature.OuterRingLonLat, Ring);
		if (Ring.Num() < 3)
		{
			++Result.PolygonsSkipped;
			continue;
		}

		const FString ClipLabel = FString::Printf(TEXT("%s_Clip_%d"), *LabelPrefix, Feature.RecordIndex);
		ACesiumCartographicPolygon* ClipActor = SpawnClipPolygon(
			*World, *Georeference, Ring, Feature.AltitudeM, ClipLabel, FolderPath);
		if (!ClipActor)
		{
			++Result.PolygonsSkipped;
			continue;
		}
		ClipActors.Add(ClipActor);
		++Result.ClipPolygonsSpawned;

		if (bSpawnLakes)
		{
			TArray<FVector> LakePoints;
			LakePoints.Reserve(Ring.Num());
			for (const FVector2D& LonLat : Ring)
			{
				LakePoints.Add(WaterCesiumPlacement::LonLatHeightToUnreal(
					*Georeference, LonLat.X, LonLat.Y, Feature.AltitudeM));
			}
			const FString LakeLabel = FString::Printf(TEXT("%s_Lake_%d"), *LabelPrefix, Feature.RecordIndex);
			if (AWaterBodyLake* Lake = SpawnLake(*World, LakePoints, LakeLabel, FolderPath))
			{
				++Result.WaterBodiesSpawned;
				AllLakePoints.Append(LakePoints);
			}
			else
			{
				UE_LOG(LogWaterPlacer, Warning, TEXT("Failed to spawn Water Body Lake for record %d."), Feature.RecordIndex);
			}
		}
	}

	if (bSpawnLakes)
	{
		EnsureWaterZone(*World, AllLakePoints, FolderPath);
	}

	for (ACesium3DTileset* Tileset : Tilesets)
	{
		if (!Tileset)
		{
			continue;
		}
		ApplyClipOverlayToTileset(*Tileset, ClipActors, bInvertSelection);
		++Result.TilesetsClipped;
	}

	World->MarkPackageDirty();

	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	Result.bSuccess = !Result.bCancelled && Result.ClipPolygonsSpawned > 0;

	if (Result.bCancelled)
	{
		Result.Message = FString::Printf(
			TEXT("Cancelled. Clip polygons=%d lakes=%d tilesets=%d. Elapsed: %.2fs."),
			Result.ClipPolygonsSpawned,
			Result.WaterBodiesSpawned,
			Result.TilesetsClipped,
			Result.ElapsedSeconds);
		UE_LOG(LogWaterPlacer, Warning, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (!Result.bSuccess)
	{
		Result.Message = TEXT("No clip polygons were spawned.");
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	FString InvertNote;
	if (bInvertSelection)
	{
		InvertNote = TEXT(" Invert Selection is on: tileset outside the polygons is hidden (islands kept). Water Body Lakes were not spawned on the land.");
	}
	if (Result.HoleRingsIgnored > 0)
	{
		InvertNote += FString::Printf(TEXT(" Ignored %d inner rings (islands in lakes)."), Result.HoleRingsIgnored);
	}

	Result.Message = FString::Printf(
		TEXT("Clipped %d polygon(s) on %d tileset(s), spawned %d Water Body Lake(s)%s. Elapsed: %.2fs."),
		Result.ClipPolygonsSpawned,
		Result.TilesetsClipped,
		Result.WaterBodiesSpawned,
		*InvertNote,
		Result.ElapsedSeconds);
	UE_LOG(LogWaterPlacer, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogWaterPlacer, Display, TEXT("========== Water Place END =========="));
	return Result;
}
