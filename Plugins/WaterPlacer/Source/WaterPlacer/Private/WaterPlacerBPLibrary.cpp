#include "WaterPlacerBPLibrary.h"

#include "WaterCesiumPlacement.h"
#include "WaterPlacerLog.h"
#include "WaterShapefileReader.h"

#include "Algo/Reverse.h"
#include "Cesium3DTileset.h"
#include "CesiumGeoreference.h"
#include "CesiumPolygonRasterOverlay.h"
#include "Components/SplineComponent.h"
#include "Editor.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Internationalization/Internationalization.h"
#include "Materials/MaterialInterface.h"
#include "Misc/PackageName.h"
#include "Misc/ScopedSlowTask.h"
#include "Runtime/Launch/Resources/Version.h"
#include "UObject/SoftObjectPath.h"
#include "WaterBodyComponent.h"
#include "WaterBodyLakeActor.h"
#include "WaterSplineComponent.h"
#include "WaterSplineMetadata.h"
#include "WaterZoneActor.h"

namespace
{
	const FName WaterPlacerTag(TEXT("WaterPlacer"));
	const FName WaterPlacerOverlayName(TEXT("WaterPlacerClip"));
	constexpr int32 MaxSplinePoints = 1024;
	constexpr double DuplicateEpsDeg = 1.0e-10;
	constexpr float DefaultLakeDepthCm = 500.0f;

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

	void EnsureCounterClockwiseXY(TArray<FVector>& Points)
	{
		if (Points.Num() < 3)
		{
			return;
		}

		double Area = 0.0;
		for (int32 I = 0; I < Points.Num(); ++I)
		{
			const FVector& A = Points[I];
			const FVector& B = Points[(I + 1) % Points.Num()];
			Area += (A.X * B.Y - B.X * A.Y);
		}
		if (Area < 0.0)
		{
			Algo::Reverse(Points);
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
			UE_LOG(
				LogWaterPlacer,
				Warning,
				TEXT("WaterMaterialPath is empty. Lakes will keep the Water plugin default material."));
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
			if (UMaterialInterface* Material = Cast<UMaterialInterface>(FSoftObjectPath(Candidate).TryLoad()))
			{
				return Material;
			}
			return nullptr;
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

	void ApplyClosedSpline(UWaterSplineComponent& Spline, const TArray<FVector>& LocalPoints)
	{
		Spline.bSplineHasBeenEdited = true;
		Spline.bInputSplinePointsToConstructionScript = true;
		Spline.WaterSplineDefaults.DefaultDepth = DefaultLakeDepthCm;

		Spline.ClearSplinePoints(false);
		Spline.SetClosedLoop(false, false);
		for (const FVector& P : LocalPoints)
		{
			Spline.AddSplinePoint(P, ESplineCoordinateSpace::Local, false);
		}
		Spline.SetClosedLoop(true, false);
		const int32 Num = Spline.GetNumberOfSplinePoints();
		for (int32 I = 0; I < Num; ++I)
		{
			Spline.SetSplinePointType(I, ESplinePointType::Linear, false);
		}
		Spline.UpdateSpline();
	}

	void RefreshWaterBody(UWaterBodyComponent& Body)
	{
#if ENGINE_MINOR_VERSION >= 2
		FOnWaterBodyChangedParams Params;
		Params.bShapeOrPositionChanged = true;
		Body.UpdateAll(Params);
#else
		Body.OnWaterBodyChanged(true);
#endif
	}

	void AssignOceanMaterial(UWaterBodyComponent& Body, UMaterialInterface* OceanMaterial)
	{
		if (!OceanMaterial)
		{
			return;
		}

		Body.WaterMaterial = OceanMaterial;
		Body.SetWaterAndUnderWaterPostProcessMaterial(OceanMaterial, Body.UnderwaterPostProcessMaterial);
	}

	void AssignWaterZone(UWaterBodyComponent& Body, AWaterZone* Zone)
	{
		if (!Zone)
		{
			return;
		}

#if ENGINE_MINOR_VERSION >= 2
		Body.SetWaterZoneOverride(Zone);
#else
		Body.WaterZoneOverride = Zone;
#endif
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

	AWaterZone* EnsureWaterZone(UWorld& World, const TArray<FVector>& AllWaterPoints, const FString& FolderPath)
	{
		if (AllWaterPoints.Num() == 0)
		{
			return nullptr;
		}

		FBox Bounds(ForceInit);
		for (const FVector& P : AllWaterPoints)
		{
			Bounds += P;
		}
		const FVector Center = Bounds.GetCenter();
		const FVector Extent = Bounds.GetExtent();
		constexpr double PadCm = 50000.0;
		const FVector2D ZoneExtent(
			FMath::Max(Extent.X * 2.0 + PadCm, PadCm),
			FMath::Max(Extent.Y * 2.0 + PadCm, PadCm));

		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		AWaterZone* Zone = World.SpawnActor<AWaterZone>(Center, FRotator::ZeroRotator, Params);
		if (!Zone)
		{
			UE_LOG(LogWaterPlacer, Warning, TEXT("Could not spawn a Water Zone. Enable the Water plugin."));
			return nullptr;
		}

		Zone->Tags.AddUnique(WaterPlacerTag);
		Zone->SetActorLabel(TEXT("WaterZone"));
		if (!FolderPath.IsEmpty())
		{
			Zone->SetFolderPath(FName(*FolderPath));
		}
		Zone->SetActorLocation(Center);
		Zone->SetZoneExtent(ZoneExtent);
		Zone->Modify();
		return Zone;
	}

	AWaterBodyLake* SpawnLake(
		UWorld& World,
		const TArray<FVector>& WorldPoints,
		UMaterialInterface* OceanMaterial,
		AWaterZone* Zone,
		const FString& Label,
		const FString& FolderPath)
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
		EnsureCounterClockwiseXY(LocalPoints);

		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		AWaterBodyLake* Lake = World.SpawnActor<AWaterBodyLake>(
			Origin,
			FRotator::ZeroRotator,
			Params);
		if (!Lake)
		{
			return nullptr;
		}

		Lake->SetActorLabel(Label);
		Lake->Tags.AddUnique(WaterPlacerTag);
		if (!FolderPath.IsEmpty())
		{
			Lake->SetFolderPath(FName(*FolderPath));
		}

		UWaterSplineComponent* Spline = Lake->GetWaterSpline();
		UWaterBodyComponent* Body = Lake->GetWaterBodyComponent();
		if (!Spline || !Body)
		{
			World.DestroyActor(Lake);
			return nullptr;
		}

		ApplyClosedSpline(*Spline, LocalPoints);

		Body->bAffectsLandscape = false;
		AssignOceanMaterial(*Body, OceanMaterial);
		AssignWaterZone(*Body, Zone);
		RefreshWaterBody(*Body);

		Lake->Modify();
		return Lake;
	}
}

FWaterPlaceResult UWaterPlacerBPLibrary::PlaceWaterFromShapefile(
	UObject* WorldContextObject,
	const FString& ShapefilePath,
	const FString& AltitudeFieldName,
	const FString& WaterMaterialPath,
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

	UE_LOG(LogWaterPlacer, Display, TEXT("========== Water Place START =========="));
	UE_LOG(
		LogWaterPlacer,
		Display,
		TEXT("shp='%s' altitudeField='%s' waterMaterial='%s'"),
		*CleanInputPath,
		AltitudeField.IsEmpty() ? TEXT("(0 ellipsoid)") : *AltitudeField,
		CleanMaterialPath.IsEmpty() ? TEXT("(empty)") : *CleanMaterialPath);

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

	RemovePreviousWaterPlacer(*World);

	UMaterialInterface* OceanMaterial = LoadWaterMaterialFromPath(CleanMaterialPath);

	TArray<TArray<FVector>> LakeWorldPoints;
	LakeWorldPoints.Reserve(Polygons.Num());
	TArray<int32> LakeRecordIds;
	TArray<FVector> AllLakePoints;

	FScopedSlowTask SlowTask(
		static_cast<float>(Polygons.Num() + 1),
		NSLOCTEXT("WaterPlacer", "PlaceProgress", "Placing Water Body Lakes..."));
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
		DecimateRing(Feature.OuterRingLonLat, Ring);
		if (Ring.Num() < 3)
		{
			++Result.PolygonsSkipped;
			continue;
		}

		TArray<FVector> WorldPoints;
		WorldPoints.Reserve(Ring.Num());
		for (const FVector2D& LonLat : Ring)
		{
			WorldPoints.Add(WaterCesiumPlacement::LonLatHeightToUnreal(
				*Georeference, LonLat.X, LonLat.Y, Feature.AltitudeM));
		}
		LakeWorldPoints.Add(MoveTemp(WorldPoints));
		LakeRecordIds.Add(Feature.RecordIndex);
		AllLakePoints.Append(LakeWorldPoints.Last());
	}

	if (Result.bCancelled)
	{
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(TEXT("Cancelled. Elapsed: %.2fs."), Result.ElapsedSeconds);
		UE_LOG(LogWaterPlacer, Warning, TEXT("%s"), *Result.Message);
		return Result;
	}

	SlowTask.EnterProgressFrame(1.0f, NSLOCTEXT("WaterPlacer", "SpawnLakes", "Spawning lakes..."));
	AWaterZone* Zone = EnsureWaterZone(*World, AllLakePoints, FolderPath);

	for (int32 I = 0; I < LakeWorldPoints.Num(); ++I)
	{
		const FString LakeLabel = FString::Printf(TEXT("%s_Lake_%d"), *LabelPrefix, LakeRecordIds[I]);
		if (SpawnLake(*World, LakeWorldPoints[I], OceanMaterial, Zone, LakeLabel, FolderPath))
		{
			++Result.WaterBodiesSpawned;
		}
		else
		{
			++Result.PolygonsSkipped;
			UE_LOG(LogWaterPlacer, Warning, TEXT("Failed to spawn Water Body Lake for record %d."), LakeRecordIds[I]);
		}
	}

	World->MarkPackageDirty();

	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	Result.bSuccess = Result.WaterBodiesSpawned > 0;

	if (!Result.bSuccess)
	{
		Result.Message = TEXT("No Water Body Lakes were spawned.");
		UE_LOG(LogWaterPlacer, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	FString Extra;
	if (Result.HoleRingsIgnored > 0)
	{
		Extra += FString::Printf(TEXT(" Ignored %d inner rings (islands in lakes)."), Result.HoleRingsIgnored);
	}
	if (!OceanMaterial)
	{
		if (CleanMaterialPath.IsEmpty())
		{
			Extra += TEXT(" WaterMaterialPath was empty; lakes use the Water plugin default.");
		}
		else
		{
			Extra += FString::Printf(
				TEXT(" Could not load water material '%s'; lakes use the Water plugin default."),
				*CleanMaterialPath);
		}
	}

	Result.Message = FString::Printf(
		TEXT("Spawned %d Water Body Lake(s) from %d polygon(s). Cesium tilesets were not clipped.%s Elapsed: %.2fs."),
		Result.WaterBodiesSpawned,
		Result.PolygonsRead,
		*Extra,
		Result.ElapsedSeconds);
	UE_LOG(LogWaterPlacer, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogWaterPlacer, Display, TEXT("========== Water Place END =========="));
	return Result;
}
