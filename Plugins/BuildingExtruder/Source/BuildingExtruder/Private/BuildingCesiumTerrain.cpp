#include "BuildingCesiumTerrain.h"
#include "BuildingCesiumPlacement.h"
#include "BuildingExtruderLog.h"

#include "Cesium3DTileset.h"
#include "CesiumCamera.h"
#include "CesiumCameraManager.h"
#include "CesiumGeoreference.h"
#include "CollisionQueryParams.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Kismet/KismetMathLibrary.h"

ACesium3DTileset* BuildingCesiumTerrain::FindTerrainTileset(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}

	ACesium3DTileset* Fallback = nullptr;
	for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
	{
		ACesium3DTileset* Tileset = *It;
		if (!Tileset)
		{
			continue;
		}
		const FString Label = Tileset->GetActorNameOrLabel();
		if (Label.Contains(TEXT("Terrain"), ESearchCase::IgnoreCase)
			|| Label.Contains(TEXT("DTM"), ESearchCase::IgnoreCase)
			|| Label.Contains(TEXT("DEM"), ESearchCase::IgnoreCase)
			|| Label.Contains(TEXT("WorldTerrain"), ESearchCase::IgnoreCase))
		{
			UE_LOG(LogBuildingExtruder, Display, TEXT("Using Cesium terrain tileset '%s'"), *Label);
			return Tileset;
		}
		Fallback = Tileset;
	}

	if (Fallback)
	{
		UE_LOG(
			LogBuildingExtruder,
			Warning,
			TEXT("No tileset name matched Terrain/DTM/DEM; using '%s'."),
			*Fallback->GetActorNameOrLabel());
	}
	return Fallback;
}

namespace
{
	bool IsHitOnTerrainTileset(const FHitResult& Hit, const ACesium3DTileset& TerrainTileset)
	{
		const AActor* HitActor = Hit.GetActor();
		if (HitActor == &TerrainTileset)
		{
			return true;
		}
		if (const UPrimitiveComponent* Comp = Hit.Component.Get())
		{
			if (Comp->GetOwner() == &TerrainTileset)
			{
				return true;
			}
		}
		return false;
	}

	bool IsBuildingExtruderActor(const AActor* Actor)
	{
		return Actor && Actor->Tags.Contains(FName(TEXT("BuildingExtruder")));
	}

	bool FindTerrainHitAlongRay(
		UWorld& World,
		const FVector& Start,
		const FVector& End,
		const ACesium3DTileset& TerrainTileset,
		const TArray<AActor*>& ExtraIgnoredActors,
		FHitResult& OutHit)
	{
		FCollisionQueryParams Params(FName(TEXT("BuildingExtruderDtmTrace")), /*bTraceComplex*/ true);
		Params.bReturnPhysicalMaterial = false;

		for (AActor* Actor : ExtraIgnoredActors)
		{
			if (Actor)
			{
				Params.AddIgnoredActor(Actor);
			}
		}

		auto TryChannel = [&](ECollisionChannel Channel) -> bool
		{
			TArray<FHitResult> Hits;
			if (!World.LineTraceMultiByChannel(Hits, Start, End, Channel, Params))
			{
				return false;
			}
			for (const FHitResult& Hit : Hits)
			{
				if (IsHitOnTerrainTileset(Hit, TerrainTileset))
				{
					OutHit = Hit;
					return true;
				}
			}
			return false;
		};

		return TryChannel(ECC_Visibility) || TryChannel(ECC_WorldStatic);
	}

	void BuildDtmTraceIgnoreList(
		UWorld& World,
		const ACesium3DTileset& TerrainTileset,
		TArray<AActor*>& OutIgnored)
	{
		OutIgnored.Reset();
		for (TActorIterator<ACesium3DTileset> It(&World); It; ++It)
		{
			ACesium3DTileset* Other = *It;
			if (Other && Other != &TerrainTileset)
			{
				OutIgnored.Add(Other);
			}
		}
		for (TActorIterator<AActor> It(&World); It; ++It)
		{
			AActor* Actor = *It;
			if (IsBuildingExtruderActor(Actor))
			{
				OutIgnored.Add(Actor);
			}
		}
	}

	struct FActorIsolateRestore
	{
		AActor* Actor = nullptr;
		bool bWasCollisionEnabled = true;
		bool bWasHidden = false;
		bool bWasTempHiddenInEditor = false;
		bool bWasTickEnabled = true;
	};

	bool ShouldKeepVisibleDuringDtmSample(const AActor* Actor, const ACesium3DTileset& TerrainTileset)
	{
		if (!Actor || Actor == &TerrainTileset)
		{
			return true;
		}
		// Needed for LLH transforms / refine cameras; they do not stream heavy tiles.
		if (Actor->IsA(ACesiumGeoreference::StaticClass()))
		{
			return true;
		}
		if (Actor->IsA(ACesiumCameraManager::StaticClass()))
		{
			return true;
		}
		return false;
	}

	/**
	 * Hide + disable collision/tick on everything except the DTM tileset (and Cesium georef/camera manager)
	 * so city/imagery tilesets stop competing during DTM refine.
	 */
	void IsolateDtmForSampling(
		UWorld& World,
		const ACesium3DTileset& TerrainTileset,
		TArray<FActorIsolateRestore>& OutRestore)
	{
		OutRestore.Reset();
		for (TActorIterator<AActor> It(&World); It; ++It)
		{
			AActor* Actor = *It;
			if (ShouldKeepVisibleDuringDtmSample(Actor, TerrainTileset))
			{
				continue;
			}

			FActorIsolateRestore Entry;
			Entry.Actor = Actor;
			Entry.bWasCollisionEnabled = Actor->GetActorEnableCollision();
			Entry.bWasHidden = Actor->IsHidden();
			Entry.bWasTickEnabled = Actor->IsActorTickEnabled();
#if WITH_EDITOR
			Entry.bWasTempHiddenInEditor = Actor->IsTemporarilyHiddenInEditor();
			Actor->SetIsTemporarilyHiddenInEditor(true);
#endif
			Actor->SetActorHiddenInGame(true);
			if (Entry.bWasCollisionEnabled)
			{
				Actor->SetActorEnableCollision(false);
			}
			if (Entry.bWasTickEnabled)
			{
				Actor->SetActorTickEnabled(false);
			}
			OutRestore.Add(Entry);
		}
	}

	void RestoreIsolatedActors(TArray<FActorIsolateRestore>& InRestore)
	{
		for (const FActorIsolateRestore& Entry : InRestore)
		{
			if (!Entry.Actor)
			{
				continue;
			}
			Entry.Actor->SetActorEnableCollision(Entry.bWasCollisionEnabled);
			Entry.Actor->SetActorHiddenInGame(Entry.bWasHidden);
			Entry.Actor->SetActorTickEnabled(Entry.bWasTickEnabled);
#if WITH_EDITOR
			Entry.Actor->SetIsTemporarilyHiddenInEditor(Entry.bWasTempHiddenInEditor);
#endif
		}
		InRestore.Reset();
	}

	bool HitToEllipsoidHeightM(
		ACesiumGeoreference& Georeference,
		const FHitResult& Hit,
		double& OutHeightM)
	{
		const FVector LocalPos = Georeference.GetActorTransform().InverseTransformPosition(Hit.ImpactPoint);
		const FVector LLH = Georeference.TransformUnrealPositionToLongitudeLatitudeHeight(LocalPos);
		OutHeightM = LLH.Z;
		return true;
	}

	bool TraceHeightAtLonLat(
		UWorld& World,
		ACesiumGeoreference& Georeference,
		ACesium3DTileset& TerrainTileset,
		const TArray<AActor*>& ExtraIgnoredActors,
		double LonDeg,
		double LatDeg,
		double& OutHeightM)
	{
		constexpr double HighM = 9000.0;
		constexpr double LowM = -500.0;
		const FVector Start = BuildingCesiumPlacement::LonLatHeightToUnreal(Georeference, LonDeg, LatDeg, HighM);
		const FVector End = BuildingCesiumPlacement::LonLatHeightToUnreal(Georeference, LonDeg, LatDeg, LowM);

		FHitResult Hit;
		if (!FindTerrainHitAlongRay(World, Start, End, TerrainTileset, ExtraIgnoredActors, Hit))
		{
			return false;
		}
		return HitToEllipsoidHeightM(Georeference, Hit, OutHeightM);
	}

	/**
	 * Advance Cesium tile loading without calling GEditor->Tick().
	 * Nested editor ticks from a Slate/UMG click crash with dynamic resolution BeginFrame asserts.
	 */
	void PumpCesiumOnly(ACesium3DTileset& Tileset, ACesiumCameraManager& CameraManager, float DeltaSeconds)
	{
		CameraManager.Tick(DeltaSeconds);
		Tileset.Tick(DeltaSeconds);
		FPlatformProcess::Sleep(FMath::Min(DeltaSeconds, 0.02f));
	}

	/**
	 * @return true if load progress reached the practical done threshold; false if timeout / stall / safety / cancel.
	 *
	 * Cesium often reports high % before occlusion / hide-next-frame completes (100% may never
	 * arrive with PumpCesiumOnly). Treat >= 95% as done. Also stop on progress stall so
	 * no-timeout runs cannot hang forever while progress plateaus.
	 */
	bool WaitForHighDetailTiles(
		ACesium3DTileset& Tileset,
		ACesiumCameraManager& CameraManager,
		bool bEnableTimeout,
		double MaxWaitSeconds,
		float DoneProgressPercent,
		float& OutProgress,
		bool& OutCancelled,
		const BuildingCesiumTerrain::FDtmProgressCallback& OnProgress,
		const BuildingCesiumTerrain::FDtmShouldCancelCallback& ShouldCancel)
	{
		DoneProgressPercent = FMath::Clamp(DoneProgressPercent, 1.0f, 99.0f);
		// If load % does not improve for this long, stop waiting (stuck refine / plateau).
		constexpr double StallSeconds = 45.0;
		constexpr float StallEpsilonPercent = 0.25f;
		constexpr double UiUpdateIntervalSeconds = 0.25;

		const double Start = FPlatformTime::Seconds();
		OutProgress = Tileset.GetLoadProgress();
		OutCancelled = false;
		float BestProgress = OutProgress;
		double LastImprovementTime = Start;
		double LastUiUpdateTime = 0.0;

		auto ReportProgress = [&](bool bFinished, bool bReachedTarget)
		{
			if (OnProgress)
			{
				OnProgress(OutProgress, bFinished, bReachedTarget);
			}
		};

		ReportProgress(false, false);

		while (FPlatformTime::Seconds() - Start < MaxWaitSeconds)
		{
			if (ShouldCancel && ShouldCancel())
			{
				OutCancelled = true;
				OutProgress = Tileset.GetLoadProgress();
				ReportProgress(true, false);
				UE_LOG(LogBuildingExtruder, Warning, TEXT("DTM refine cancelled."));
				return false;
			}

			PumpCesiumOnly(Tileset, CameraManager, 0.05f);
			OutProgress = Tileset.GetLoadProgress();

			if (OutProgress > BestProgress + StallEpsilonPercent)
			{
				BestProgress = OutProgress;
				LastImprovementTime = FPlatformTime::Seconds();
			}

			const double Now = FPlatformTime::Seconds();
			if ((Now - LastUiUpdateTime) >= UiUpdateIntervalSeconds)
			{
				LastUiUpdateTime = Now;
				ReportProgress(false, false);
			}

			if (OutProgress >= DoneProgressPercent)
			{
				for (int32 I = 0; I < 15; ++I)
				{
					PumpCesiumOnly(Tileset, CameraManager, 0.05f);
				}
				OutProgress = Tileset.GetLoadProgress();
				ReportProgress(true, true);
				return true;
			}

			// No-timeout path: don't hang forever if Cesium plateaus below the done threshold.
			if (!bEnableTimeout
				&& (FPlatformTime::Seconds() - LastImprovementTime) >= StallSeconds)
			{
				UE_LOG(LogBuildingExtruder, Warning, TEXT("DTM refine stalled; continuing with current tiles."));
				ReportProgress(true, false);
				return false;
			}
		}

		OutProgress = Tileset.GetLoadProgress();
		UE_LOG(LogBuildingExtruder, Warning, TEXT("DTM refine wait ended early (timeout or safety cap)."));
		ReportProgress(true, false);
		return false;
	}

}

bool BuildingCesiumTerrain::SampleHeightsBlocking(
	UWorld& World,
	ACesiumGeoreference& Georeference,
	ACesium3DTileset& TerrainTileset,
	const TArray<FVector>& InLonLatPoints,
	const TArray<int32>& InPointTileIndices,
	bool bEnableDtmLoadTimeout,
	float DoneProgressPercent,
	double TimeoutSecondsOverride,
	TArray<double>& OutHeightsM,
	TArray<bool>& OutSuccess,
	FString& OutError,
	const FDtmProgressCallback& OnProgress,
	const FDtmShouldCancelCallback& ShouldCancel)
{
	OutHeightsM.SetNum(InLonLatPoints.Num());
	OutSuccess.SetNum(InLonLatPoints.Num());
	for (int32 I = 0; I < InLonLatPoints.Num(); ++I)
	{
		OutHeightsM[I] = 0.0;
		OutSuccess[I] = false;
	}

	if (InLonLatPoints.Num() == 0)
	{
		return true;
	}

	(void)InPointTileIndices;

	ACesiumCameraManager* CameraManager = ACesiumCameraManager::GetDefaultCameraManager(&World);
	if (!CameraManager)
	{
		OutError = TEXT("Could not get CesiumCameraManager (needed to force high-detail DTM tiles).");
		return false;
	}

	const double OldSSE = TerrainTileset.MaximumScreenSpaceError;
	TerrainTileset.SetMaximumScreenSpaceError(1.0);

	TArray<int32> CameraIds;
	CameraIds.Reserve(InLonLatPoints.Num());

	TArray<FActorIsolateRestore> IsolateRestore;
	IsolateDtmForSampling(World, TerrainTileset, IsolateRestore);

	auto CleanupRefine = [&]()
	{
		for (const int32 CamId : CameraIds)
		{
			CameraManager->RemoveCamera(CamId);
		}
		TerrainTileset.SetMaximumScreenSpaceError(OldSSE);
		RestoreIsolatedActors(IsolateRestore);
	};

	TArray<AActor*> DtmIgnoredActors;
	BuildDtmTraceIgnoreList(World, TerrainTileset, DtmIgnoredActors);

	// Pass 1: coarse heights for camera placement.
	TArray<double> CoarseHeights;
	CoarseHeights.SetNum(InLonLatPoints.Num());
	for (int32 I = 0; I < InLonLatPoints.Num(); ++I)
	{
		double H = 0.0;
		if (!TraceHeightAtLonLat(
				World,
				Georeference,
				TerrainTileset,
				DtmIgnoredActors,
				InLonLatPoints[I].X,
				InLonLatPoints[I].Y,
				H))
		{
			H = 500.0;
		}
		CoarseHeights[I] = H;
	}

	constexpr double CameraHeightAboveGroundM = 80.0;
	for (int32 I = 0; I < InLonLatPoints.Num(); ++I)
	{
		const double Lon = InLonLatPoints[I].X;
		const double Lat = InLonLatPoints[I].Y;
		const double GroundH = CoarseHeights[I];
		const FVector CamLoc = BuildingCesiumPlacement::LonLatHeightToUnreal(
			Georeference,
			Lon,
			Lat,
			GroundH + CameraHeightAboveGroundM);
		const FVector LookAt = BuildingCesiumPlacement::LonLatHeightToUnreal(
			Georeference,
			Lon,
			Lat,
			GroundH - 5.0);
		const FRotator CamRot = UKismetMathLibrary::FindLookAtRotation(CamLoc, LookAt);

		FCesiumCamera Cam(FVector2D(256.0, 256.0), CamLoc, CamRot, 60.0);
		const int32 CamId = CameraManager->AddCamera(Cam);
		if (CamId >= 0)
		{
			CameraIds.Add(CamId);
		}
	}


	const double DefaultTimeoutSeconds = 8.0 + InLonLatPoints.Num() * 0.05;
	constexpr double NoTimeoutSafetySeconds = 600.0;
	const double TimeoutSeconds =
		(TimeoutSecondsOverride > 0.0) ? TimeoutSecondsOverride : DefaultTimeoutSeconds;
	const double MaxWaitSeconds = bEnableDtmLoadTimeout ? TimeoutSeconds : NoTimeoutSafetySeconds;

	float ProgressAtEnd = 0.0f;
	bool bCancelled = false;
	const bool bReachedDetail = WaitForHighDetailTiles(
		TerrainTileset,
		*CameraManager,
		bEnableDtmLoadTimeout,
		MaxWaitSeconds,
		DoneProgressPercent,
		ProgressAtEnd,
		bCancelled,
		OnProgress,
		ShouldCancel);

	if (bCancelled)
	{
		CleanupRefine();
		OutError = TEXT("Cancelled during DTM sampling.");
		return false;
	}

	(void)bReachedDetail;

	int32 SuccessCount = 0;
	for (int32 I = 0; I < InLonLatPoints.Num(); ++I)
	{
		double HeightM = 0.0;
		const bool bOk = TraceHeightAtLonLat(
			World,
			Georeference,
			TerrainTileset,
			DtmIgnoredActors,
			InLonLatPoints[I].X,
			InLonLatPoints[I].Y,
			HeightM);
		OutSuccess[I] = bOk;
		OutHeightsM[I] = HeightM;
		if (bOk)
		{
			++SuccessCount;
		}
	}

	CleanupRefine();


	if (SuccessCount == 0)
	{
		OutError = TEXT(
			"All DTM line traces missed the terrain tileset after high-detail refine. "
			"Enable collision on the DTM Cesium3DTileset (other tilesets are ignored during sampling).");
		return false;
	}

	return true;
}

