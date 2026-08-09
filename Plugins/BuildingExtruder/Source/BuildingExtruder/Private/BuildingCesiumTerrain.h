#pragma once

#include "CoreMinimal.h"

class ACesiumGeoreference;
class ACesium3DTileset;
class UWorld;

namespace BuildingCesiumTerrain
{
	/** Prefers tilesets named Terrain/DTM/DEM; otherwise first Cesium3DTileset. */
	ACesium3DTileset* FindTerrainTileset(UWorld* World);

	/**
	 * Optional UI callback while waiting for DTM refine.
	 * @param LoadProgressPercent Cesium GetLoadProgress() (0–100) for the current refine request.
	 * @param bWaitFinished True when this wait ended.
	 * @param bReachedTarget True if the internal done threshold was reached.
	 */
	using FDtmProgressCallback = TFunction<void(float LoadProgressPercent, bool bWaitFinished, bool bReachedTarget)>;

	/** Optional cancel poll (e.g. SlowTask.ShouldCancel). */
	using FDtmShouldCancelCallback = TFunction<bool()>;

	/**
	 * Destroys in-memory Cesium tiles via RefreshTileset and pumps ticks so the next
	 * SampleHeightsBlocking starts from an empty tileset (HTTP cache may still speed downloads).
	 */
	void ColdReloadTileset(ACesium3DTileset& TerrainTileset, UWorld* World);

	/**
	 * Refines DTM near the given footprint vertices and measures how long until the floor
	 * min height stops changing (within EpsilonM for HoldSeconds). Used to calibrate
	 * per-tile sample timeouts.
	 *
	 * @param OutTimeToStableSeconds Elapsed seconds when stability was reached, or MaxProbeSeconds if not.
	 * @param OutHitMaxProbe True if MaxProbeSeconds was reached without stability.
	 */
	bool MeasureTimeToStableFloorHeight(
		UWorld& World,
		ACesiumGeoreference& Georeference,
		ACesium3DTileset& TerrainTileset,
		const TArray<FVector>& InLonLatPoints,
		double MaxProbeSeconds,
		double HoldSeconds,
		double EpsilonM,
		double& OutTimeToStableSeconds,
		double& OutStableFloorHeightM,
		bool& OutHitMaxProbe,
		FString& OutError,
		const FDtmShouldCancelCallback& ShouldCancel = FDtmShouldCancelCallback());

	/**
	 * Samples ellipsoid heights (meters) at lon/lat points (X=lon, Y=lat) using DTM tileset only.
	 * Forces high-detail Cesium terrain tiles to load near each point, then line-traces.
	 *
	 * @param DoneProgressPercent Cesium GetLoadProgress() threshold to treat refine as done (e.g. 95).
	 * @param TimeoutSecondsOverride If > 0 and bEnableDtmLoadTimeout, use this wait cap (seconds).
	 *        If <= 0 and timeout enabled, use default 8 + 0.05*pointCount.
	 */
	bool SampleHeightsBlocking(
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
		const FDtmProgressCallback& OnProgress = FDtmProgressCallback(),
		const FDtmShouldCancelCallback& ShouldCancel = FDtmShouldCancelCallback());
}
