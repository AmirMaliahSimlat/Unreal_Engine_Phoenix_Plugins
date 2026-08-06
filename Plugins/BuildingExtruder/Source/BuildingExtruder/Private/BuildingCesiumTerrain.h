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
	 * Samples ellipsoid heights (meters) at lon/lat points (X=lon, Y=lat) using DTM tileset only.
	 * Forces high-detail Cesium terrain tiles to load near each point, then line-traces.
	 */
	bool SampleHeightsBlocking(
		UWorld& World,
		ACesiumGeoreference& Georeference,
		ACesium3DTileset& TerrainTileset,
		const TArray<FVector>& InLonLatPoints,
		const TArray<int32>& InPointTileIndices,
		bool bEnableDtmLoadTimeout,
		TArray<double>& OutHeightsM,
		TArray<bool>& OutSuccess,
		FString& OutError,
		const FDtmProgressCallback& OnProgress = FDtmProgressCallback(),
		const FDtmShouldCancelCallback& ShouldCancel = FDtmShouldCancelCallback());
}
