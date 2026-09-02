#pragma once

#include "CoreMinimal.h"

class ACesiumGeoreference;
class ACesium3DTileset;
class UWorld;

namespace WaterCesiumPlacement
{
	ACesiumGeoreference* FindGeoreference(UWorld* World);

	/** Prefers Cesium World Terrain / DTM; skips photogrammetry and OSM buildings. */
	ACesium3DTileset* FindTerrainTileset(UWorld* World);

	void EnsureTilesetQueryCollision(ACesium3DTileset& Tileset);

	FVector LonLatHeightToUnreal(
		ACesiumGeoreference& Georeference,
		double LonDeg,
		double LatDeg,
		double HeightM);

	/**
	 * Samples the loaded Cesium quantized-mesh along the ellipsoid normal at lon/lat.
	 * On miss, uses FallbackHeightM (shapefile altitude / ellipsoid).
	 */
	FVector DrapeLonLatToUnreal(
		UWorld& World,
		ACesiumGeoreference& Georeference,
		ACesium3DTileset* Terrain,
		double LonDeg,
		double LatDeg,
		double FallbackHeightM,
		double OffsetM,
		bool& bHitTerrain);
}
