#pragma once

#include "CoreMinimal.h"

namespace WaterElevation
{
	/**
	 * Samples ellipsoid (or raster) height in meters from a local DTM folder.
	 * Supports Cesium quantized-mesh (.terrain), GeoTIFF (.tif/.tiff), and ESRI ASCII (.asc).
	 * For quantized-mesh, picks the highest zoom tile that covers the point (best LOD on disk).
	 */
	class FSampler
	{
	public:
		bool Load(const FString& FolderOrFile, FString& OutError);
		bool IsLoaded() const { return bLoaded; }
		bool SampleHeightM(double LonDeg, double LatDeg, double& OutHeightM) const;
		FString Describe() const;

	private:
		bool bLoaded = false;
		FString SourceDescription;

		bool LoadQuantizedMeshFolder(const FString& Folder, const TArray<FString>& TerrainFiles, FString& OutError);
		bool LoadRasters(const TArray<FString>& RasterFiles, FString& OutError);

		bool SampleQuantizedMesh(double LonDeg, double LatDeg, double& OutHeightM) const;
		bool SampleRasters(double LonDeg, double LatDeg, double& OutHeightM) const;
	};
}
