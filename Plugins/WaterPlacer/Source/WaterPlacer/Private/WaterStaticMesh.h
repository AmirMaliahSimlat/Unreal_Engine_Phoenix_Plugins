#pragma once

#include "CoreMinimal.h"

class AStaticMeshActor;
class UMaterialInterface;
class UStaticMesh;
class UWorld;

struct FWaterFlatMesh
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
};

namespace WaterStaticMesh
{
	bool BuildFlatPolygonMesh(
		const TArray<FVector>& LocalRing,
		double MetersPerUv,
		FWaterFlatMesh& OutMesh,
		FString& OutError);

	UMaterialInterface* GetOrCreateWavyWaterMaterial(
		const FString& ContentFolder,
		FString& OutError);

	UMaterialInterface* PrepareMaterialForStaticMesh(
		UMaterialInterface* Source,
		const FString& ContentFolder,
		FString& OutError);

	UStaticMesh* CreatePersistentStaticMesh(
		const FString& PackageFolder,
		const FString& AssetName,
		const FWaterFlatMesh& Mesh,
		UMaterialInterface* Material,
		int32 SmoothShadingPasses,
		FString& OutError);

	AStaticMeshActor* SpawnMeshActor(
		UWorld& World,
		const FVector& Origin,
		UStaticMesh* Mesh,
		UMaterialInterface* Material,
		const FString& Label,
		const FString& FolderPath,
		const FName& Tag);
}
