#pragma once

#include "CoreMinimal.h"
#include "RoadTriangulate.h"

class AStaticMeshActor;
class UMaterialInterface;
class UStaticMesh;
class UWorld;

namespace RoadStaticMesh
{
	UStaticMesh* CreatePersistentStaticMesh(
		const FString& PackageFolder,
		const FString& AssetName,
		const TArray<FVector>& LocalVertices,
		const TArray<int32>& Triangles,
		UMaterialInterface* Material,
		double MetersPerUv,
		bool bSoftenEdges,
		FString& OutError);

	AStaticMeshActor* SpawnMeshActor(
		UWorld& World,
		const FVector& Origin,
		UStaticMesh* Mesh,
		UMaterialInterface* Material,
		const FString& Label,
		const FString& FolderPath,
		const FName& Tag,
		bool bEnableCollision);
}
