#include "WaterCesiumPlacement.h"
#include "WaterPlacerLog.h"

#include "Cesium3DTileset.h"
#include "CesiumGeoreference.h"
#include "CollisionQueryParams.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "UObject/UnrealType.h"

ACesiumGeoreference* WaterCesiumPlacement::FindGeoreference(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}

	if (ACesiumGeoreference* DefaultGeo = ACesiumGeoreference::GetDefaultGeoreference(World))
	{
		return DefaultGeo;
	}

	for (TActorIterator<ACesiumGeoreference> It(World); It; ++It)
	{
		if (*It)
		{
			return *It;
		}
	}
	return nullptr;
}

ACesium3DTileset* WaterCesiumPlacement::FindTerrainTileset(UWorld* World)
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
		const FString Label = Tileset->GetActorLabel();
		if (Label.Contains(TEXT("Google")) || Label.Contains(TEXT("Photogrammetry")) || Label.Contains(TEXT("OSM")))
		{
			continue;
		}
		if (Label.Contains(TEXT("Terrain")) || Label.Contains(TEXT("World Terrain")) || Label.Contains(TEXT("DTM")))
		{
			return Tileset;
		}
		if (!Fallback)
		{
			Fallback = Tileset;
		}
	}
	return Fallback;
}

void WaterCesiumPlacement::EnsureTilesetQueryCollision(ACesium3DTileset& Tileset)
{
	Tileset.SetActorEnableCollision(true);

	if (FProperty* Prop = Tileset.GetClass()->FindPropertyByName(TEXT("CreatePhysicsMeshes")))
	{
		if (FBoolProperty* BoolProp = CastField<FBoolProperty>(Prop))
		{
			if (!BoolProp->GetPropertyValue_InContainer(&Tileset))
			{
				BoolProp->SetPropertyValue_InContainer(&Tileset, true);
				Tileset.RefreshTileset();
				UE_LOG(
					LogWaterPlacer,
					Display,
					TEXT("Enabled CreatePhysicsMeshes on '%s' so shoreline draping can hit the quantized mesh."),
					*Tileset.GetActorLabel());
			}
		}
	}

	TArray<UPrimitiveComponent*> Comps;
	Tileset.GetComponents<UPrimitiveComponent>(Comps);
	for (UPrimitiveComponent* Comp : Comps)
	{
		if (!Comp)
		{
			continue;
		}
		Comp->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		Comp->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
		Comp->SetCollisionResponseToChannel(ECC_WorldStatic, ECR_Block);
	}
}

FVector WaterCesiumPlacement::LonLatHeightToUnreal(
	ACesiumGeoreference& Georeference,
	double LonDeg,
	double LatDeg,
	double HeightM)
{
	const FVector LLH(LonDeg, LatDeg, HeightM);
	const FVector LocalPos = Georeference.TransformLongitudeLatitudeHeightPositionToUnreal(LLH);
	return Georeference.GetActorTransform().TransformPosition(LocalPos);
}

namespace
{
	bool TraceTilesetMesh(
		ACesium3DTileset& Terrain,
		const FVector& Start,
		const FVector& End,
		FHitResult& OutHit)
	{
		TArray<UPrimitiveComponent*> Comps;
		Terrain.GetComponents<UPrimitiveComponent>(Comps);
		double BestDistSq = TNumericLimits<double>::Max();
		bool bHit = false;
		FCollisionQueryParams Params(FName(TEXT("WaterPlacerDrapeComp")), true);
		for (UPrimitiveComponent* Comp : Comps)
		{
			if (!Comp || !Comp->IsRegistered())
			{
				continue;
			}
			FHitResult Hit;
			if (Comp->LineTraceComponent(Hit, Start, End, Params))
			{
				const double DistSq = FVector::DistSquared(Start, Hit.ImpactPoint);
				if (DistSq < BestDistSq)
				{
					BestDistSq = DistSq;
					OutHit = Hit;
					bHit = true;
				}
			}
		}
		return bHit;
	}
}

FVector WaterCesiumPlacement::DrapeLonLatToUnreal(
	UWorld& World,
	ACesiumGeoreference& Georeference,
	ACesium3DTileset* Terrain,
	double LonDeg,
	double LatDeg,
	double FallbackHeightM,
	double OffsetM,
	bool& bHitTerrain)
{
	bHitTerrain = false;
	const FVector Start = LonLatHeightToUnreal(Georeference, LonDeg, LatDeg, FallbackHeightM + 8000.0);
	const FVector End = LonLatHeightToUnreal(Georeference, LonDeg, LatDeg, FallbackHeightM - 3000.0);
	const FVector Up = (Start - End).GetSafeNormal();

	FHitResult Hit;
	if (Terrain && TraceTilesetMesh(*Terrain, Start, End, Hit))
	{
		bHitTerrain = true;
		return Hit.ImpactPoint + Up * (OffsetM * 100.0);
	}

	FCollisionQueryParams Params(FName(TEXT("WaterPlacerDrape")), true);
	Params.bTraceComplex = true;
	for (TActorIterator<AStaticMeshActor> It(&World); It; ++It)
	{
		if (*It)
		{
			Params.AddIgnoredActor(*It);
		}
	}
	if (Terrain)
	{
		for (TActorIterator<ACesium3DTileset> It(&World); It; ++It)
		{
			if (*It && *It != Terrain)
			{
				Params.AddIgnoredActor(*It);
			}
		}
	}

	if (World.LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Params) && Hit.bBlockingHit)
	{
		bHitTerrain = true;
		return Hit.ImpactPoint + Up * (OffsetM * 100.0);
	}
	if (World.LineTraceSingleByChannel(Hit, Start, End, ECC_WorldStatic, Params) && Hit.bBlockingHit)
	{
		bHitTerrain = true;
		return Hit.ImpactPoint + Up * (OffsetM * 100.0);
	}

	return LonLatHeightToUnreal(Georeference, LonDeg, LatDeg, FallbackHeightM + OffsetM);
}
