#include "BuildingRoofObjectPlacement.h"

#include "Engine/StaticMesh.h"

namespace
{
	constexpr double ExtraClearanceCm = 50.0;
	constexpr int32 SampleAttempts = 64;

	double DistPointSegXY(const FVector2D& P, const FVector2D& A, const FVector2D& B)
	{
		const FVector2D AB = B - A;
		const double LenSq = AB.SizeSquared();
		if (LenSq < 1.0e-8)
		{
			return FVector2D::Distance(P, A);
		}
		const double T = FMath::Clamp(FVector2D::DotProduct(P - A, AB) / LenSq, 0.0, 1.0);
		return FVector2D::Distance(P, A + AB * T);
	}

	double TriangleAreaXY(const FVector& A, const FVector& B, const FVector& C)
	{
		return 0.5 * FMath::Abs((B.X - A.X) * (C.Y - A.Y) - (C.X - A.X) * (B.Y - A.Y));
	}

	double TriangleInradiusXY(const FVector& A, const FVector& B, const FVector& C)
	{
		const double Area = TriangleAreaXY(A, B, C);
		const double Peri =
			FVector::Dist2D(A, B) + FVector::Dist2D(B, C) + FVector::Dist2D(C, A);
		if (Peri < 1.0e-3)
		{
			return 0.0;
		}
		return (2.0 * Area) / Peri;
	}

	bool BarycentricXY(
		const FVector& A,
		const FVector& B,
		const FVector& C,
		const FVector2D& P,
		double& OutU,
		double& OutV,
		double& OutW)
	{
		const double Den =
			(B.Y - C.Y) * (A.X - C.X) + (C.X - B.X) * (A.Y - C.Y);
		if (FMath::Abs(Den) < 1.0e-12)
		{
			return false;
		}
		OutU = ((B.Y - C.Y) * (P.X - C.X) + (C.X - B.X) * (P.Y - C.Y)) / Den;
		OutV = ((C.Y - A.Y) * (P.X - C.X) + (A.X - C.X) * (P.Y - C.Y)) / Den;
		OutW = 1.0 - OutU - OutV;
		return true;
	}

	FVector InterpolateOnTri(const FRoofPlaceTriangle& Tri, double U, double V, double W)
	{
		return Tri.A * U + Tri.B * V + Tri.C * W;
	}

	bool PointFitsTri(
		const FRoofPlaceTriangle& Tri,
		const FVector2D& P,
		double ClearanceCm)
	{
		double U = 0.0;
		double V = 0.0;
		double W = 0.0;
		if (!BarycentricXY(Tri.A, Tri.B, Tri.C, P, U, V, W))
		{
			return false;
		}
		if (U < 0.0 || V < 0.0 || W < 0.0)
		{
			return false;
		}
		const FVector2D A(Tri.A.X, Tri.A.Y);
		const FVector2D B(Tri.B.X, Tri.B.Y);
		const FVector2D C(Tri.C.X, Tri.C.Y);
		return DistPointSegXY(P, A, B) >= ClearanceCm
			&& DistPointSegXY(P, B, C) >= ClearanceCm
			&& DistPointSegXY(P, C, A) >= ClearanceCm;
	}

	void FootprintCornersXY(
		const FRoofObjectFootprint& Foot,
		const FVector2D& Center,
		float YawDeg,
		TArray<FVector2D>& OutCorners)
	{
		OutCorners.Reset();
		const FRotator YawRot(0.0, YawDeg, 0.0);
		const FVector2D Locals[4] = {
			FVector2D(Foot.LocalMinX, Foot.LocalMinY),
			FVector2D(Foot.LocalMaxX, Foot.LocalMinY),
			FVector2D(Foot.LocalMaxX, Foot.LocalMaxY),
			FVector2D(Foot.LocalMinX, Foot.LocalMaxY)
		};
		for (const FVector2D& Local : Locals)
		{
			const FVector Rotated = YawRot.RotateVector(FVector(Local.X, Local.Y, 0.0));
			OutCorners.Add(Center + FVector2D(Rotated.X, Rotated.Y));
		}
	}

	bool BoxFitsTri(
		const FRoofPlaceTriangle& Tri,
		const TArray<FVector2D>& Corners,
		double ClearanceCm,
		double& OutMinZ)
	{
		OutMinZ = TNumericLimits<double>::Max();
		for (const FVector2D& Corner : Corners)
		{
			if (!PointFitsTri(Tri, Corner, ClearanceCm))
			{
				return false;
			}
			double U = 0.0;
			double V = 0.0;
			double W = 0.0;
			BarycentricXY(Tri.A, Tri.B, Tri.C, Corner, U, V, W);
			OutMinZ = FMath::Min(OutMinZ, static_cast<double>(InterpolateOnTri(Tri, U, V, W).Z));
		}
		return OutMinZ < TNumericLimits<double>::Max() / 4.0;
	}

	bool OverlapsOccupied(
		const FVector2D& Center,
		double Radius,
		const TArray<FPlacedRoofObject2D>& Occupied)
	{
		for (const FPlacedRoofObject2D& Other : Occupied)
		{
			const double MinDist = Radius + Other.RadiusCm + ExtraClearanceCm;
			if ((Center - Other.CenterXY).SizeSquared() < MinDist * MinDist)
			{
				return true;
			}
		}
		return false;
	}
}

FRoofObjectFootprint BuildingRoofObjectPlacement::MakeFootprint(const UStaticMesh& Mesh)
{
	FRoofObjectFootprint Foot;
	const FBox Box = Mesh.GetBounds().GetBox();
	const FVector Size = Box.GetSize();
	Foot.RadiusCm = 0.5 * FVector2D(Size.X, Size.Y).Size();
	Foot.PivotZMin = Box.Min.Z;
	Foot.LocalMinX = Box.Min.X;
	Foot.LocalMinY = Box.Min.Y;
	Foot.LocalMaxX = Box.Max.X;
	Foot.LocalMaxY = Box.Max.Y;
	return Foot;
}

bool BuildingRoofObjectPlacement::TryPlace(
	const TArray<FRoofPlaceTriangle>& WorldTris,
	const FRoofObjectFootprint& Foot,
	const TArray<FPlacedRoofObject2D>& Occupied,
	FRandomStream& Rng,
	FTransform& OutXform,
	FPlacedRoofObject2D& OutOccupied)
{
	const double SizeX = FMath::Abs(Foot.LocalMaxX - Foot.LocalMinX);
	const double SizeY = FMath::Abs(Foot.LocalMaxY - Foot.LocalMinY);
	const double ClearanceCm = FMath::Max(ExtraClearanceCm, 0.2 * FMath::Max(SizeX, SizeY));
	const double NeedInradius = Foot.RadiusCm + ClearanceCm;
	if (WorldTris.Num() == 0 || NeedInradius < 1.0)
	{
		return false;
	}

	TArray<int32> Eligible;
	Eligible.Reserve(WorldTris.Num());
	for (int32 I = 0; I < WorldTris.Num(); ++I)
	{
		if (TriangleInradiusXY(WorldTris[I].A, WorldTris[I].B, WorldTris[I].C) > NeedInradius)
		{
			Eligible.Add(I);
		}
	}
	if (Eligible.Num() == 0)
	{
		return false;
	}

	TArray<FVector2D> Corners;
	for (int32 Attempt = 0; Attempt < SampleAttempts; ++Attempt)
	{
		const FRoofPlaceTriangle& Tri = WorldTris[Eligible[Rng.RandRange(0, Eligible.Num() - 1)]];
		const float R1 = FMath::Sqrt(Rng.FRand());
		const float R2 = Rng.FRand();
		const double U = 1.0 - static_cast<double>(R1);
		const double V = static_cast<double>(R1) * (1.0 - static_cast<double>(R2));
		const double W = 1.0 - U - V;
		const FVector Sample = InterpolateOnTri(Tri, U, V, W);
		const FVector2D Center(Sample.X, Sample.Y);
		const float YawDeg = Rng.FRandRange(0.0f, 360.0f);
		FootprintCornersXY(Foot, Center, YawDeg, Corners);

		double MinRoofZ = 0.0;
		if (!BoxFitsTri(Tri, Corners, ClearanceCm, MinRoofZ))
		{
			continue;
		}
		if (OverlapsOccupied(Center, Foot.RadiusCm, Occupied))
		{
			continue;
		}

		FVector Location(Center.X, Center.Y, MinRoofZ);
		Location.Z -= Foot.PivotZMin;
		OutXform = FTransform(FRotator(0.0, YawDeg, 0.0), Location, FVector::OneVector);
		OutOccupied.CenterXY = Center;
		OutOccupied.RadiusCm = Foot.RadiusCm;
		return true;
	}
	return false;
}
