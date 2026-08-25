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

	FVector2D ClosestPointOnSegXY(const FVector2D& P, const FVector2D& A, const FVector2D& B)
	{
		const FVector2D AB = B - A;
		const double LenSq = AB.SizeSquared();
		if (LenSq < 1.0e-8)
		{
			return A;
		}
		const double T = FMath::Clamp(FVector2D::DotProduct(P - A, AB) / LenSq, 0.0, 1.0);
		return A + AB * T;
	}

	bool ClosestFootprintEdge(
		const FVector2D& P,
		const TArray<FVector2D>& Ring,
		FVector2D& OutDir,
		FVector2D& OutClosest)
	{
		if (Ring.Num() < 2)
		{
			return false;
		}
		double BestDist = TNumericLimits<double>::Max();
		bool bFound = false;
		const int32 N = Ring.Num();
		const int32 SegCount = N >= 3 ? N : (N - 1);
		for (int32 I = 0; I < SegCount; ++I)
		{
			const FVector2D& A = Ring[I];
			const FVector2D& B = Ring[(I + 1) % N];
			const FVector2D Delta = B - A;
			if (Delta.SizeSquared() < 1.0)
			{
				continue;
			}
			const FVector2D Closest = ClosestPointOnSegXY(P, A, B);
			const double Dist = FVector2D::Distance(P, Closest);
			if (Dist < BestDist)
			{
				BestDist = Dist;
				OutDir = Delta.GetSafeNormal();
				OutClosest = Closest;
				bFound = true;
			}
		}
		return bFound;
	}

	/**
	 * Local +X along the eave/footprint edge, local +Y into the roof (away from that edge).
	 * Picks the 180° so the same side of the mesh always faces the edge.
	 */
	float YawAlongEdgeFacingInward(
		const FVector2D& Center,
		const FVector2D& AlignDirHint,
		const TArray<FVector2D>& FootprintXY)
	{
		FVector2D EdgeDir = FVector2D::ZeroVector;
		FVector2D Closest = Center;
		if (!ClosestFootprintEdge(Center, FootprintXY, EdgeDir, Closest))
		{
			EdgeDir = AlignDirHint;
		}
		if (AlignDirHint.SizeSquared() > 1.0e-8)
		{
			EdgeDir = AlignDirHint.GetSafeNormal();
		}
		if (EdgeDir.SizeSquared() < 1.0e-12)
		{
			return 0.0f;
		}

		FVector2D Inward = Center - Closest;
		if (Inward.SizeSquared() < 1.0)
		{
			Inward = FVector2D(-EdgeDir.Y, EdgeDir.X);
		}
		// Local +Y after yaw(EdgeDir) is (-EdgeDir.Y, EdgeDir.X). Flip so it points into the roof.
		if (EdgeDir.X * Inward.Y - EdgeDir.Y * Inward.X < 0.0)
		{
			EdgeDir = -EdgeDir;
		}
		return FMath::RadiansToDegrees(FMath::Atan2(EdgeDir.Y, EdgeDir.X));
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
	const TArray<FVector2D>& FootprintXY,
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

		const float YawDeg = YawAlongEdgeFacingInward(Center, Tri.AlignDirXY, FootprintXY);
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
