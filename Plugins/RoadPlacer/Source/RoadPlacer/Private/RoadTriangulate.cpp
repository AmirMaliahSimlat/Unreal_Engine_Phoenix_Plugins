#include "RoadTriangulate.h"

namespace
{
	bool PointInRing(const FVector2D& P, const TArray<FVector2D>& Ring)
	{
		const int32 N = Ring.Num();
		if (N < 3)
		{
			return false;
		}
		bool bInside = false;
		for (int32 I = 0, J = N - 1; I < N; J = I++)
		{
			const FVector2D& A = Ring[I];
			const FVector2D& B = Ring[J];
			const bool bIntersect = ((A.Y > P.Y) != (B.Y > P.Y))
				&& (P.X < (B.X - A.X) * (P.Y - A.Y) / (B.Y - A.Y + 1.0e-30) + A.X);
			if (bIntersect)
			{
				bInside = !bInside;
			}
		}
		return bInside;
	}

	struct FTri
	{
		int32 A = 0;
		int32 B = 0;
		int32 C = 0;
	};

	bool CircumcircleContains(
		const FVector2D& A,
		const FVector2D& B,
		const FVector2D& C,
		const FVector2D& P)
	{
		const double Ax = A.X - P.X;
		const double Ay = A.Y - P.Y;
		const double Bx = B.X - P.X;
		const double By = B.Y - P.Y;
		const double Cx = C.X - P.X;
		const double Cy = C.Y - P.Y;
		const double Det = (Ax * Ax + Ay * Ay) * (Bx * Cy - Cx * By)
			- (Bx * Bx + By * By) * (Ax * Cy - Cx * Ay)
			+ (Cx * Cx + Cy * Cy) * (Ax * By - Bx * Ay);
		const double Ori = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
		return Ori * Det > 0.0;
	}

	uint64 EdgeKey(int32 I, int32 J)
	{
		const int32 A = FMath::Min(I, J);
		const int32 B = FMath::Max(I, J);
		return (static_cast<uint64>(static_cast<uint32>(A)) << 32) | static_cast<uint32>(B);
	}
}

bool RoadTriangulate::PointInMask(const FVector2D& LonLat, const TArray<FRoadShapefileMask>& Masks)
{
	for (const FRoadShapefileMask& Mask : Masks)
	{
		if (!PointInRing(LonLat, Mask.Outer.LonLat))
		{
			continue;
		}
		bool bInHole = false;
		for (const FRoadShapefileRing& Hole : Mask.Holes)
		{
			if (PointInRing(LonLat, Hole.LonLat))
			{
				bInHole = true;
				break;
			}
		}
		if (!bInHole)
		{
			return true;
		}
	}
	return false;
}

double RoadTriangulate::EdgeMeters(const FRoadSample& A, const FRoadSample& B)
{
	const double MidLat = 0.5 * (A.Lat + B.Lat);
	const double MetersLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(MidLat)), 0.05);
	const double Dx = (A.Lon - B.Lon) * MetersLon;
	const double Dy = (A.Lat - B.Lat) * 110540.0;
	return FMath::Sqrt(Dx * Dx + Dy * Dy);
}

bool RoadTriangulate::BuildTin(
	const TArray<FRoadSample>& Samples,
	const TArray<FRoadShapefileMask>& Masks,
	double MaxEdgeMeters,
	FRoadTin& OutTin,
	FString& OutError)
{
	OutTin = FRoadTin();
	if (Samples.Num() < 3)
	{
		OutError = TEXT("Need at least 3 elevation samples to triangulate.");
		return false;
	}

	TArray<FRoadSample> Pts;
	Pts.Reserve(Samples.Num());
	TSet<uint64> Seen;
	auto QuantKey = [](double Lon, double Lat) -> uint64
	{
		const int32 Qx = FMath::RoundToInt(Lon * 1.0e7);
		const int32 Qy = FMath::RoundToInt(Lat * 1.0e7);
		return (static_cast<uint64>(static_cast<uint32>(Qx)) << 32) | static_cast<uint32>(Qy);
	};
	for (const FRoadSample& S : Samples)
	{
		const uint64 Key = QuantKey(S.Lon, S.Lat);
		if (Seen.Contains(Key))
		{
			continue;
		}
		Seen.Add(Key);
		Pts.Add(S);
	}
	if (Pts.Num() < 3)
	{
		OutError = TEXT("All elevation samples collapsed to fewer than 3 unique points.");
		return false;
	}

	double MinX = Pts[0].Lon, MaxX = Pts[0].Lon, MinY = Pts[0].Lat, MaxY = Pts[0].Lat;
	for (const FRoadSample& P : Pts)
	{
		MinX = FMath::Min(MinX, P.Lon);
		MaxX = FMath::Max(MaxX, P.Lon);
		MinY = FMath::Min(MinY, P.Lat);
		MaxY = FMath::Max(MaxY, P.Lat);
	}
	const double Dx = FMath::Max(MaxX - MinX, 1.0e-6);
	const double Dy = FMath::Max(MaxY - MinY, 1.0e-6);
	const FRoadSample Super0{ MinX - Dx * 10.0, MinY - Dy * 10.0, 0.0 };
	const FRoadSample Super1{ MinX + Dx * 0.5, MaxY + Dy * 10.0, 0.0 };
	const FRoadSample Super2{ MaxX + Dx * 10.0, MinY - Dy * 10.0, 0.0 };
	const int32 S0 = Pts.Num();
	Pts.Add(Super0);
	Pts.Add(Super1);
	Pts.Add(Super2);

	TArray<FTri> Tris;
	Tris.Add(FTri{ S0, S0 + 1, S0 + 2 });

	auto XY = [&](int32 I) -> FVector2D
	{
		return FVector2D(Pts[I].Lon, Pts[I].Lat);
	};

	for (int32 Pi = 0; Pi < S0; ++Pi)
	{
		const FVector2D P(Pts[Pi].Lon, Pts[Pi].Lat);
		TArray<int32> Bad;
		for (int32 T = 0; T < Tris.Num(); ++T)
		{
			const FTri& Tri = Tris[T];
			if (CircumcircleContains(XY(Tri.A), XY(Tri.B), XY(Tri.C), P))
			{
				Bad.Add(T);
			}
		}

		TMap<uint64, int32> EdgeCount;
		TArray<TPair<int32, int32>> Edges;
		for (const int32 Ti : Bad)
		{
			const FTri& Tri = Tris[Ti];
			const int32 E[3][2] = { {Tri.A, Tri.B}, {Tri.B, Tri.C}, {Tri.C, Tri.A} };
			for (int32 Eidx = 0; Eidx < 3; ++Eidx)
			{
				const uint64 Key = EdgeKey(E[Eidx][0], E[Eidx][1]);
				int32& Count = EdgeCount.FindOrAdd(Key);
				if (Count == 0)
				{
					Edges.Add(TPair<int32, int32>(E[Eidx][0], E[Eidx][1]));
				}
				++Count;
			}
		}

		Bad.Sort();
		for (int32 B = Bad.Num() - 1; B >= 0; --B)
		{
			Tris.RemoveAtSwap(Bad[B]);
		}

		for (const TPair<int32, int32>& E : Edges)
		{
			if (EdgeCount.FindRef(EdgeKey(E.Key, E.Value)) != 1)
			{
				continue;
			}
			Tris.Add(FTri{ E.Key, E.Value, Pi });
		}
	}

	const double MaxEdge = FMath::Max(MaxEdgeMeters, 0.25);
	OutTin.Vertices = Pts;
	OutTin.Vertices.SetNum(S0);
	OutTin.Triangles.Reserve(Tris.Num() * 3);
	for (const FTri& Tri : Tris)
	{
		if (Tri.A >= S0 || Tri.B >= S0 || Tri.C >= S0)
		{
			continue;
		}
		const FRoadSample& A = Pts[Tri.A];
		const FRoadSample& B = Pts[Tri.B];
		const FRoadSample& C = Pts[Tri.C];
		if (EdgeMeters(A, B) > MaxEdge || EdgeMeters(B, C) > MaxEdge || EdgeMeters(C, A) > MaxEdge)
		{
			continue;
		}
		const FVector2D Centroid((A.Lon + B.Lon + C.Lon) / 3.0, (A.Lat + B.Lat + C.Lat) / 3.0);
		if (!PointInMask(Centroid, Masks))
		{
			continue;
		}
		OutTin.Triangles.Add(Tri.A);
		OutTin.Triangles.Add(Tri.B);
		OutTin.Triangles.Add(Tri.C);
	}

	if (OutTin.Triangles.Num() < 3)
	{
		OutError = TEXT("No triangles remained inside the road mask (try a larger Max Edge Meters).");
		return false;
	}
	return true;
}
