#include "RoadTriangulate.h"
#include "RoadPlacerLog.h"

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
	FString& OutError,
	TFunction<bool(float, const TCHAR*)> Progress)
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
	const double OriginLon = 0.5 * (MinX + MaxX);
	const double OriginLat = 0.5 * (MinY + MaxY);
	const double MetersLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(OriginLat)), 0.05);
	const double MetersLat = 110540.0;
	auto ToXY = [&](double Lon, double Lat) -> FVector2D
	{
		return FVector2D((Lon - OriginLon) * MetersLon, (Lat - OriginLat) * MetersLat);
	};

	TArray<FVector2D> XYs;
	XYs.Reserve(Pts.Num() + 3);
	for (const FRoadSample& P : Pts)
	{
		XYs.Add(ToXY(P.Lon, P.Lat));
	}

	double MinMX = XYs[0].X, MaxMX = XYs[0].X, MinMY = XYs[0].Y, MaxMY = XYs[0].Y;
	for (const FVector2D& P : XYs)
	{
		MinMX = FMath::Min(MinMX, P.X);
		MaxMX = FMath::Max(MaxMX, P.X);
		MinMY = FMath::Min(MinMY, P.Y);
		MaxMY = FMath::Max(MaxMY, P.Y);
	}
	const double Dx = FMath::Max(MaxMX - MinMX, 1.0);
	const double Dy = FMath::Max(MaxMY - MinMY, 1.0);
	const int32 S0 = Pts.Num();
	XYs.Add(FVector2D(MinMX - Dx * 10.0, MinMY - Dy * 10.0));
	XYs.Add(FVector2D(MinMX + Dx * 0.5, MaxMY + Dy * 10.0));
	XYs.Add(FVector2D(MaxMX + Dx * 10.0, MinMY - Dy * 10.0));
	Pts.Add(FRoadSample{ OriginLon, OriginLat, 0.0 });
	Pts.Add(FRoadSample{ OriginLon, OriginLat, 0.0 });
	Pts.Add(FRoadSample{ OriginLon, OriginLat, 0.0 });

	TArray<FTri> Tris;
	Tris.Add(FTri{ S0, S0 + 1, S0 + 2 });

	auto Report = [&](float Fraction01, const TCHAR* Stage) -> bool
	{
		if (!Progress)
		{
			return true;
		}
		return Progress(FMath::Clamp(Fraction01, 0.0f, 1.0f), Stage);
	};

	const int32 InsertStride = FMath::Max(S0 / 50, 64);
	if (!Report(0.0f, TEXT("triangulating")))
	{
		OutError = TEXT("Cancelled.");
		return false;
	}

	for (int32 Pi = 0; Pi < S0; ++Pi)
	{
		if ((Pi % InsertStride) == 0 || Pi + 1 == S0)
		{
			const float Frac = 0.85f * static_cast<float>(Pi + 1) / static_cast<float>(S0);
			if (!Report(Frac, TEXT("triangulating")))
			{
				OutError = TEXT("Cancelled.");
				return false;
			}
		}
		const FVector2D P = XYs[Pi];
		TArray<int32> Bad;
		for (int32 T = 0; T < Tris.Num(); ++T)
		{
			const FTri& Tri = Tris[T];
			if (CircumcircleContains(XYs[Tri.A], XYs[Tri.B], XYs[Tri.C], P))
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

	const double MaxEdge = MaxEdgeMeters;
	OutTin.Vertices = Pts;
	OutTin.Vertices.SetNum(S0);
	OutTin.Triangles.Reserve(Tris.Num() * 3);
	int32 DroppedSuper = 0;
	int32 DroppedLong = 0;
	int32 DroppedOutside = 0;
	const int32 FilterStride = FMath::Max(Tris.Num() / 25, 32);
	for (int32 Ti = 0; Ti < Tris.Num(); ++Ti)
	{
		if ((Ti % FilterStride) == 0 || Ti + 1 == Tris.Num())
		{
			const float Frac = 0.85f + 0.15f * static_cast<float>(Ti + 1)
				/ static_cast<float>(FMath::Max(Tris.Num(), 1));
			if (!Report(Frac, TEXT("clipping to mask")))
			{
				OutError = TEXT("Cancelled.");
				return false;
			}
		}
		const FTri& Tri = Tris[Ti];
		if (Tri.A >= S0 || Tri.B >= S0 || Tri.C >= S0)
		{
			++DroppedSuper;
			continue;
		}
		const FRoadSample& A = Pts[Tri.A];
		const FRoadSample& B = Pts[Tri.B];
		const FRoadSample& C = Pts[Tri.C];
		if (MaxEdge > 0.0
			&& (EdgeMeters(A, B) > MaxEdge || EdgeMeters(B, C) > MaxEdge || EdgeMeters(C, A) > MaxEdge))
		{
			++DroppedLong;
			continue;
		}
		const FVector2D Centroid((A.Lon + B.Lon + C.Lon) / 3.0, (A.Lat + B.Lat + C.Lat) / 3.0);
		if (!PointInMask(Centroid, Masks))
		{
			++DroppedOutside;
			continue;
		}
		OutTin.Triangles.Add(Tri.A);
		OutTin.Triangles.Add(Tri.B);
		OutTin.Triangles.Add(Tri.C);
	}

	UE_LOG(
		LogRoadPlacer,
		Verbose,
		TEXT("TIN: %d Delaunay tri(s), kept %d (dropped super=%d longEdge=%d outsideMask=%d, maxEdgeM=%.1f)."),
		Tris.Num(),
		OutTin.Triangles.Num() / 3,
		DroppedSuper,
		DroppedLong,
		DroppedOutside,
		MaxEdge);

	if (OutTin.Triangles.Num() < 3)
	{
		OutError = TEXT("No triangles remained inside the road mask.");
		return false;
	}
	return true;
}
