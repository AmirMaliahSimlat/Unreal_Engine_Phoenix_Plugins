#include "RoadPlacerPrivatePCH.h"
#include "RoadTriangulate.h"
#include "RoadPlacerLog.h"

#include "HAL/PlatformTime.h"

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

	bool BoxContains(const FBox2D& B, const FVector2D& P)
	{
		return B.bIsValid
			&& P.X >= B.Min.X && P.X <= B.Max.X
			&& P.Y >= B.Min.Y && P.Y <= B.Max.Y;
	}

	bool PointInMaskOne(const FVector2D& P, const FRoadShapefileMask& Mask)
	{
		if (Mask.Bounds.bIsValid && !BoxContains(Mask.Bounds, P))
		{
			return false;
		}
		if (!PointInRing(P, Mask.Outer.LonLat))
		{
			return false;
		}
		for (const FRoadShapefileRing& Hole : Mask.Holes)
		{
			if (PointInRing(P, Hole.LonLat))
			{
				return false;
			}
		}
		return true;
	}

	double Orient2D(const FVector2D& A, const FVector2D& B, const FVector2D& C)
	{
		return (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
	}

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

	struct FDelTri
	{
		int32 V[3] = { 0, 0, 0 };
		int32 N[3] = { INDEX_NONE, INDEX_NONE, INDEX_NONE };
	};
}

bool RoadTriangulate::PointInMask(const FVector2D& LonLat, const TArray<FRoadShapefileMask>& Masks)
{
	for (const FRoadShapefileMask& Mask : Masks)
	{
		if (PointInMaskOne(LonLat, Mask))
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

	TArray<FDelTri> Tris;
	TArray<uint8> Alive;
	Tris.Reserve(S0 * 2 + 8);
	Alive.Reserve(S0 * 2 + 8);
	{
		FDelTri Super;
		Super.V[0] = S0;
		Super.V[1] = S0 + 1;
		Super.V[2] = S0 + 2;
		if (Orient2D(XYs[Super.V[0]], XYs[Super.V[1]], XYs[Super.V[2]]) < 0.0)
		{
			Swap(Super.V[1], Super.V[2]);
		}
		Tris.Add(Super);
		Alive.Add(1);
	}

	auto Report = [&](float Fraction01, const TCHAR* Stage) -> bool
	{
		if (!Progress)
		{
			return true;
		}
		return Progress(FMath::Clamp(Fraction01, 0.0f, 1.0f), Stage);
	};

	if (!Report(0.0f, TEXT("triangulating")))
	{
		OutError = TEXT("Cancelled.");
		return false;
	}

	TArray<int32> MarkGen;
	int32 Stamp = 1;
	auto EnsureMark = [&]()
	{
		if (MarkGen.Num() < Tris.Num())
		{
			MarkGen.SetNumZeroed(Tris.Num());
		}
	};

	auto PointInTri = [&](int32 Ti, const FVector2D& P) -> bool
	{
		const FDelTri& T = Tris[Ti];
		return Orient2D(XYs[T.V[0]], XYs[T.V[1]], P) >= -1.0e-6
			&& Orient2D(XYs[T.V[1]], XYs[T.V[2]], P) >= -1.0e-6
			&& Orient2D(XYs[T.V[2]], XYs[T.V[0]], P) >= -1.0e-6;
	};

	auto WalkTo = [&](int32 Start, const FVector2D& P) -> int32
	{
		int32 T = (Start >= 0 && Alive.IsValidIndex(Start) && Alive[Start]) ? Start : 0;
		const int32 GuardMax = FMath::Max(Tris.Num() * 2 + 8, 32);
		for (int32 Guard = 0; Guard < GuardMax; ++Guard)
		{
			if (!Alive.IsValidIndex(T) || !Alive[T])
			{
				break;
			}
			const FDelTri& Tri = Tris[T];
			int32 Next = INDEX_NONE;
			if (Orient2D(XYs[Tri.V[1]], XYs[Tri.V[2]], P) < -1.0e-6)
			{
				Next = Tri.N[0];
			}
			else if (Orient2D(XYs[Tri.V[2]], XYs[Tri.V[0]], P) < -1.0e-6)
			{
				Next = Tri.N[1];
			}
			else if (Orient2D(XYs[Tri.V[0]], XYs[Tri.V[1]], P) < -1.0e-6)
			{
				Next = Tri.N[2];
			}
			else
			{
				return T;
			}
			if (Next == INDEX_NONE || !Alive.IsValidIndex(Next) || !Alive[Next])
			{
				break;
			}
			T = Next;
		}
		for (int32 Ti = 0; Ti < Tris.Num(); ++Ti)
		{
			if (Alive[Ti] && PointInTri(Ti, P))
			{
				return Ti;
			}
		}
		return INDEX_NONE;
	};

	int32 Hint = 0;
	double LastReportTime = FPlatformTime::Seconds();
	const int32 InsertStride = FMath::Max(S0 / 200, 32);
	for (int32 Pi = 0; Pi < S0; ++Pi)
	{
		const double Now = FPlatformTime::Seconds();
		if ((Pi % InsertStride) == 0 || Pi + 1 == S0 || (Now - LastReportTime) > 0.25)
		{
			LastReportTime = Now;
			const float Frac = 0.85f * static_cast<float>(Pi + 1) / static_cast<float>(S0);
			if (!Report(Frac, TEXT("triangulating")))
			{
				OutError = TEXT("Cancelled.");
				return false;
			}
		}

		const FVector2D P = XYs[Pi];
		const int32 Start = WalkTo(Hint, P);
		if (Start == INDEX_NONE)
		{
			continue;
		}

		++Stamp;
		if (Stamp == TNumericLimits<int32>::Max())
		{
			MarkGen.Reset();
			Stamp = 1;
		}
		EnsureMark();
		TArray<int32> Cavity;
		Cavity.Reserve(16);
		TArray<int32> Stack;
		Stack.Add(Start);
		MarkGen[Start] = Stamp;
		Cavity.Add(Start);
		while (Stack.Num() > 0)
		{
			const int32 Ti = Stack.Pop();
			const FDelTri& Tri = Tris[Ti];
			for (int32 E = 0; E < 3; ++E)
			{
				const int32 Nbr = Tri.N[E];
				if (Nbr == INDEX_NONE || !Alive.IsValidIndex(Nbr) || !Alive[Nbr])
				{
					continue;
				}
				EnsureMark();
				if (MarkGen[Nbr] == Stamp)
				{
					continue;
				}
				const FDelTri& NT = Tris[Nbr];
				if (CircumcircleContains(XYs[NT.V[0]], XYs[NT.V[1]], XYs[NT.V[2]], P))
				{
					MarkGen[Nbr] = Stamp;
					Cavity.Add(Nbr);
					Stack.Add(Nbr);
				}
			}
		}

		struct FBoundEdge
		{
			int32 A = 0;
			int32 B = 0;
			int32 Outer = INDEX_NONE;
			int32 OuterEdge = INDEX_NONE;
		};
		TArray<FBoundEdge> Boundary;
		Boundary.Reserve(Cavity.Num() + 3);
		for (const int32 Ti : Cavity)
		{
			const FDelTri& Tri = Tris[Ti];
			for (int32 E = 0; E < 3; ++E)
			{
				const int32 Nbr = Tri.N[E];
				const bool bOuter = (Nbr == INDEX_NONE)
					|| !Alive.IsValidIndex(Nbr)
					|| !Alive[Nbr]
					|| MarkGen[Nbr] != Stamp;
				if (!bOuter)
				{
					continue;
				}
				FBoundEdge Edge;
				Edge.A = Tri.V[(E + 1) % 3];
				Edge.B = Tri.V[(E + 2) % 3];
				Edge.Outer = Nbr;
				if (Nbr != INDEX_NONE && Alive.IsValidIndex(Nbr) && Alive[Nbr])
				{
					for (int32 K = 0; K < 3; ++K)
					{
						if (Tris[Nbr].N[K] == Ti)
						{
							Edge.OuterEdge = K;
							break;
						}
					}
				}
				Boundary.Add(Edge);
			}
		}

		for (const int32 Ti : Cavity)
		{
			Alive[Ti] = 0;
		}

		TArray<int32> NewTris;
		NewTris.Reserve(Boundary.Num());
		for (const FBoundEdge& Edge : Boundary)
		{
			int32 A = Edge.A;
			int32 B = Edge.B;
			if (Orient2D(XYs[A], XYs[B], P) < 0.0)
			{
				Swap(A, B);
			}
			FDelTri Neu;
			Neu.V[0] = A;
			Neu.V[1] = B;
			Neu.V[2] = Pi;
			Neu.N[0] = INDEX_NONE;
			Neu.N[1] = INDEX_NONE;
			Neu.N[2] = Edge.Outer;
			const int32 Ni = Tris.Add(Neu);
			Alive.Add(1);
			NewTris.Add(Ni);
			if (Edge.Outer != INDEX_NONE && Edge.OuterEdge != INDEX_NONE
				&& Alive.IsValidIndex(Edge.Outer) && Alive[Edge.Outer])
			{
				Tris[Edge.Outer].N[Edge.OuterEdge] = Ni;
			}
		}

		TMap<uint64, TPair<int32, int32>> EdgeOwner;
		for (const int32 Ni : NewTris)
		{
			FDelTri& Tri = Tris[Ni];
			for (int32 E = 0; E < 3; ++E)
			{
				const int32 VA = Tri.V[(E + 1) % 3];
				const int32 VB = Tri.V[(E + 2) % 3];
				const uint64 Key = EdgeKey(VA, VB);
				if (TPair<int32, int32>* Other = EdgeOwner.Find(Key))
				{
					Tri.N[E] = Other->Key;
					Tris[Other->Key].N[Other->Value] = Ni;
				}
				else
				{
					EdgeOwner.Add(Key, TPair<int32, int32>(Ni, E));
				}
			}
		}

		if (NewTris.Num() > 0)
		{
			Hint = NewTris.Last();
		}
	}

	const double MaxEdge = MaxEdgeMeters;
	OutTin.Vertices = Pts;
	OutTin.Vertices.SetNum(S0);
	OutTin.Triangles.Reserve(Tris.Num() * 3);
	int32 DroppedSuper = 0;
	int32 DroppedLong = 0;
	int32 DroppedOutside = 0;
	int32 Live = 0;
	for (int32 Ti = 0; Ti < Tris.Num(); ++Ti)
	{
		if (Alive[Ti])
		{
			++Live;
		}
	}
	const int32 FilterStride = FMath::Max(Live / 50, 32);
	int32 SeenLive = 0;
	LastReportTime = FPlatformTime::Seconds();
	for (int32 Ti = 0; Ti < Tris.Num(); ++Ti)
	{
		if (!Alive[Ti])
		{
			continue;
		}
		++SeenLive;
		const double Now = FPlatformTime::Seconds();
		if ((SeenLive % FilterStride) == 0 || SeenLive == Live || (Now - LastReportTime) > 0.25)
		{
			LastReportTime = Now;
			const float Frac = 0.85f + 0.15f * static_cast<float>(SeenLive)
				/ static_cast<float>(FMath::Max(Live, 1));
			if (!Report(Frac, TEXT("clipping to mask")))
			{
				OutError = TEXT("Cancelled.");
				return false;
			}
		}
		const FDelTri& Tri = Tris[Ti];
		if (Tri.V[0] >= S0 || Tri.V[1] >= S0 || Tri.V[2] >= S0)
		{
			++DroppedSuper;
			continue;
		}
		const FRoadSample& A = Pts[Tri.V[0]];
		const FRoadSample& B = Pts[Tri.V[1]];
		const FRoadSample& C = Pts[Tri.V[2]];
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
		// Walking Delaunay keeps CCW in east/north. 1.7 Bowyer–Watson emitted CW;
		// ReverseAllPolygonFacing then made the top visible from above. Match 1.7.
		int32 I0 = Tri.V[0];
		int32 I1 = Tri.V[1];
		int32 I2 = Tri.V[2];
		if (Orient2D(XYs[I0], XYs[I1], XYs[I2]) > 0.0)
		{
			Swap(I1, I2);
		}
		OutTin.Triangles.Add(I0);
		OutTin.Triangles.Add(I1);
		OutTin.Triangles.Add(I2);
	}

	UE_LOG(
		LogRoadPlacer,
		Display,
		TEXT("TIN: %d insert(s), %d live tri(s), kept %d (dropped super=%d longEdge=%d outsideMask=%d)."),
		S0,
		Live,
		OutTin.Triangles.Num() / 3,
		DroppedSuper,
		DroppedLong,
		DroppedOutside);

	if (OutTin.Triangles.Num() < 3)
	{
		OutError = TEXT("No triangles remained inside the road mask.");
		return false;
	}
	return true;
}
