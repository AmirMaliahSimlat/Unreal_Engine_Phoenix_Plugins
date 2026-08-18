#include "BuildingExtrudeUtils.h"
#include "BuildingExtruderLog.h"

namespace
{
	double SignedArea2XY(const TArray<FVector>& Ring)
	{
		double Area2 = 0.0;
		const int32 N = Ring.Num();
		for (int32 I = 0; I < N; ++I)
		{
			const FVector& A = Ring[I];
			const FVector& B = Ring[(I + 1) % N];
			Area2 += A.X * B.Y - B.X * A.Y;
		}
		return Area2;
	}

	/** Positive = CCW around Up (viewed from the Up tip / from above if Up is sky). */
	double SignedAreaAboutUp(const TArray<FVector>& Ring, const FVector& Up)
	{
		double Area = 0.0;
		if (Ring.Num() < 3)
		{
			return 0.0;
		}
		const FVector Origin = Ring[0];
		for (int32 I = 0; I < Ring.Num(); ++I)
		{
			const FVector A = Ring[I] - Origin;
			const FVector B = Ring[(I + 1) % Ring.Num()] - Origin;
			Area += FVector::DotProduct(FVector::CrossProduct(A, B), Up);
		}
		return Area;
	}

	/**
	 * Outward (away from polygon interior) for a boundary edge, including C/H recesses.
	 * Uses edge × Up and ring winding in the Up plane — not the vertex centroid.
	 */
	FVector EdgeOutwardDir(const FVector& From, const FVector& To, const FVector& Up, double AreaAboutUp)
	{
		FVector Edge = To - From;
		Edge -= Up * FVector::DotProduct(Edge, Up);
		FVector Outward = FVector::CrossProduct(Edge, Up);
		if (AreaAboutUp < 0.0)
		{
			Outward = -Outward;
		}
		Outward = Outward.GetSafeNormal();
		if (Outward.IsNearlyZero())
		{
			Outward = FVector::CrossProduct(To - From, Up).GetSafeNormal();
			if (AreaAboutUp < 0.0)
			{
				Outward = -Outward;
			}
		}
		return Outward;
	}

	bool PointInTriangleXY(const FVector& P, const FVector& A, const FVector& B, const FVector& C)
	{
		const double D1 = (P.X - B.X) * (A.Y - B.Y) - (A.X - B.X) * (P.Y - B.Y);
		const double D2 = (P.X - C.X) * (B.Y - C.Y) - (B.X - C.X) * (P.Y - C.Y);
		const double D3 = (P.X - A.X) * (C.Y - A.Y) - (C.X - A.X) * (P.Y - A.Y);
		const bool HasNeg = (D1 < 0.0) || (D2 < 0.0) || (D3 < 0.0);
		const bool HasPos = (D1 > 0.0) || (D2 > 0.0) || (D3 > 0.0);
		return !(HasNeg && HasPos);
	}

	bool IsConvexVertexXY(const FVector& Prev, const FVector& Curr, const FVector& Next, bool bCCW)
	{
		const double Cross = (Curr.X - Prev.X) * (Next.Y - Curr.Y) - (Curr.Y - Prev.Y) * (Next.X - Curr.X);
		return bCCW ? (Cross > 0.0) : (Cross < 0.0);
	}

	void StripClosingDuplicate(TArray<FVector>& Ring)
	{
		if (Ring.Num() >= 2 && Ring[0].Equals(Ring.Last(), 1.0e-3))
		{
			Ring.Pop();
		}
	}

	/** Drops tiny edges and near-collinear verts so the hip skeleton does not explode. */
	void CleanHipRingXY(TArray<FVector>& Ring)
	{
		StripClosingDuplicate(Ring);
		bool bChanged = true;
		for (int32 Pass = 0; bChanged && Ring.Num() >= 3 && Pass < 8; ++Pass)
		{
			bChanged = false;
			for (int32 I = 0; I < Ring.Num() && Ring.Num() >= 3; )
			{
				const int32 J = (I + 1) % Ring.Num();
				if (FVector::Dist2D(Ring[I], Ring[J]) < 1.0)
				{
					Ring.RemoveAt(J);
					bChanged = true;
					continue;
				}
				++I;
			}
			for (int32 I = 0; I < Ring.Num() && Ring.Num() >= 3; )
			{
				const int32 N = Ring.Num();
				const int32 IPrev = (I + N - 1) % N;
				const int32 INext = (I + 1) % N;
				const FVector2D U(Ring[I].X - Ring[IPrev].X, Ring[I].Y - Ring[IPrev].Y);
				const FVector2D V(Ring[INext].X - Ring[I].X, Ring[INext].Y - Ring[I].Y);
				const double LenU = U.Size();
				const double LenV = V.Size();
				if (LenU < 1.0e-6 || LenV < 1.0e-6)
				{
					Ring.RemoveAt(I);
					bChanged = true;
					continue;
				}
				const double Cross = (U.X * V.Y - U.Y * V.X) / (LenU * LenV);
				const double Dot = (U.X * V.X + U.Y * V.Y) / (LenU * LenV);
				if (FMath::Abs(Cross) < 1.0e-2 && Dot > 0.0)
				{
					Ring.RemoveAt(I);
					bChanged = true;
					continue;
				}
				++I;
			}
		}
	}

	struct FHipBounds
	{
		FVector2D Min = FVector2D::ZeroVector;
		FVector2D Max = FVector2D::ZeroVector;
		double Pad = 0.0;
		double MaxD = 0.0;
	};

	FHipBounds MakeHipBounds(const TArray<FVector>& Ring)
	{
		FHipBounds B;
		B.Min = FVector2D(Ring[0].X, Ring[0].Y);
		B.Max = B.Min;
		for (int32 I = 1; I < Ring.Num(); ++I)
		{
			B.Min.X = FMath::Min(B.Min.X, static_cast<double>(Ring[I].X));
			B.Min.Y = FMath::Min(B.Min.Y, static_cast<double>(Ring[I].Y));
			B.Max.X = FMath::Max(B.Max.X, static_cast<double>(Ring[I].X));
			B.Max.Y = FMath::Max(B.Max.Y, static_cast<double>(Ring[I].Y));
		}
		const double Dx = B.Max.X - B.Min.X;
		const double Dy = B.Max.Y - B.Min.Y;
		const double Diag = FMath::Sqrt(Dx * Dx + Dy * Dy);
		B.Pad = FMath::Max(Diag * 0.05, 100.0);
		const double MinHalf = 0.5 * FMath::Min(Dx, Dy);
		double Area2 = 0.0;
		double Peri = 0.0;
		for (int32 I = 0; I < Ring.Num(); ++I)
		{
			const FVector& A = Ring[I];
			const FVector& C = Ring[(I + 1) % Ring.Num()];
			Area2 += A.X * C.Y - C.X * A.Y;
			Peri += FVector::Dist2D(A, C);
		}
		const double Inr = (Peri > 1.0e-6) ? (FMath::Abs(Area2) / Peri) : 0.0;
		B.MaxD = FMath::Max(FMath::Max(MinHalf, Inr) * 1.5, 50.0);
		return B;
	}

	bool IsFinitePoint(const FVector2D& P)
	{
		return FMath::IsFinite(P.X) && FMath::IsFinite(P.Y);
	}

	bool IsPointInHipBounds(const FHipBounds& B, const FVector2D& P)
	{
		return IsFinitePoint(P)
			&& P.X >= B.Min.X - B.Pad && P.X <= B.Max.X + B.Pad
			&& P.Y >= B.Min.Y - B.Pad && P.Y <= B.Max.Y + B.Pad;
	}

	bool EarClipTriangulate(TArray<FVector> Ring, TArray<int32>& OutIndices, FString& OutError)
	{
		OutIndices.Reset();
		StripClosingDuplicate(Ring);
		const int32 N0 = Ring.Num();
		if (N0 < 3)
		{
			OutError = TEXT("Polygon has fewer than 3 vertices.");
			return false;
		}

		const bool bCCW = SignedArea2XY(Ring) > 0.0;
		TArray<int32> V;
		V.Reserve(N0);
		for (int32 I = 0; I < N0; ++I)
		{
			V.Add(I);
		}

		int32 Guard = 0;
		const int32 GuardMax = N0 * N0 + 8;
		while (V.Num() > 3 && Guard++ < GuardMax)
		{
			bool bClipped = false;
			for (int32 I = 0; I < V.Num(); ++I)
			{
				const int32 IPrev = (I + V.Num() - 1) % V.Num();
				const int32 INext = (I + 1) % V.Num();
				const int32 A = V[IPrev];
				const int32 B = V[I];
				const int32 C = V[INext];

				if (!IsConvexVertexXY(Ring[A], Ring[B], Ring[C], bCCW))
				{
					continue;
				}

				bool bEar = true;
				for (int32 J = 0; J < V.Num(); ++J)
				{
					const int32 P = V[J];
					if (P == A || P == B || P == C)
					{
						continue;
					}
					if (PointInTriangleXY(Ring[P], Ring[A], Ring[B], Ring[C]))
					{
						bEar = false;
						break;
					}
				}
				if (!bEar)
				{
					continue;
				}

				OutIndices.Add(A);
				OutIndices.Add(B);
				OutIndices.Add(C);
				V.RemoveAt(I);
				bClipped = true;
				break;
			}
			if (!bClipped)
			{
				OutError = TEXT("Ear clipping failed (self-intersecting or degenerate polygon).");
				return false;
			}
		}

		if (V.Num() == 3)
		{
			OutIndices.Add(V[0]);
			OutIndices.Add(V[1]);
			OutIndices.Add(V[2]);
			return true;
		}

		OutError = TEXT("Ear clipping did not finish.");
		return false;
	}

	void AddTriWithUV(
		FExtrudedPrismMesh& Mesh,
		const FVector& A,
		const FVector& B,
		const FVector& C,
		const FVector2D& UvA,
		const FVector2D& UvB,
		const FVector2D& UvC,
		const FVector& DesiredOutwardDir,
		int32 MaterialSlotIndex = 0)
	{
		// Ensure winding so the geometric normal faces DesiredOutwardDir.
		FVector PA = A;
		FVector PB = B;
		FVector PC = C;
		FVector2D UVA = UvA;
		FVector2D UVB = UvB;
		FVector2D UVC = UvC;

		FVector Normal = FVector::CrossProduct(PB - PA, PC - PA).GetSafeNormal();
		if (Normal.IsNearlyZero())
		{
			Normal = DesiredOutwardDir.GetSafeNormal();
			if (Normal.IsNearlyZero())
			{
				Normal = FVector::UpVector;
			}
		}
		else if (!DesiredOutwardDir.IsNearlyZero() && FVector::DotProduct(Normal, DesiredOutwardDir) < 0.0)
		{
			Swap(PB, PC);
			Swap(UVB, UVC);
			Normal = -Normal;
		}

		const int32 BaseIdx = Mesh.Vertices.Num();
		Mesh.Vertices.Add(PA);
		Mesh.Vertices.Add(PB);
		Mesh.Vertices.Add(PC);
		Mesh.Normals.Add(Normal);
		Mesh.Normals.Add(Normal);
		Mesh.Normals.Add(Normal);
		Mesh.UVs.Add(UVA);
		Mesh.UVs.Add(UVB);
		Mesh.UVs.Add(UVC);
		Mesh.Triangles.Add(BaseIdx);
		Mesh.Triangles.Add(BaseIdx + 1);
		Mesh.Triangles.Add(BaseIdx + 2);
		Mesh.TriangleMaterialIndices.Add(MaterialSlotIndex);
	}

	void AppendMesh(FExtrudedPrismMesh& Combined, const FExtrudedPrismMesh& Part)
	{
		const int32 VertexOffset = Combined.Vertices.Num();
		Combined.Vertices.Append(Part.Vertices);
		Combined.Normals.Append(Part.Normals);
		Combined.UVs.Append(Part.UVs);
		Combined.TriangleMaterialIndices.Append(Part.TriangleMaterialIndices);
		Combined.Triangles.Reserve(Combined.Triangles.Num() + Part.Triangles.Num());
		for (const int32 Index : Part.Triangles)
		{
			Combined.Triangles.Add(Index + VertexOffset);
		}
	}

	void AddQuadWithUV(
		FExtrudedPrismMesh& Mesh,
		const FVector& A,
		const FVector& B,
		const FVector& C,
		const FVector& D,
		const FVector2D& UvA,
		const FVector2D& UvB,
		const FVector2D& UvC,
		const FVector2D& UvD,
		const FVector& DesiredOutwardDir)
	{
		AddTriWithUV(Mesh, A, B, C, UvA, UvB, UvC, DesiredOutwardDir);
		AddTriWithUV(Mesh, A, C, D, UvA, UvC, UvD, DesiredOutwardDir);
	}

	FVector2D InwardNormalXY(const FVector& From, const FVector& To, bool bCCW)
	{
		const double Dx = To.X - From.X;
		const double Dy = To.Y - From.Y;
		const double Len = FMath::Sqrt(Dx * Dx + Dy * Dy);
		if (Len < 1.0e-9)
		{
			return FVector2D::ZeroVector;
		}
		if (bCCW)
		{
			return FVector2D(-Dy / Len, Dx / Len);
		}
		return FVector2D(Dy / Len, -Dx / Len);
	}

	FVector2D VertexOffsetVelXY(const FVector& Prev, const FVector& Curr, const FVector& Next, bool bCCW)
	{
		const FVector2D N0 = InwardNormalXY(Prev, Curr, bCCW);
		const FVector2D N1 = InwardNormalXY(Curr, Next, bCCW);
		if (N0.IsNearlyZero() && N1.IsNearlyZero())
		{
			return FVector2D::ZeroVector;
		}
		if (N0.IsNearlyZero())
		{
			return N1;
		}
		if (N1.IsNearlyZero())
		{
			return N0;
		}
		const double Denom = 1.0 + FVector2D::DotProduct(N0, N1);
		if (FMath::Abs(Denom) < 1.0e-6)
		{
			return (N0 + N1) * 0.5;
		}
		return (N0 + N1) / Denom;
	}

	double RingAreaAbsXY(const TArray<FVector>& Ring)
	{
		return FMath::Abs(SignedArea2XY(Ring)) * 0.5;
	}

	double RingPerimeterXY(const TArray<FVector>& Ring)
	{
		double Peri = 0.0;
		for (int32 I = 0; I < Ring.Num(); ++I)
		{
			Peri += FVector::Dist2D(Ring[I], Ring[(I + 1) % Ring.Num()]);
		}
		return Peri;
	}

	double InradiusEstimateXY(const TArray<FVector>& Ring)
	{
		const double Peri = RingPerimeterXY(Ring);
		if (Peri < 1.0e-6)
		{
			return 0.0;
		}
		return (2.0 * RingAreaAbsXY(Ring)) / Peri;
	}

	bool InsetRingXY(const TArray<FVector>& Ring, double Width, TArray<FVector>& OutInset)
	{
		OutInset.Reset();
		const int32 N = Ring.Num();
		if (N < 3 || Width <= 1.0e-4)
		{
			return false;
		}
		const bool bCCW = SignedArea2XY(Ring) > 0.0;
		OutInset.SetNum(N);
		for (int32 I = 0; I < N; ++I)
		{
			const FVector& Prev = Ring[(I + N - 1) % N];
			const FVector& Curr = Ring[I];
			const FVector& Next = Ring[(I + 1) % N];
			const FVector2D Vel = VertexOffsetVelXY(Prev, Curr, Next, bCCW);
			if (Vel.SizeSquared() > 1.0e8)
			{
				return false;
			}
			OutInset[I] = FVector(Curr.X + Vel.X * Width, Curr.Y + Vel.Y * Width, Curr.Z);
		}

		const double AreaIn = SignedArea2XY(OutInset);
		const double AreaOut = SignedArea2XY(Ring);
		if (AreaIn * AreaOut <= 0.0 || FMath::Abs(AreaIn) < 1.0e-2)
		{
			return false;
		}
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D E0(Ring[(I + 1) % N].X - Ring[I].X, Ring[(I + 1) % N].Y - Ring[I].Y);
			const FVector2D E1(
				OutInset[(I + 1) % N].X - OutInset[I].X,
				OutInset[(I + 1) % N].Y - OutInset[I].Y);
			if (FVector2D::DotProduct(E0, E1) <= 0.0)
			{
				return false;
			}
		}
		return true;
	}

	FVector PointOnEavePlane(const FVector2D& XY, const FVector& PlanePoint, const FVector& Up)
	{
		FVector P(XY.X, XY.Y, PlanePoint.Z);
		if (FMath::Abs(Up.Z) > 1.0e-6)
		{
			const double Dz =
				-((XY.X - PlanePoint.X) * Up.X + (XY.Y - PlanePoint.Y) * Up.Y) / Up.Z;
			P.Z = PlanePoint.Z + Dz;
		}
		return P;
	}

	struct FRoofTri2D
	{
		FVector2D A, B, C;
		double DA = 0.0;
		double DB = 0.0;
		double DC = 0.0;
	};

	void AddRoofTri2D(
		TArray<FRoofTri2D>& Tris,
		const FVector2D& A,
		double DA,
		const FVector2D& B,
		double DB,
		const FVector2D& C,
		double DC)
	{
		const double Cross = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
		if (FMath::Abs(Cross) < 1.0e-8)
		{
			return;
		}
		FRoofTri2D Tri;
		Tri.A = A;
		Tri.B = B;
		Tri.C = C;
		Tri.DA = DA;
		Tri.DB = DB;
		Tri.DC = DC;
		Tris.Add(Tri);
	}

	/** Edge whose two vertices share offset-from-eave (iso-height) is parallel to that slope's footprint edge. */
	FVector2D EaveDirFromRoofTri2D(const FRoofTri2D& Tri)
	{
		FVector2D BestDir = FVector2D::ZeroVector;
		double BestDiff = TNumericLimits<double>::Max();
		auto Consider = [&BestDir, &BestDiff](const FVector2D& P0, const FVector2D& P1, double D0, double D1)
		{
			const FVector2D Delta = P1 - P0;
			if (Delta.SizeSquared() < 1.0)
			{
				return;
			}
			const double Diff = FMath::Abs(D0 - D1);
			if (Diff < BestDiff)
			{
				BestDiff = Diff;
				BestDir = Delta.GetSafeNormal();
			}
		};
		Consider(Tri.A, Tri.B, Tri.DA, Tri.DB);
		Consider(Tri.B, Tri.C, Tri.DB, Tri.DC);
		Consider(Tri.C, Tri.A, Tri.DC, Tri.DA);
		return BestDir;
	}

	struct FWaveVert
	{
		FVector2D P0;
		FVector2D Vel;
		int32 LeftEdge = 0;
		int32 RightEdge = 0;
	};

	FVector2D WaveAt(const FWaveVert& V, double D)
	{
		return V.P0 + V.Vel * D;
	}

	double Cross2(const FVector2D& U, const FVector2D& V)
	{
		return U.X * V.Y - U.Y * V.X;
	}

	bool WaveMeet(const FWaveVert& A, const FWaveVert& B, double DMin, double& OutD, FVector2D& OutP)
	{
		const FVector2D RelV = A.Vel - B.Vel;
		const double Denom = RelV.SizeSquared();
		if (Denom < 1.0e-14)
		{
			return false;
		}
		const double D = FVector2D::DotProduct(B.P0 - A.P0, RelV) / Denom;
		if (D <= DMin + 1.0e-7)
		{
			return false;
		}
		const FVector2D PA = WaveAt(A, D);
		const FVector2D PB = WaveAt(B, D);
		if ((PA - PB).SizeSquared() > 1.0)
		{
			return false;
		}
		OutD = D;
		OutP = (PA + PB) * 0.5;
		return true;
	}

	bool WaveHitEdge(
		const FWaveVert& V,
		const FWaveVert& E0,
		const FWaveVert& E1,
		double DMin,
		double& OutD,
		FVector2D& OutP)
	{
		const FVector2D AB = V.P0 - E0.P0;
		const FVector2D Vab = V.Vel - E0.Vel;
		const FVector2D C = E1.P0 - E0.P0;
		const FVector2D Vd = E1.Vel - E0.Vel;
		const double Qa = Cross2(Vab, Vd);
		const double Qb = Cross2(AB, Vd) + Cross2(Vab, C);
		const double Qc = Cross2(AB, C);
		double Roots[2];
		int32 NumRoots = 0;
		if (FMath::Abs(Qa) < 1.0e-14)
		{
			if (FMath::Abs(Qb) < 1.0e-14)
			{
				return false;
			}
			Roots[NumRoots++] = -Qc / Qb;
		}
		else
		{
			const double Disc = Qb * Qb - 4.0 * Qa * Qc;
			if (Disc < 0.0)
			{
				return false;
			}
			const double S = FMath::Sqrt(Disc);
			Roots[NumRoots++] = (-Qb - S) / (2.0 * Qa);
			Roots[NumRoots++] = (-Qb + S) / (2.0 * Qa);
		}

		bool bFound = false;
		double BestD = 0.0;
		FVector2D BestP = FVector2D::ZeroVector;
		for (int32 R = 0; R < NumRoots; ++R)
		{
			const double D = Roots[R];
			if (D <= DMin + 1.0e-7)
			{
				continue;
			}
			const FVector2D P = WaveAt(V, D);
			const FVector2D Q0 = WaveAt(E0, D);
			const FVector2D Q1 = WaveAt(E1, D);
			const FVector2D Edge = Q1 - Q0;
			const double EdgeLenSq = Edge.SizeSquared();
			if (EdgeLenSq < 1.0e-12)
			{
				continue;
			}
			const double U = FVector2D::DotProduct(P - Q0, Edge) / EdgeLenSq;
			// Axis-aligned C/U notches often hit exactly at an edge endpoint.
			// Rejecting those (old 1e-4 margin) skips the split and the hip falls back to flat.
			if (U < -1.0e-3 || U > 1.0 + 1.0e-3)
			{
				continue;
			}
			const double UClamped = FMath::Clamp(U, 0.0, 1.0);
			const FVector2D Closest = Q0 + Edge * UClamped;
			if ((P - Closest).SizeSquared() > 1.0)
			{
				continue;
			}
			if (!bFound || D < BestD)
			{
				bFound = true;
				BestD = D;
				BestP = Closest;
			}
		}
		if (!bFound)
		{
			return false;
		}
		OutD = BestD;
		OutP = BestP;
		return true;
	}

	FWaveVert MakeWaveVert(
		const FVector2D& PAtD,
		double D,
		const FVector2D& LeftN,
		const FVector2D& RightN,
		int32 LeftEdge,
		int32 RightEdge)
	{
		const double Denom = 1.0 + FVector2D::DotProduct(LeftN, RightN);
		FVector2D Vel = (FMath::Abs(Denom) < 1.0e-6)
			? (LeftN + RightN) * 0.5
			: (LeftN + RightN) / Denom;
		if (Vel.SizeSquared() > 1.0e4)
		{
			Vel = Vel.GetSafeNormal() * 100.0;
		}
		FWaveVert W;
		W.Vel = Vel;
		W.P0 = PAtD - Vel * D;
		W.LeftEdge = LeftEdge;
		W.RightEdge = RightEdge;
		return W;
	}

	bool IsReflexWave(const TArray<FWaveVert>& Loop, int32 I, double D, bool bCCW)
	{
		const int32 N = Loop.Num();
		const FVector2D Prev = WaveAt(Loop[(I + N - 1) % N], D);
		const FVector2D Curr = WaveAt(Loop[I], D);
		const FVector2D Next = WaveAt(Loop[(I + 1) % N], D);
		const double CrossVal = (Curr.X - Prev.X) * (Next.Y - Curr.Y) - (Curr.Y - Prev.Y) * (Next.X - Curr.X);
		return bCCW ? (CrossVal < -1.0e-8) : (CrossVal > 1.0e-8);
	}

	void EmitLoft(TArray<FRoofTri2D>& OutTris, const TArray<FWaveVert>& Loop, double D0, double D1)
	{
		if (D1 <= D0 + 1.0e-9)
		{
			return;
		}
		const int32 N = Loop.Num();
		for (int32 I = 0; I < N; ++I)
		{
			const int32 J = (I + 1) % N;
			const FVector2D A = WaveAt(Loop[I], D0);
			const FVector2D B = WaveAt(Loop[J], D0);
			const FVector2D C = WaveAt(Loop[J], D1);
			const FVector2D Dv = WaveAt(Loop[I], D1);
			AddRoofTri2D(OutTris, A, D0, B, D0, C, D1);
			AddRoofTri2D(OutTris, A, D0, C, D1, Dv, D1);
		}
	}

	/** True when the shrinking loop has become a ridge (or a point), not a 2D roof region. */
	bool WaveLoopIsRidge(const TArray<FWaveVert>& Loop, double D)
	{
		const int32 N = Loop.Num();
		if (N < 3)
		{
			return true;
		}
		TArray<FVector2D> Pts;
		Pts.SetNum(N);
		double Area2 = 0.0;
		double Peri = 0.0;
		for (int32 I = 0; I < N; ++I)
		{
			Pts[I] = WaveAt(Loop[I], D);
		}
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D& A = Pts[I];
			const FVector2D& B = Pts[(I + 1) % N];
			Area2 += A.X * B.Y - B.X * A.Y;
			Peri += (B - A).Size();
		}
		if (Peri < 20.0)
		{
			return true;
		}
		const double Area = FMath::Abs(Area2) * 0.5;
		const double Inr = 2.0 * Area / Peri;
		return Inr < 3.0;
	}

	bool LoopPointsInBounds(const TArray<FWaveVert>& Loop, double D, const FHipBounds& Bounds)
	{
		for (const FWaveVert& V : Loop)
		{
			if (V.Vel.SizeSquared() > 1.0e6 || !IsPointInHipBounds(Bounds, WaveAt(V, D)))
			{
				return false;
			}
		}
		return true;
	}

	bool ShrinkLoop(
		TArray<FWaveVert> Loop,
		double DCurrent,
		bool bCCW,
		const TArray<FVector2D>& EdgeNormals,
		const FHipBounds& Bounds,
		TArray<FRoofTri2D>& OutTris,
		int32 Depth)
	{
		if (Depth > 64 || Loop.Num() < 3)
		{
			return true;
		}

		for (int32 Guard = 0; Guard < 256 && Loop.Num() >= 3; ++Guard)
		{
			if (WaveLoopIsRidge(Loop, DCurrent))
			{
				break;
			}

			double BestD = TNumericLimits<double>::Max();
			int32 EventType = 0;
			int32 IA = INDEX_NONE;
			int32 IB = INDEX_NONE;
			FVector2D HitP = FVector2D::ZeroVector;
			const int32 N = Loop.Num();

			for (int32 I = 0; I < N; ++I)
			{
				const int32 J = (I + 1) % N;
				double D = 0.0;
				FVector2D P;
				if (WaveMeet(Loop[I], Loop[J], DCurrent, D, P)
					&& D <= Bounds.MaxD
					&& IsPointInHipBounds(Bounds, P)
					&& LoopPointsInBounds(Loop, D, Bounds)
					&& D < BestD)
				{
					BestD = D;
					EventType = 1;
					IA = I;
					IB = J;
					HitP = P;
				}
			}
			for (int32 I = 0; I < N; ++I)
			{
				if (!IsReflexWave(Loop, I, DCurrent, bCCW))
				{
					continue;
				}
				for (int32 E = 0; E < N; ++E)
				{
					const int32 E1 = (E + 1) % N;
					if (E == I || E1 == I || E == (I + N - 1) % N)
					{
						continue;
					}
					double D = 0.0;
					FVector2D P;
					if (WaveHitEdge(Loop[I], Loop[E], Loop[E1], DCurrent, D, P)
						&& D <= Bounds.MaxD
						&& IsPointInHipBounds(Bounds, P)
						&& LoopPointsInBounds(Loop, D, Bounds)
						&& D < BestD)
					{
						BestD = D;
						EventType = 2;
						IA = I;
						IB = E;
						HitP = P;
					}
				}
			}

			if (EventType == 0 || !FMath::IsFinite(BestD))
			{
				break;
			}
			if (BestD > Bounds.MaxD || !IsPointInHipBounds(Bounds, HitP) || !LoopPointsInBounds(Loop, BestD, Bounds))
			{
				break;
			}

			EmitLoft(OutTris, Loop, DCurrent, BestD);

			if (EventType == 1)
			{
				TArray<FWaveVert> NextLoop;
				NextLoop.Reserve(N - 1);
				for (int32 I = 0; I < N; ++I)
				{
					if (I == IA)
					{
						const int32 Left = Loop[IA].LeftEdge;
						const int32 Right = Loop[IB].RightEdge;
						NextLoop.Add(MakeWaveVert(
							HitP, BestD, EdgeNormals[Left], EdgeNormals[Right], Left, Right));
					}
					else if (I != IB)
					{
						NextLoop.Add(Loop[I]);
					}
				}
				Loop = MoveTemp(NextLoop);
				DCurrent = BestD;
			}
			else
			{
				const int32 E1 = (IB + 1) % N;
				TArray<FWaveVert> LoopA;
				TArray<FWaveVert> LoopB;
				{
					int32 K = (IA + 1) % N;
					int32 Walk = 0;
					while (K != E1 && Walk++ < N + 2)
					{
						LoopA.Add(Loop[K]);
						K = (K + 1) % N;
					}
					LoopA.Add(MakeWaveVert(
						HitP,
						BestD,
						EdgeNormals[Loop[IB].RightEdge],
						EdgeNormals[Loop[IA].RightEdge],
						Loop[IB].RightEdge,
						Loop[IA].RightEdge));
				}
				{
					int32 K = E1;
					int32 Walk = 0;
					while (K != IA && Walk++ < N + 2)
					{
						LoopB.Add(Loop[K]);
						K = (K + 1) % N;
					}
					LoopB.Add(MakeWaveVert(
						HitP,
						BestD,
						EdgeNormals[Loop[IA].LeftEdge],
						EdgeNormals[Loop[IB].RightEdge],
						Loop[IA].LeftEdge,
						Loop[IB].RightEdge));
				}

				bool bOk = true;
				if (LoopA.Num() >= 3)
				{
					bOk = ShrinkLoop(MoveTemp(LoopA), BestD, bCCW, EdgeNormals, Bounds, OutTris, Depth + 1) && bOk;
				}
				if (LoopB.Num() >= 3)
				{
					bOk = ShrinkLoop(MoveTemp(LoopB), BestD, bCCW, EdgeNormals, Bounds, OutTris, Depth + 1) && bOk;
				}
				return bOk;
			}
		}

		return OutTris.Num() > 0;
	}

	bool HippedTrisStayInBounds(const TArray<FRoofTri2D>& Tris, const FHipBounds& Bounds)
	{
		for (const FRoofTri2D& Tri : Tris)
		{
			if (Tri.DA > Bounds.MaxD || Tri.DB > Bounds.MaxD || Tri.DC > Bounds.MaxD
				|| !IsPointInHipBounds(Bounds, Tri.A)
				|| !IsPointInHipBounds(Bounds, Tri.B)
				|| !IsPointInHipBounds(Bounds, Tri.C))
			{
				return false;
			}
		}
		return Tris.Num() > 0;
	}

	bool BuildHippedRoofTris(const TArray<FVector>& TopIn, TArray<FRoofTri2D>& OutTris)
	{
		OutTris.Reset();
		TArray<FVector> Top = TopIn;
		CleanHipRingXY(Top);
		const int32 N = Top.Num();
		if (N < 3)
		{
			return false;
		}
		const FHipBounds Bounds = MakeHipBounds(Top);
		const bool bCCW = SignedArea2XY(Top) > 0.0;
		TArray<FVector2D> EdgeNormals;
		EdgeNormals.SetNum(N);
		TArray<FWaveVert> Loop;
		Loop.SetNum(N);
		for (int32 I = 0; I < N; ++I)
		{
			const int32 J = (I + 1) % N;
			EdgeNormals[I] = InwardNormalXY(Top[I], Top[J], bCCW);
			const FVector& Prev = Top[(I + N - 1) % N];
			Loop[I].P0 = FVector2D(Top[I].X, Top[I].Y);
			Loop[I].Vel = VertexOffsetVelXY(Prev, Top[I], Top[J], bCCW);
			Loop[I].LeftEdge = (I + N - 1) % N;
			Loop[I].RightEdge = I;
			if (Loop[I].Vel.SizeSquared() > 1.0e6)
			{
				return false;
			}
		}
		return ShrinkLoop(MoveTemp(Loop), 0.0, bCCW, EdgeNormals, Bounds, OutTris, 0)
			&& HippedTrisStayInBounds(OutTris, Bounds);
	}
}

void BuildingExtrudeUtils::AssignAllTrianglesMaterialSlot(FExtrudedPrismMesh& Mesh, int32 MaterialSlotIndex)
{
	const int32 NumTris = Mesh.Triangles.Num() / 3;
	Mesh.TriangleMaterialIndices.SetNum(NumTris);
	for (int32 I = 0; I < NumTris; ++I)
	{
		Mesh.TriangleMaterialIndices[I] = MaterialSlotIndex;
	}
}

bool BuildingExtrudeUtils::BuildPrismPartsFromRings(
	const TArray<FVector>& BaseRingLocal,
	const TArray<FVector>& TopRingLocal,
	double MetersPerUv,
	FExtrudedPrismMesh& OutWallsAndFloor,
	FExtrudedPrismMesh& OutRoof,
	FString& OutError)
{
	OutWallsAndFloor = FExtrudedPrismMesh();
	OutRoof = FExtrudedPrismMesh();

	TArray<FVector> Base = BaseRingLocal;
	TArray<FVector> Top = TopRingLocal;
	StripClosingDuplicate(Base);
	StripClosingDuplicate(Top);

	if (Base.Num() < 3 || Top.Num() < 3)
	{
		OutError = TEXT("Base/top rings need at least 3 vertices.");
		return false;
	}
	if (Base.Num() != Top.Num())
	{
		OutError = TEXT("Base and top rings must have the same vertex count.");
		return false;
	}

	TArray<int32> CapTris;
	if (!EarClipTriangulate(Base, CapTris, OutError))
	{
		return false;
	}

	const double UvScaleCm = FMath::Max(MetersPerUv, 0.01) * 100.0;

	// Orient against explicit outward dirs (CCW-in-XY guesses fail after Cesium frames).
	FVector Up(0, 0, 0);
	for (int32 I = 0; I < Base.Num(); ++I)
	{
		Up += (Top[I] - Base[I]);
	}
	Up = Up.GetSafeNormal();
	if (Up.IsNearlyZero())
	{
		Up = FVector::UpVector;
	}
	const FVector Down = -Up;
	const double AreaAboutUp = SignedAreaAboutUp(Base, Up);

	double MinX = Base[0].X;
	double MinY = Base[0].Y;
	for (int32 I = 1; I < Base.Num(); ++I)
	{
		MinX = FMath::Min(MinX, static_cast<double>(Base[I].X));
		MinY = FMath::Min(MinY, static_cast<double>(Base[I].Y));
	}

	auto CapUvFromXY = [MinX, MinY, UvScaleCm](const FVector& P) -> FVector2D
	{
		return FVector2D(
			static_cast<float>((static_cast<double>(P.X) - MinX) / UvScaleCm),
			static_cast<float>((static_cast<double>(P.Y) - MinY) / UvScaleCm));
	};

	TArray<double> CumDist;
	CumDist.SetNum(Base.Num());
	CumDist[0] = 0.0;
	for (int32 I = 1; I < Base.Num(); ++I)
	{
		CumDist[I] = CumDist[I - 1] + static_cast<double>(FVector::Dist2D(Base[I - 1], Base[I]));
	}
	const double ClosingSeg = static_cast<double>(FVector::Dist2D(Base.Last(), Base[0]));
	const double Perimeter = FMath::Max(CumDist.Last() + ClosingSeg, 1.0);

	// Floor — normals face down (outward).
	for (int32 I = 0; I + 2 < CapTris.Num(); I += 3)
	{
		const FVector& A = Base[CapTris[I]];
		const FVector& B = Base[CapTris[I + 1]];
		const FVector& C = Base[CapTris[I + 2]];
		AddTriWithUV(
			OutWallsAndFloor,
			A, B, C,
			CapUvFromXY(A), CapUvFromXY(B), CapUvFromXY(C),
			Down);
	}

	// Roof — normals face up (outward).
	for (int32 I = 0; I + 2 < CapTris.Num(); I += 3)
	{
		const FVector& A = Top[CapTris[I]];
		const FVector& B = Top[CapTris[I + 1]];
		const FVector& C = Top[CapTris[I + 2]];
		AddTriWithUV(
			OutRoof,
			A, B, C,
			CapUvFromXY(Base[CapTris[I]]),
			CapUvFromXY(Base[CapTris[I + 1]]),
			CapUvFromXY(Base[CapTris[I + 2]]),
			Up);
	}

	// Walls — outward is edge × Up using ring winding (works for C/H recesses).
	const int32 N = Base.Num();
	for (int32 I = 0; I < N; ++I)
	{
		const FVector& B0 = Base[I];
		const FVector& B1 = Base[(I + 1) % N];
		const FVector& T0 = Top[I];
		const FVector& T1 = Top[(I + 1) % N];
		const double U0 = CumDist[I] / UvScaleCm;
		const double U1 = (I + 1 < N)
			? (CumDist[I + 1] / UvScaleCm)
			: (Perimeter / UvScaleCm);
		const double Vt0 = static_cast<double>(FVector::Distance(B0, T0)) / UvScaleCm;
		const double Vt1 = static_cast<double>(FVector::Distance(B1, T1)) / UvScaleCm;
		const FVector2D UvB0(static_cast<float>(U0), 0.0f);
		const FVector2D UvB1(static_cast<float>(U1), 0.0f);
		const FVector2D UvT0(static_cast<float>(U0), static_cast<float>(Vt0));
		const FVector2D UvT1(static_cast<float>(U1), static_cast<float>(Vt1));

		const FVector Outward = EdgeOutwardDir(B0, B1, Up, AreaAboutUp);

		AddTriWithUV(OutWallsAndFloor, B0, B1, T1, UvB0, UvB1, UvT1, Outward);
		AddTriWithUV(OutWallsAndFloor, B0, T1, T0, UvB0, UvT1, UvT0, Outward);
	}

	if (OutWallsAndFloor.Triangles.Num() < 3 || OutRoof.Triangles.Num() < 3)
	{
		OutError = TEXT("Prism parts produced empty meshes.");
		return false;
	}
	return true;
}

EBuildingRoofType BuildingExtrudeUtils::ResolveRoofType(
	int32 RoofTypeCode,
	int32 FlatIndex,
	int32 HippedIndex,
	int32 ParapetIndex)
{
	if (RoofTypeCode == HippedIndex)
	{
		return EBuildingRoofType::Hipped;
	}
	if (RoofTypeCode == ParapetIndex)
	{
		return EBuildingRoofType::Parapet;
	}
	if (RoofTypeCode == FlatIndex)
	{
		return EBuildingRoofType::Flat;
	}
	return EBuildingRoofType::Flat;
}

bool BuildingExtrudeUtils::BuildFlatRoofPartsFromRings(
	const TArray<FVector>& BaseRingLocal,
	const TArray<FVector>& TopRingLocal,
	double MetersPerUv,
	FExtrudedPrismMesh& OutWallsAndFloor,
	FExtrudedPrismMesh& OutRoof,
	FString& OutError)
{
	return BuildPrismPartsFromRings(
		BaseRingLocal,
		TopRingLocal,
		MetersPerUv,
		OutWallsAndFloor,
		OutRoof,
		OutError);
}

bool BuildingExtrudeUtils::BuildHippedRoofPartsFromRings(
	const TArray<FVector>& BaseRingLocal,
	const TArray<FVector>& TopRingLocal,
	double MetersPerUv,
	double HippedHeightMeters,
	FExtrudedPrismMesh& OutWallsAndFloor,
	FExtrudedPrismMesh& OutRoof,
	FString& OutError)
{
	if (!BuildPrismPartsFromRings(
			BaseRingLocal,
			TopRingLocal,
			MetersPerUv,
			OutWallsAndFloor,
			OutRoof,
			OutError))
	{
		return false;
	}

	const double HeightCm = FMath::Max(HippedHeightMeters, 0.0) * 100.0;
	if (HeightCm <= 1.0)
	{
		return true;
	}

	TArray<FVector> Base = BaseRingLocal;
	TArray<FVector> Top = TopRingLocal;
	StripClosingDuplicate(Base);
	StripClosingDuplicate(Top);
	if (Base.Num() != Top.Num() || Top.Num() < 3)
	{
		return true;
	}

	FVector Up(0, 0, 0);
	for (int32 I = 0; I < Base.Num(); ++I)
	{
		Up += (Top[I] - Base[I]);
	}
	Up = Up.GetSafeNormal();
	if (Up.IsNearlyZero())
	{
		Up = FVector::UpVector;
	}

	TArray<FRoofTri2D> Tris2D;
	if (!BuildHippedRoofTris(Top, Tris2D))
	{
		UE_LOG(LogBuildingExtruder, Warning, TEXT("Hipped roof skeleton failed or produced a spike; keeping flat roof."));
		return true;
	}

	double MaxD = 0.0;
	for (const FRoofTri2D& Tri : Tris2D)
	{
		MaxD = FMath::Max(MaxD, FMath::Max(Tri.DA, FMath::Max(Tri.DB, Tri.DC)));
	}
	if (MaxD < 1.0e-3)
	{
		return true;
	}

	const double UvScaleCm = FMath::Max(MetersPerUv, 0.01) * 100.0;
	double MinX = Top[0].X;
	double MinY = Top[0].Y;
	for (int32 I = 1; I < Top.Num(); ++I)
	{
		MinX = FMath::Min(MinX, static_cast<double>(Top[I].X));
		MinY = FMath::Min(MinY, static_cast<double>(Top[I].Y));
	}
	auto CapUvFromXY = [MinX, MinY, UvScaleCm](const FVector2D& P) -> FVector2D
	{
		return FVector2D(
			static_cast<float>((static_cast<double>(P.X) - MinX) / UvScaleCm),
			static_cast<float>((static_cast<double>(P.Y) - MinY) / UvScaleCm));
	};

	const double Scale = HeightCm / MaxD;
	FExtrudedPrismMesh HipRoof;
	for (const FRoofTri2D& Tri : Tris2D)
	{
		const FVector A = PointOnEavePlane(Tri.A, Top[0], Up) + Up * (Tri.DA * Scale);
		const FVector B = PointOnEavePlane(Tri.B, Top[0], Up) + Up * (Tri.DB * Scale);
		const FVector C = PointOnEavePlane(Tri.C, Top[0], Up) + Up * (Tri.DC * Scale);
		const FVector FaceN = FVector::CrossProduct(B - A, C - A).GetSafeNormal();
		const double DSpan = FMath::Max3(Tri.DA, Tri.DB, Tri.DC) - FMath::Min3(Tri.DA, Tri.DB, Tri.DC);
		// Drop vertical "fin" triangles left by shrinking past a ridge.
		if (!FaceN.IsNearlyZero() && FMath::Abs(FVector::DotProduct(FaceN, Up)) < 0.25 && DSpan * Scale > 30.0)
		{
			continue;
		}
		AddTriWithUV(HipRoof, A, B, C, CapUvFromXY(Tri.A), CapUvFromXY(Tri.B), CapUvFromXY(Tri.C), Up);
	}
	if (HipRoof.Triangles.Num() >= 3)
	{
		OutRoof = MoveTemp(HipRoof);
	}
	return true;
}

bool BuildingExtrudeUtils::BuildParapetRoofPartsFromRings(
	const TArray<FVector>& BaseRingLocal,
	const TArray<FVector>& TopRingLocal,
	double MetersPerUv,
	double ParapetHeightMeters,
	double ParapetWidthMeters,
	FExtrudedPrismMesh& OutWallsAndFloor,
	FExtrudedPrismMesh& OutRoof,
	FString& OutError)
{
	if (!BuildPrismPartsFromRings(
			BaseRingLocal,
			TopRingLocal,
			MetersPerUv,
			OutWallsAndFloor,
			OutRoof,
			OutError))
	{
		return false;
	}

	TArray<FVector> Base = BaseRingLocal;
	TArray<FVector> Top = TopRingLocal;
	StripClosingDuplicate(Base);
	StripClosingDuplicate(Top);
	if (Base.Num() != Top.Num() || Top.Num() < 3)
	{
		return true;
	}

	const double WidthCm = FMath::Max(ParapetWidthMeters, 0.0) * 100.0;
	double HeightCm = FMath::Max(ParapetHeightMeters, 0.0) * 100.0;
	if (WidthCm <= 1.0 || HeightCm <= 1.0)
	{
		return true;
	}

	FVector Up(0, 0, 0);
	double WallHeightCm = 0.0;
	for (int32 I = 0; I < Base.Num(); ++I)
	{
		Up += (Top[I] - Base[I]);
		WallHeightCm += FVector::Distance(Base[I], Top[I]);
	}
	Up = Up.GetSafeNormal();
	if (Up.IsNearlyZero())
	{
		Up = FVector::UpVector;
	}
	const double AreaAboutUp = SignedAreaAboutUp(Top, Up);
	WallHeightCm /= static_cast<double>(Base.Num());
	HeightCm = FMath::Min(HeightCm, WallHeightCm * 0.95);
	if (HeightCm <= 1.0)
	{
		return true;
	}

	const double Inradius = InradiusEstimateXY(Top);
	if (WidthCm >= Inradius * 0.49)
	{
		UE_LOG(
			LogBuildingExtruder,
			Warning,
			TEXT("Parapet width too large for footprint (width=%.1fcm, inradius~%.1fcm); using flat roof."),
			WidthCm,
			Inradius);
		return true;
	}

	TArray<FVector> InnerTop;
	if (!InsetRingXY(Top, WidthCm, InnerTop))
	{
		UE_LOG(LogBuildingExtruder, Warning, TEXT("Parapet inset failed; using flat roof."));
		return true;
	}

	TArray<int32> InnerTris;
	FString ClipError;
	if (!EarClipTriangulate(InnerTop, InnerTris, ClipError))
	{
		UE_LOG(LogBuildingExtruder, Warning, TEXT("Parapet inner deck triangulate failed; using flat roof."));
		return true;
	}

	const double UvScaleCm = FMath::Max(MetersPerUv, 0.01) * 100.0;
	double MinX = Top[0].X;
	double MinY = Top[0].Y;
	for (int32 I = 1; I < Top.Num(); ++I)
	{
		MinX = FMath::Min(MinX, static_cast<double>(Top[I].X));
		MinY = FMath::Min(MinY, static_cast<double>(Top[I].Y));
	}
	auto CapUvFromXY = [MinX, MinY, UvScaleCm](const FVector& P) -> FVector2D
	{
		return FVector2D(
			static_cast<float>((static_cast<double>(P.X) - MinX) / UvScaleCm),
			static_cast<float>((static_cast<double>(P.Y) - MinY) / UvScaleCm));
	};

	TArray<double> CumDist;
	CumDist.SetNum(Top.Num());
	CumDist[0] = 0.0;
	for (int32 I = 1; I < Top.Num(); ++I)
	{
		CumDist[I] = CumDist[I - 1] + static_cast<double>(FVector::Dist2D(Top[I - 1], Top[I]));
	}
	const double ClosingSeg = static_cast<double>(FVector::Dist2D(Top.Last(), Top[0]));
	const double Perimeter = FMath::Max(CumDist.Last() + ClosingSeg, 1.0);

	TArray<FVector> InnerDeck;
	InnerDeck.SetNum(InnerTop.Num());
	for (int32 I = 0; I < InnerTop.Num(); ++I)
	{
		InnerDeck[I] = InnerTop[I] - Up * HeightCm;
	}

	FExtrudedPrismMesh ParapetRoof;
	for (int32 I = 0; I + 2 < InnerTris.Num(); I += 3)
	{
		const FVector& A = InnerDeck[InnerTris[I]];
		const FVector& B = InnerDeck[InnerTris[I + 1]];
		const FVector& C = InnerDeck[InnerTris[I + 2]];
		AddTriWithUV(
			ParapetRoof,
			A, B, C,
			CapUvFromXY(InnerTop[InnerTris[I]]),
			CapUvFromXY(InnerTop[InnerTris[I + 1]]),
			CapUvFromXY(InnerTop[InnerTris[I + 2]]),
			Up);
	}

	const int32 N = Top.Num();
	for (int32 I = 0; I < N; ++I)
	{
		const int32 J = (I + 1) % N;
		const FVector& O0 = Top[I];
		const FVector& O1 = Top[J];
		const FVector& I0 = InnerTop[I];
		const FVector& I1 = InnerTop[J];
		AddQuadWithUV(
			ParapetRoof,
			O0, O1, I1, I0,
			CapUvFromXY(O0), CapUvFromXY(O1), CapUvFromXY(I1), CapUvFromXY(I0),
			Up);

		const double U0 = CumDist[I] / UvScaleCm;
		const double U1 = (I + 1 < N) ? (CumDist[I + 1] / UvScaleCm) : (Perimeter / UvScaleCm);
		const double Vh = HeightCm / UvScaleCm;
		const FVector2D UvD0(static_cast<float>(U0), 0.0f);
		const FVector2D UvD1(static_cast<float>(U1), 0.0f);
		const FVector2D UvT0(static_cast<float>(U0), static_cast<float>(Vh));
		const FVector2D UvT1(static_cast<float>(U1), static_cast<float>(Vh));

		const FVector Inward = -EdgeOutwardDir(I0, I1, Up, AreaAboutUp);
		AddQuadWithUV(
			ParapetRoof,
			InnerDeck[I], InnerDeck[J], InnerTop[J], InnerTop[I],
			UvD0, UvD1, UvT1, UvT0,
			Inward);
	}

	if (ParapetRoof.Triangles.Num() >= 3)
	{
		OutRoof = MoveTemp(ParapetRoof);
	}
	return true;
}

bool BuildingExtrudeUtils::BuildRoofPlacementTriangles(
	EBuildingRoofType RoofType,
	const TArray<FVector>& BaseRingLocal,
	const TArray<FVector>& TopRingLocal,
	double ParapetHeightMeters,
	double ParapetWidthMeters,
	double HippedHeightMeters,
	TArray<FRoofPlaceTriangle>& OutTris)
{
	OutTris.Reset();
	TArray<FVector> Base = BaseRingLocal;
	TArray<FVector> Top = TopRingLocal;
	StripClosingDuplicate(Base);
	StripClosingDuplicate(Top);
	if (Top.Num() < 3)
	{
		return false;
	}

	FVector Up(0, 0, 0);
	const int32 Count = FMath::Min(Base.Num(), Top.Num());
	for (int32 I = 0; I < Count; ++I)
	{
		Up += (Top[I] - Base[I]);
	}
	Up = Up.GetSafeNormal();
	if (Up.IsNearlyZero())
	{
		Up = FVector::UpVector;
	}

	auto AddCapFromRing = [&OutTris](const TArray<FVector>& Ring) -> bool
	{
		TArray<int32> CapTris;
		FString ClipError;
		if (!EarClipTriangulate(Ring, CapTris, ClipError) || CapTris.Num() < 3)
		{
			return false;
		}
		for (int32 I = 0; I + 2 < CapTris.Num(); I += 3)
		{
			FRoofPlaceTriangle Tri;
			Tri.A = Ring[CapTris[I]];
			Tri.B = Ring[CapTris[I + 1]];
			Tri.C = Ring[CapTris[I + 2]];
			OutTris.Add(Tri);
		}
		return OutTris.Num() > 0;
	};

	if (RoofType == EBuildingRoofType::Parapet)
	{
		const double WidthCm = FMath::Max(ParapetWidthMeters, 0.0) * 100.0;
		double HeightCm = FMath::Max(ParapetHeightMeters, 0.0) * 100.0;
		double WallHeightCm = 0.0;
		for (int32 I = 0; I < Count; ++I)
		{
			WallHeightCm += FVector::Distance(Base[I], Top[I]);
		}
		WallHeightCm /= static_cast<double>(FMath::Max(Count, 1));
		HeightCm = FMath::Min(HeightCm, WallHeightCm * 0.95);
		if (WidthCm > 1.0 && HeightCm > 1.0 && WidthCm < InradiusEstimateXY(Top) * 0.49)
		{
			TArray<FVector> InnerTop;
			if (InsetRingXY(Top, WidthCm, InnerTop))
			{
				TArray<FVector> InnerDeck;
				InnerDeck.SetNum(InnerTop.Num());
				for (int32 I = 0; I < InnerTop.Num(); ++I)
				{
					InnerDeck[I] = InnerTop[I] - Up * HeightCm;
				}
				if (AddCapFromRing(InnerDeck))
				{
					return true;
				}
			}
		}
		OutTris.Reset();
		return AddCapFromRing(Top);
	}

	if (RoofType == EBuildingRoofType::Hipped)
	{
		const double HeightCm = FMath::Max(HippedHeightMeters, 0.0) * 100.0;
		TArray<FRoofTri2D> Tris2D;
		if (HeightCm > 1.0 && BuildHippedRoofTris(Top, Tris2D))
		{
			double MaxD = 0.0;
			for (const FRoofTri2D& Tri : Tris2D)
			{
				MaxD = FMath::Max(MaxD, FMath::Max(Tri.DA, FMath::Max(Tri.DB, Tri.DC)));
			}
			if (MaxD > 1.0e-3)
			{
				const double Scale = HeightCm / MaxD;
				for (const FRoofTri2D& Tri : Tris2D)
				{
					const FVector A = PointOnEavePlane(Tri.A, Top[0], Up) + Up * (Tri.DA * Scale);
					const FVector B = PointOnEavePlane(Tri.B, Top[0], Up) + Up * (Tri.DB * Scale);
					const FVector C = PointOnEavePlane(Tri.C, Top[0], Up) + Up * (Tri.DC * Scale);
					const FVector FaceN = FVector::CrossProduct(B - A, C - A).GetSafeNormal();
					const double DSpan =
						FMath::Max3(Tri.DA, Tri.DB, Tri.DC) - FMath::Min3(Tri.DA, Tri.DB, Tri.DC);
					if (!FaceN.IsNearlyZero()
						&& FMath::Abs(FVector::DotProduct(FaceN, Up)) < 0.25
						&& DSpan * Scale > 30.0)
					{
						continue;
					}
					FRoofPlaceTriangle Place;
					Place.A = A;
					Place.B = B;
					Place.C = C;
					Place.AlignDirXY = EaveDirFromRoofTri2D(Tri);
					OutTris.Add(Place);
				}
				if (OutTris.Num() > 0)
				{
					return true;
				}
			}
		}
		OutTris.Reset();
		return AddCapFromRing(Top);
	}

	return AddCapFromRing(Top);
}

bool BuildingExtrudeUtils::BuildRoofPartsFromRings(
	EBuildingRoofType RoofType,
	const TArray<FVector>& BaseRingLocal,
	const TArray<FVector>& TopRingLocal,
	double MetersPerUv,
	double ParapetHeightMeters,
	double ParapetWidthMeters,
	double HippedHeightMeters,
	FExtrudedPrismMesh& OutWallsAndFloor,
	FExtrudedPrismMesh& OutRoof,
	FString& OutError)
{
	switch (RoofType)
	{
	case EBuildingRoofType::Hipped:
		return BuildHippedRoofPartsFromRings(
			BaseRingLocal,
			TopRingLocal,
			MetersPerUv,
			HippedHeightMeters,
			OutWallsAndFloor,
			OutRoof,
			OutError);
	case EBuildingRoofType::Parapet:
		return BuildParapetRoofPartsFromRings(
			BaseRingLocal,
			TopRingLocal,
			MetersPerUv,
			ParapetHeightMeters,
			ParapetWidthMeters,
			OutWallsAndFloor,
			OutRoof,
			OutError);
	case EBuildingRoofType::Flat:
	default:
		return BuildFlatRoofPartsFromRings(
			BaseRingLocal, TopRingLocal, MetersPerUv, OutWallsAndFloor, OutRoof, OutError);
	}
}

bool BuildingExtrudeUtils::BuildPrismFromRings(
	const TArray<FVector>& BaseRingLocal,
	const TArray<FVector>& TopRingLocal,
	double MetersPerUv,
	FExtrudedPrismMesh& OutMesh,
	FString& OutError)
{
	FExtrudedPrismMesh WallsAndFloor;
	FExtrudedPrismMesh Roof;
	if (!BuildPrismPartsFromRings(
			BaseRingLocal,
			TopRingLocal,
			MetersPerUv,
			WallsAndFloor,
			Roof,
			OutError))
	{
		OutMesh = FExtrudedPrismMesh();
		return false;
	}

	OutMesh = FExtrudedPrismMesh();
	AppendMesh(OutMesh, WallsAndFloor);
	AppendMesh(OutMesh, Roof);
	return OutMesh.Triangles.Num() >= 3;
}

bool BuildingExtrudeUtils::BuildPrism(
	const TArray<FVector>& BaseRingLocal,
	double HeightCm,
	FExtrudedPrismMesh& OutMesh,
	FString& OutError)
{
	if (HeightCm <= 0.0)
	{
		OutError = TEXT("Height must be positive.");
		return false;
	}

	TArray<FVector> Base = BaseRingLocal;
	StripClosingDuplicate(Base);
	for (FVector& V : Base)
	{
		V.Z = 0.0;
	}

	TArray<FVector> Top = Base;
	for (FVector& V : Top)
	{
		V.Z = HeightCm;
	}

	return BuildPrismFromRings(
		Base,
		Top,
		/*MetersPerUv*/ 3.0,
		OutMesh,
		OutError);
}
