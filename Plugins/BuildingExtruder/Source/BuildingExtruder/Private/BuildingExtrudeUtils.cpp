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
		int32 MaterialSlotIndex = 0)
	{
		FVector Normal = FVector::CrossProduct(B - A, C - A).GetSafeNormal();
		if (Normal.IsNearlyZero())
		{
			Normal = FVector::UpVector;
		}
		const int32 Base = Mesh.Vertices.Num();
		Mesh.Vertices.Add(A);
		Mesh.Vertices.Add(B);
		Mesh.Vertices.Add(C);
		Mesh.Normals.Add(Normal);
		Mesh.Normals.Add(Normal);
		Mesh.Normals.Add(Normal);
		Mesh.UVs.Add(UvA);
		Mesh.UVs.Add(UvB);
		Mesh.UVs.Add(UvC);
		Mesh.Triangles.Add(Base);
		Mesh.Triangles.Add(Base + 1);
		Mesh.Triangles.Add(Base + 2);
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

	const bool bCCW = SignedArea2XY(Base) > 0.0;
	const double UvScaleCm = FMath::Max(MetersPerUv, 0.01) * 100.0;

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

	// Floor (bottom cap) — normals face downward (outward).
	for (int32 I = 0; I + 2 < CapTris.Num(); I += 3)
	{
		const FVector& A = Base[CapTris[I]];
		const FVector& B = Base[CapTris[I + 1]];
		const FVector& C = Base[CapTris[I + 2]];
		const FVector2D UvA = CapUvFromXY(A);
		const FVector2D UvB = CapUvFromXY(B);
		const FVector2D UvC = CapUvFromXY(C);
		// Previous winding faced inward (looked good from inside). Flip for outward.
		if (bCCW)
		{
			AddTriWithUV(OutWallsAndFloor, A, B, C, UvA, UvB, UvC);
		}
		else
		{
			AddTriWithUV(OutWallsAndFloor, A, C, B, UvA, UvC, UvB);
		}
	}

	// Roof (top cap) — normals face upward (outward).
	for (int32 I = 0; I + 2 < CapTris.Num(); I += 3)
	{
		const FVector& A = Top[CapTris[I]];
		const FVector& B = Top[CapTris[I + 1]];
		const FVector& C = Top[CapTris[I + 2]];
		const FVector2D UvA = CapUvFromXY(Base[CapTris[I]]);
		const FVector2D UvB = CapUvFromXY(Base[CapTris[I + 1]]);
		const FVector2D UvC = CapUvFromXY(Base[CapTris[I + 2]]);
		if (bCCW)
		{
			AddTriWithUV(OutRoof, A, C, B, UvA, UvC, UvB);
		}
		else
		{
			AddTriWithUV(OutRoof, A, B, C, UvA, UvB, UvC);
		}
	}

	// Walls — normals face outward. No double-sided copies (those used broken UVs).
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

		if (bCCW)
		{
			AddTriWithUV(OutWallsAndFloor, B0, T1, B1, UvB0, UvT1, UvB1);
			AddTriWithUV(OutWallsAndFloor, B0, T0, T1, UvB0, UvT0, UvT1);
		}
		else
		{
			AddTriWithUV(OutWallsAndFloor, B0, B1, T1, UvB0, UvB1, UvT1);
			AddTriWithUV(OutWallsAndFloor, B0, T1, T0, UvB0, UvT1, UvT0);
		}
	}

	if (OutWallsAndFloor.Triangles.Num() < 3 || OutRoof.Triangles.Num() < 3)
	{
		OutError = TEXT("Prism parts produced empty meshes.");
		return false;
	}
	return true;
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
