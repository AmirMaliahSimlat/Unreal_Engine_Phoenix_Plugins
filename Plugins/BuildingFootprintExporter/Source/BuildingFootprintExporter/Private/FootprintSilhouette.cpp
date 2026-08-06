#include "FootprintSilhouette.h"

#include "Algo/Reverse.h"
#include "BuildingFootprintExporterLog.h"

namespace
{
	double SilhouetteSignedArea2(const TArray<FVector2D>& Ring)
	{
		if (Ring.Num() < 3)
		{
			return 0.0;
		}
		double Area2 = 0.0;
		const int32 Count = Ring.Num();
		for (int32 I = 0; I < Count; ++I)
		{
			const FVector2D& A = Ring[I];
			const FVector2D& B = Ring[(I + 1) % Count];
			Area2 += A.X * B.Y - B.X * A.Y;
		}
		return Area2;
	}

	double PolygonAreaAbsM2(const TArray<FVector2D>& RingCm, double UnrealUnitsPerMeter)
	{
		const double AreaCm2 = FMath::Abs(SilhouetteSignedArea2(RingCm)) * 0.5;
		const double Scale = UnrealUnitsPerMeter * UnrealUnitsPerMeter;
		return AreaCm2 / Scale;
	}

	bool PointInTriangle(const FVector2D& P, const FVector2D& A, const FVector2D& B, const FVector2D& C)
	{
		const double D1 = (P.X - B.X) * (A.Y - B.Y) - (A.X - B.X) * (P.Y - B.Y);
		const double D2 = (P.X - C.X) * (B.Y - C.Y) - (B.X - C.X) * (P.Y - C.Y);
		const double D3 = (P.X - A.X) * (C.Y - A.Y) - (C.X - A.X) * (P.Y - A.Y);
		const bool HasNeg = (D1 < 0.0) || (D2 < 0.0) || (D3 < 0.0);
		const bool HasPos = (D1 > 0.0) || (D2 > 0.0) || (D3 > 0.0);
		return !(HasNeg && HasPos);
	}

	bool BarycentricWeights(const FVector2D& P, const FVector2D& A, const FVector2D& B, const FVector2D& C, double& OutWA, double& OutWB, double& OutWC)
	{
		const double Denom = (B.Y - C.Y) * (A.X - C.X) + (C.X - B.X) * (A.Y - C.Y);
		if (FMath::IsNearlyZero(Denom))
		{
			return false;
		}
		OutWA = ((B.Y - C.Y) * (P.X - C.X) + (C.X - B.X) * (P.Y - C.Y)) / Denom;
		OutWB = ((C.Y - A.Y) * (P.X - C.X) + (A.X - C.X) * (P.Y - C.Y)) / Denom;
		OutWC = 1.0 - OutWA - OutWB;
		return true;
	}

	void RasterizeTriangle(
		TArray<uint8>& Grid,
		TArray<double>& CellZMin,
		TArray<double>& CellZMax,
		TArray<uint8>& CellHasZ,
		int32 Width,
		int32 Height,
		const FVector2D& OriginCm,
		double CellSizeCm,
		const FGroundTriangle2D& Tri)
	{
		const double MinX = FMath::Min3(Tri.A.X, Tri.B.X, Tri.C.X);
		const double MinY = FMath::Min3(Tri.A.Y, Tri.B.Y, Tri.C.Y);
		const double MaxX = FMath::Max3(Tri.A.X, Tri.B.X, Tri.C.X);
		const double MaxY = FMath::Max3(Tri.A.Y, Tri.B.Y, Tri.C.Y);

		const int32 X0 = FMath::Clamp(FMath::FloorToInt((MinX - OriginCm.X) / CellSizeCm), 0, Width - 1);
		const int32 Y0 = FMath::Clamp(FMath::FloorToInt((MinY - OriginCm.Y) / CellSizeCm), 0, Height - 1);
		const int32 X1 = FMath::Clamp(FMath::CeilToInt((MaxX - OriginCm.X) / CellSizeCm), 0, Width - 1);
		const int32 Y1 = FMath::Clamp(FMath::CeilToInt((MaxY - OriginCm.Y) / CellSizeCm), 0, Height - 1);

		auto UpdateCellZ = [&](int32 Idx, double Z)
		{
			if (!CellHasZ[Idx])
			{
				CellZMin[Idx] = Z;
				CellZMax[Idx] = Z;
				CellHasZ[Idx] = 1;
			}
			else
			{
				CellZMin[Idx] = FMath::Min(CellZMin[Idx], Z);
				CellZMax[Idx] = FMath::Max(CellZMax[Idx], Z);
			}
		};

		auto SampleZ = [&](const FVector2D& P) -> double
		{
			double WA = 0.0, WB = 0.0, WC = 0.0;
			if (BarycentricWeights(P, Tri.A, Tri.B, Tri.C, WA, WB, WC))
			{
				return WA * Tri.ZA + WB * Tri.ZB + WC * Tri.ZC;
			}
			return FMath::Min3(Tri.ZA, Tri.ZB, Tri.ZC);
		};

		auto MarkCell = [&](int32 X, int32 Y, const FVector2D& SampleP)
		{
			if (X < 0 || Y < 0 || X >= Width || Y >= Height)
			{
				return;
			}
			const int32 Idx = Y * Width + X;
			Grid[Idx] = 1;
			UpdateCellZ(Idx, SampleZ(SampleP));
		};

		// Always mark cells that contain triangle vertices (preserves thin tips / balcony corners).
		const FVector2D Verts[3] = {Tri.A, Tri.B, Tri.C};
		for (const FVector2D& V : Verts)
		{
			const int32 VX = FMath::FloorToInt((V.X - OriginCm.X) / CellSizeCm);
			const int32 VY = FMath::FloorToInt((V.Y - OriginCm.Y) / CellSizeCm);
			MarkCell(VX, VY, V);
		}

		for (int32 Y = Y0; Y <= Y1; ++Y)
		{
			for (int32 X = X0; X <= X1; ++X)
			{
				const double XMin = OriginCm.X + static_cast<double>(X) * CellSizeCm;
				const double YMin = OriginCm.Y + static_cast<double>(Y) * CellSizeCm;
				const double XMax = XMin + CellSizeCm;
				const double YMax = YMin + CellSizeCm;
				const FVector2D Center(0.5 * (XMin + XMax), 0.5 * (YMin + YMax));
				const FVector2D C00(XMin, YMin);
				const FVector2D C10(XMax, YMin);
				const FVector2D C01(XMin, YMax);
				const FVector2D C11(XMax, YMax);

				// Conservative coverage: center or any cell corner inside the triangle.
				// Fixes trapezoids collapsing to triangles when only the center was tested.
				const bool bHit =
					PointInTriangle(Center, Tri.A, Tri.B, Tri.C) ||
					PointInTriangle(C00, Tri.A, Tri.B, Tri.C) ||
					PointInTriangle(C10, Tri.A, Tri.B, Tri.C) ||
					PointInTriangle(C01, Tri.A, Tri.B, Tri.C) ||
					PointInTriangle(C11, Tri.A, Tri.B, Tri.C);

				if (!bHit)
				{
					continue;
				}

				MarkCell(X, Y, Center);
			}
		}
	}

	struct FGridEdge
	{
		int32 X0 = 0;
		int32 Y0 = 0;
		int32 X1 = 0;
		int32 Y1 = 0;

		bool operator==(const FGridEdge& Other) const
		{
			return X0 == Other.X0 && Y0 == Other.Y0 && X1 == Other.X1 && Y1 == Other.Y1;
		}
	};

	FORCEINLINE uint32 GetTypeHash(const FGridEdge& E)
	{
		return HashCombine(
			HashCombine(::GetTypeHash(E.X0), ::GetTypeHash(E.Y0)),
			HashCombine(::GetTypeHash(E.X1), ::GetTypeHash(E.Y1)));
	}

	bool IsFilled(const TArray<uint8>& Grid, int32 Width, int32 Height, int32 X, int32 Y)
	{
		if (X < 0 || Y < 0 || X >= Width || Y >= Height)
		{
			return false;
		}
		return Grid[Y * Width + X] != 0;
	}

	TArray<TArray<FVector2D>> ExtractRingsFromGrid(
		const TArray<uint8>& Grid,
		int32 Width,
		int32 Height,
		const FVector2D& OriginCm,
		double CellSizeCm)
	{
		// Directed boundary edges around filled cells (CCW: fill stays on the left).
		TMap<uint64, TArray<FIntPoint>> Adjacency;
		auto Pack = [](int32 X, int32 Y) -> uint64
		{
			return (static_cast<uint64>(static_cast<uint32>(X)) << 32) | static_cast<uint32>(Y);
		};
		auto AddEdge = [&](int32 X0, int32 Y0, int32 X1, int32 Y1)
		{
			Adjacency.FindOrAdd(Pack(X0, Y0)).Add(FIntPoint(X1, Y1));
		};

		for (int32 Y = 0; Y < Height; ++Y)
		{
			for (int32 X = 0; X < Width; ++X)
			{
				if (!IsFilled(Grid, Width, Height, X, Y))
				{
					continue;
				}

				if (!IsFilled(Grid, Width, Height, X, Y - 1))
				{
					AddEdge(X, Y, X + 1, Y);
				}
				if (!IsFilled(Grid, Width, Height, X, Y + 1))
				{
					AddEdge(X + 1, Y + 1, X, Y + 1);
				}
				if (!IsFilled(Grid, Width, Height, X - 1, Y))
				{
					AddEdge(X, Y + 1, X, Y);
				}
				if (!IsFilled(Grid, Width, Height, X + 1, Y))
				{
					AddEdge(X + 1, Y, X + 1, Y + 1);
				}
			}
		}

		auto PickLeftmost = [](int32 FromX, int32 FromY, int32 AtX, int32 AtY, const TArray<FIntPoint>& Cands, const TSet<FGridEdge>& Visited, FIntPoint& OutChosen) -> bool
		{
			const int32 InX = AtX - FromX;
			const int32 InY = AtY - FromY;
			bool bHasBest = false;
			int32 BestCross = 0;
			int32 BestDot = 0;
			OutChosen = FIntPoint(0, 0);
			for (const FIntPoint& Cand : Cands)
			{
				FGridEdge CandEdge{AtX, AtY, Cand.X, Cand.Y};
				if (Visited.Contains(CandEdge))
				{
					continue;
				}
				const int32 OutX = Cand.X - AtX;
				const int32 OutY = Cand.Y - AtY;
				const int32 Cross = InX * OutY - InY * OutX;
				const int32 Dot = InX * OutX + InY * OutY;
				if (!bHasBest || Cross > BestCross || (Cross == BestCross && Dot > BestDot))
				{
					bHasBest = true;
					BestCross = Cross;
					BestDot = Dot;
					OutChosen = Cand;
				}
			}
			return bHasBest;
		};

		TSet<FGridEdge> Visited;
		TArray<TArray<FVector2D>> Rings;

		for (const TPair<uint64, TArray<FIntPoint>>& StartPair : Adjacency)
		{
			if (StartPair.Value.Num() == 0)
			{
				continue;
			}

			const int32 StartX = static_cast<int32>(StartPair.Key >> 32);
			const int32 StartY = static_cast<int32>(StartPair.Key & 0xffffffffu);

			for (const FIntPoint& FirstNext : StartPair.Value)
			{
				FGridEdge StartEdge{StartX, StartY, FirstNext.X, FirstNext.Y};
				if (Visited.Contains(StartEdge))
				{
					continue;
				}

				TArray<FVector2D> Ring;
				int32 PX = StartX;
				int32 PY = StartY;
				int32 CX = StartX;
				int32 CY = StartY;
				int32 NX = FirstNext.X;
				int32 NY = FirstNext.Y;
				const int32 MaxSteps = (Width + 1) * (Height + 1) * 4 + 8;

				for (int32 Step = 0; Step < MaxSteps; ++Step)
				{
					FGridEdge Edge{CX, CY, NX, NY};
					if (Visited.Contains(Edge))
					{
						break;
					}
					Visited.Add(Edge);
					Ring.Add(FVector2D(
						OriginCm.X + static_cast<double>(CX) * CellSizeCm,
						OriginCm.Y + static_cast<double>(CY) * CellSizeCm));

					PX = CX;
					PY = CY;
					CX = NX;
					CY = NY;

					if (CX == StartX && CY == StartY && Ring.Num() > 2)
					{
						break;
					}

					const TArray<FIntPoint>* NextList = Adjacency.Find(Pack(CX, CY));
					if (!NextList || NextList->Num() == 0)
					{
						break;
					}

					FIntPoint Chosen;
					if (!PickLeftmost(PX, PY, CX, CY, *NextList, Visited, Chosen))
					{
						break;
					}
					NX = Chosen.X;
					NY = Chosen.Y;
				}

				if (Ring.Num() >= 3)
				{
					if (!Ring[0].Equals(Ring.Last(), 1.0e-6))
					{
						const FVector2D First = Ring[0];
						Ring.Add(First);
					}
					Rings.Add(MoveTemp(Ring));
				}
			}
		}

		return Rings;
	}

	FVector2D RingCentroid(const TArray<FVector2D>& Ring)
	{
		FVector2D Centroid = FVector2D::ZeroVector;
		int32 Count = Ring.Num();
		if (Count >= 2 && Ring[0].Equals(Ring.Last(), 1.0e-6))
		{
			--Count;
		}
		if (Count <= 0)
		{
			return Centroid;
		}
		for (int32 I = 0; I < Count; ++I)
		{
			Centroid += Ring[I];
		}
		return Centroid / static_cast<double>(Count);
	}

	bool PointInRing(const FVector2D& P, const TArray<FVector2D>& Ring)
	{
		// Ray casting
		bool bInside = false;
		const int32 N = Ring.Num();
		for (int32 I = 0, J = N - 1; I < N; J = I++)
		{
			const FVector2D& A = Ring[I];
			const FVector2D& B = Ring[J];
			const bool Intersect = ((A.Y > P.Y) != (B.Y > P.Y))
				&& (P.X < (B.X - A.X) * (P.Y - A.Y) / (B.Y - A.Y + 1.0e-30) + A.X);
			if (Intersect)
			{
				bInside = !bInside;
			}
		}
		return bInside;
	}

	double PerpendicularDistance(const FVector2D& P, const FVector2D& A, const FVector2D& B)
	{
		const double DX = B.X - A.X;
		const double DY = B.Y - A.Y;
		if (FMath::IsNearlyZero(DX) && FMath::IsNearlyZero(DY))
		{
			return FVector2D::Distance(P, A);
		}
		const double T = FMath::Clamp(((P.X - A.X) * DX + (P.Y - A.Y) * DY) / (DX * DX + DY * DY), 0.0, 1.0);
		const FVector2D Proj(A.X + T * DX, A.Y + T * DY);
		return FVector2D::Distance(P, Proj);
	}

	void DouglasPeucker(const TArray<FVector2D>& Points, int32 Start, int32 End, double Tolerance, TArray<bool>& Keep)
	{
		if (End <= Start + 1)
		{
			return;
		}

		double MaxDist = -1.0;
		int32 MaxIndex = Start;
		for (int32 I = Start + 1; I < End; ++I)
		{
			const double Dist = PerpendicularDistance(Points[I], Points[Start], Points[End]);
			if (Dist > MaxDist)
			{
				MaxDist = Dist;
				MaxIndex = I;
			}
		}

		if (MaxDist > Tolerance)
		{
			Keep[MaxIndex] = true;
			DouglasPeucker(Points, Start, MaxIndex, Tolerance, Keep);
			DouglasPeucker(Points, MaxIndex, End, Tolerance, Keep);
		}
	}

	TArray<FVector2D> SimplifyRing(TArray<FVector2D> Ring, double ToleranceCm)
	{
		if (ToleranceCm <= 0.0 || Ring.Num() < 4)
		{
			return Ring;
		}

		// Drop closing duplicate for simplification.
		if (Ring.Num() >= 2 && Ring[0].Equals(Ring.Last(), 1.0e-6))
		{
			Ring.Pop();
		}
		if (Ring.Num() < 3)
		{
			return Ring;
		}

		TArray<bool> Keep;
		Keep.Init(false, Ring.Num());
		Keep[0] = true;
		Keep.Last() = true;
		DouglasPeucker(Ring, 0, Ring.Num() - 1, ToleranceCm, Keep);

		TArray<FVector2D> Out;
		Out.Reserve(Ring.Num());
		for (int32 I = 0; I < Ring.Num(); ++I)
		{
			if (Keep[I])
			{
				Out.Add(Ring[I]);
			}
		}
		if (Out.Num() >= 3 && !Out[0].Equals(Out.Last(), 1.0e-6))
		{
			const FVector2D First = Out[0];
			Out.Add(First);
		}
		return Out;
	}

	}

TArray<FSilhouettePolygon2D> FootprintSilhouette::BuildSilhouettesFromTriangles(
	const TArray<FGroundTriangle2D>& TrianglesCm,
	double CellSizeCm,
	double SimplifyToleranceCm,
	int32 MaxGridDimension,
	double UnrealUnitsPerMeter)
{
	TArray<FSilhouettePolygon2D> Result;
	if (TrianglesCm.Num() == 0)
	{
		return Result;
	}

	FBox2D Bounds(FVector2D::ZeroVector, FVector2D::ZeroVector);
	bool bHasBounds = false;
	for (const FGroundTriangle2D& Tri : TrianglesCm)
	{
		const double Area2 = (Tri.B.X - Tri.A.X) * (Tri.C.Y - Tri.A.Y) - (Tri.C.X - Tri.A.X) * (Tri.B.Y - Tri.A.Y);
		if (FMath::Abs(Area2) < 1.0e-6)
		{
			continue;
		}
		if (!bHasBounds)
		{
			Bounds = FBox2D(Tri.A, Tri.A);
			bHasBounds = true;
		}
		Bounds += Tri.A;
		Bounds += Tri.B;
		Bounds += Tri.C;
	}
	if (!bHasBounds)
	{
		return Result;
	}

	double Cell = FMath::Max(CellSizeCm, 1.0);
	const double RequestedCellCm = Cell;
	const double Pad = Cell;
	Bounds.Min -= FVector2D(Pad, Pad);
	Bounds.Max += FVector2D(Pad, Pad);

	int32 Width = FMath::Max(1, FMath::CeilToInt((Bounds.Max.X - Bounds.Min.X) / Cell));
	int32 Height = FMath::Max(1, FMath::CeilToInt((Bounds.Max.Y - Bounds.Min.Y) / Cell));
	const int32 MaxDim = FMath::Max(MaxGridDimension, 64);
	if (Width > MaxDim || Height > MaxDim)
	{
		const double ScaleW = static_cast<double>(Width) / static_cast<double>(MaxDim);
		const double ScaleH = static_cast<double>(Height) / static_cast<double>(MaxDim);
		Cell *= FMath::Max(ScaleW, ScaleH);
		Width = FMath::Max(1, FMath::CeilToInt((Bounds.Max.X - Bounds.Min.X) / Cell));
		Height = FMath::Max(1, FMath::CeilToInt((Bounds.Max.Y - Bounds.Min.Y) / Cell));
	}

	// Simplify runs AFTER rasterization (see SimplifyRing below), using EffectiveSimplifyCm.
	// Tolerance is a world-space feature size ("ignore wiggles smaller than X cm").
	// Do NOT scale it with cell (30→60 when cell 1→2 over-smooths real corners).
	// Only raise it when the cell itself is so large that grid stairs exceed X.
	const double EffectiveSimplifyCm = (SimplifyToleranceCm <= 0.0)
		? 0.0
		: FMath::Max(SimplifyToleranceCm, Cell * 2.0);
	if (Cell > RequestedCellCm + 1.0e-6)
	{
		UE_LOG(
			LogBuildingFootprintExporter,
			Display,
			TEXT("Grid coarsened for large cluster | cell %.2f->%.2f cm, simplify %.2f->%.2f cm (max of input, 2*cell), grid %dx%d"),
			RequestedCellCm,
			Cell,
			SimplifyToleranceCm,
			EffectiveSimplifyCm,
			Width,
			Height);
	}

	TArray<uint8> Grid;
	Grid.SetNumZeroed(Width * Height);
	TArray<double> CellZMin;
	TArray<double> CellZMax;
	TArray<uint8> CellHasZ;
	CellZMin.SetNumZeroed(Width * Height);
	CellZMax.SetNumZeroed(Width * Height);
	CellHasZ.SetNumZeroed(Width * Height);

	for (const FGroundTriangle2D& Tri : TrianglesCm)
	{
		RasterizeTriangle(Grid, CellZMin, CellZMax, CellHasZ, Width, Height, Bounds.Min, Cell, Tri);
	}

	// Connected components (8-connected so diagonal touches stay one building).
	TArray<int32> Labels;
	Labels.Init(-1, Width * Height);
	int32 NextLabel = 0;
	TArray<FIntPoint> Stack;

	for (int32 Y = 0; Y < Height; ++Y)
	{
		for (int32 X = 0; X < Width; ++X)
		{
			const int32 StartIdx = Y * Width + X;
			if (Grid[StartIdx] == 0 || Labels[StartIdx] >= 0)
			{
				continue;
			}

			const int32 Label = NextLabel++;
			Stack.Reset();
			Stack.Add(FIntPoint(X, Y));
			Labels[StartIdx] = Label;

			while (Stack.Num() > 0)
			{
				const FIntPoint P = Stack.Pop(false);
				const int32 Dirs[8][2] = {
					{1, 0}, {-1, 0}, {0, 1}, {0, -1},
					{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
				for (const auto& D : Dirs)
				{
					const int32 NX = P.X + D[0];
					const int32 NY = P.Y + D[1];
					if (NX < 0 || NY < 0 || NX >= Width || NY >= Height)
					{
						continue;
					}
					const int32 NIdx = NY * Width + NX;
					if (Grid[NIdx] == 0 || Labels[NIdx] >= 0)
					{
						continue;
					}
					Labels[NIdx] = Label;
					Stack.Add(FIntPoint(NX, NY));
				}
			}
		}
	}

	for (int32 Label = 0; Label < NextLabel; ++Label)
	{
		TArray<uint8> ComponentGrid;
		ComponentGrid.SetNumZeroed(Width * Height);
		for (int32 I = 0; I < Labels.Num(); ++I)
		{
			if (Labels[I] == Label)
			{
				ComponentGrid[I] = 1;
			}
		}

		TArray<TArray<FVector2D>> Rings = ExtractRingsFromGrid(ComponentGrid, Width, Height, Bounds.Min, Cell);
		if (Rings.Num() == 0)
		{
			continue;
		}

		bool bHasHeight = false;
		double CompZMin = 0.0;
		double CompZMax = 0.0;
		for (int32 I = 0; I < Labels.Num(); ++I)
		{
			if (Labels[I] != Label || !CellHasZ[I])
			{
				continue;
			}
			if (!bHasHeight)
			{
				CompZMin = CellZMin[I];
				CompZMax = CellZMax[I];
				bHasHeight = true;
			}
			else
			{
				CompZMin = FMath::Min(CompZMin, CellZMin[I]);
				CompZMax = FMath::Max(CompZMax, CellZMax[I]);
			}
		}
		const double HeightM = bHasHeight ? FMath::Max(0.0, (CompZMax - CompZMin) / UnrealUnitsPerMeter) : 0.0;

		TArray<double> AbsAreas;
		AbsAreas.SetNum(Rings.Num());
		for (int32 I = 0; I < Rings.Num(); ++I)
		{
			AbsAreas[I] = FMath::Abs(SilhouetteSignedArea2(Rings[I]));
		}

		TArray<int32> ExteriorIndices;
		for (int32 I = 0; I < Rings.Num(); ++I)
		{
			const FVector2D Centroid = RingCentroid(Rings[I]);
			bool bInsideLarger = false;
			for (int32 J = 0; J < Rings.Num(); ++J)
			{
				if (I == J || AbsAreas[J] <= AbsAreas[I])
				{
					continue;
				}
				if (PointInRing(Centroid, Rings[J]))
				{
					bInsideLarger = true;
					break;
				}
			}
			if (!bInsideLarger)
			{
				ExteriorIndices.Add(I);
			}
		}

		for (const int32 OuterIndex : ExteriorIndices)
		{
			TArray<FVector2D> Outer = SimplifyRing(Rings[OuterIndex], EffectiveSimplifyCm);
			if (Outer.Num() < 3)
			{
				continue;
			}
			if (SilhouetteSignedArea2(Outer) < 0.0)
			{
				Algo::Reverse(Outer);
			}

			FSilhouettePolygon2D Poly;
			Poly.OuterRingCm = Outer;
			Poly.AreaM2 = PolygonAreaAbsM2(Outer, UnrealUnitsPerMeter);
			Poly.HeightM = HeightM;

			for (int32 I = 0; I < Rings.Num(); ++I)
			{
				if (I == OuterIndex)
				{
					continue;
				}
				const FVector2D Centroid = RingCentroid(Rings[I]);
				if (!PointInRing(Centroid, Outer))
				{
					continue;
				}
				bool bBetterOuterExists = false;
				for (const int32 OtherOuter : ExteriorIndices)
				{
					if (OtherOuter == OuterIndex || AbsAreas[OtherOuter] >= AbsAreas[OuterIndex])
					{
						continue;
					}
					if (PointInRing(Centroid, Rings[OtherOuter]))
					{
						bBetterOuterExists = true;
						break;
					}
				}
				if (bBetterOuterExists)
				{
					continue;
				}

				TArray<FVector2D> Hole = SimplifyRing(Rings[I], EffectiveSimplifyCm);
				if (Hole.Num() < 3)
				{
					continue;
				}
				if (SilhouetteSignedArea2(Hole) > 0.0)
				{
					Algo::Reverse(Hole);
				}
				Poly.HoleRingsCm.Add(MoveTemp(Hole));
				Poly.AreaM2 -= PolygonAreaAbsM2(Poly.HoleRingsCm.Last(), UnrealUnitsPerMeter);
			}

			if (Poly.AreaM2 > 0.0)
			{
				Result.Add(MoveTemp(Poly));
			}
		}
	}

	return Result;
}

TArray<FSilhouettePolygon2D> FootprintSilhouette::BuildSilhouettesFromFilledRings(
	const TArray<TArray<FVector2D>>& OuterRingsCm,
	double CellSizeCm,
	double SimplifyToleranceCm,
	int32 MaxGridDimension,
	double UnrealUnitsPerMeter,
	double ForcedHeightM)
{
	TArray<FSilhouettePolygon2D> Result;
	if (OuterRingsCm.Num() == 0)
	{
		return Result;
	}

	FBox2D Bounds(FVector2D::ZeroVector, FVector2D::ZeroVector);
	bool bHasBounds = false;
	for (const TArray<FVector2D>& Ring : OuterRingsCm)
	{
		if (Ring.Num() < 3)
		{
			continue;
		}
		for (const FVector2D& P : Ring)
		{
			if (!bHasBounds)
			{
				Bounds = FBox2D(P, P);
				bHasBounds = true;
			}
			else
			{
				Bounds += P;
			}
		}
	}
	if (!bHasBounds)
	{
		return Result;
	}

	double Cell = FMath::Max(CellSizeCm, 1.0);
	const double RequestedCellCm = Cell;
	const double Pad = Cell;
	Bounds.Min -= FVector2D(Pad, Pad);
	Bounds.Max += FVector2D(Pad, Pad);

	int32 Width = FMath::Max(1, FMath::CeilToInt((Bounds.Max.X - Bounds.Min.X) / Cell));
	int32 Height = FMath::Max(1, FMath::CeilToInt((Bounds.Max.Y - Bounds.Min.Y) / Cell));
	const int32 MaxDim = FMath::Max(MaxGridDimension, 64);
	if (Width > MaxDim || Height > MaxDim)
	{
		const double ScaleW = static_cast<double>(Width) / static_cast<double>(MaxDim);
		const double ScaleH = static_cast<double>(Height) / static_cast<double>(MaxDim);
		Cell *= FMath::Max(ScaleW, ScaleH);
		Width = FMath::Max(1, FMath::CeilToInt((Bounds.Max.X - Bounds.Min.X) / Cell));
		Height = FMath::Max(1, FMath::CeilToInt((Bounds.Max.Y - Bounds.Min.Y) / Cell));
	}

	const double EffectiveSimplifyCm = (SimplifyToleranceCm <= 0.0)
		? 0.0
		: FMath::Max(SimplifyToleranceCm, Cell * 2.0);

	TArray<uint8> Grid;
	Grid.SetNumZeroed(Width * Height);

	for (const TArray<FVector2D>& Ring : OuterRingsCm)
	{
		if (Ring.Num() < 3)
		{
			continue;
		}

		FBox2D RingBounds(Ring[0], Ring[0]);
		for (const FVector2D& P : Ring)
		{
			RingBounds += P;
		}

		const int32 X0 = FMath::Clamp(FMath::FloorToInt((RingBounds.Min.X - Bounds.Min.X) / Cell), 0, Width - 1);
		const int32 Y0 = FMath::Clamp(FMath::FloorToInt((RingBounds.Min.Y - Bounds.Min.Y) / Cell), 0, Height - 1);
		const int32 X1 = FMath::Clamp(FMath::CeilToInt((RingBounds.Max.X - Bounds.Min.X) / Cell), 0, Width - 1);
		const int32 Y1 = FMath::Clamp(FMath::CeilToInt((RingBounds.Max.Y - Bounds.Min.Y) / Cell), 0, Height - 1);

		for (int32 Y = Y0; Y <= Y1; ++Y)
		{
			for (int32 X = X0; X <= X1; ++X)
			{
				const FVector2D Center(
					Bounds.Min.X + (static_cast<double>(X) + 0.5) * Cell,
					Bounds.Min.Y + (static_cast<double>(Y) + 0.5) * Cell);
				if (PointInRing(Center, Ring))
				{
					Grid[Y * Width + X] = 1;
				}
			}
		}
	}

	TArray<int32> Labels;
	Labels.Init(-1, Width * Height);
	int32 NextLabel = 0;
	TArray<FIntPoint> Stack;

	for (int32 Y = 0; Y < Height; ++Y)
	{
		for (int32 X = 0; X < Width; ++X)
		{
			const int32 StartIdx = Y * Width + X;
			if (Grid[StartIdx] == 0 || Labels[StartIdx] >= 0)
			{
				continue;
			}

			const int32 Label = NextLabel++;
			Stack.Reset();
			Stack.Add(FIntPoint(X, Y));
			Labels[StartIdx] = Label;

			while (Stack.Num() > 0)
			{
				const FIntPoint P = Stack.Pop(false);
				const int32 Dirs[8][2] = {
					{1, 0}, {-1, 0}, {0, 1}, {0, -1},
					{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
				for (const auto& D : Dirs)
				{
					const int32 NX = P.X + D[0];
					const int32 NY = P.Y + D[1];
					if (NX < 0 || NY < 0 || NX >= Width || NY >= Height)
					{
						continue;
					}
					const int32 NIdx = NY * Width + NX;
					if (Grid[NIdx] == 0 || Labels[NIdx] >= 0)
					{
						continue;
					}
					Labels[NIdx] = Label;
					Stack.Add(FIntPoint(NX, NY));
				}
			}
		}
	}

	for (int32 Label = 0; Label < NextLabel; ++Label)
	{
		TArray<uint8> ComponentGrid;
		ComponentGrid.SetNumZeroed(Width * Height);
		for (int32 I = 0; I < Labels.Num(); ++I)
		{
			if (Labels[I] == Label)
			{
				ComponentGrid[I] = 1;
			}
		}

		TArray<TArray<FVector2D>> Rings = ExtractRingsFromGrid(ComponentGrid, Width, Height, Bounds.Min, Cell);
		if (Rings.Num() == 0)
		{
			continue;
		}

		int32 OuterIndex = 0;
		double BestAbsArea = -1.0;
		for (int32 I = 0; I < Rings.Num(); ++I)
		{
			const double AbsA = FMath::Abs(SilhouetteSignedArea2(Rings[I]));
			if (AbsA > BestAbsArea)
			{
				BestAbsArea = AbsA;
				OuterIndex = I;
			}
		}

		TArray<FVector2D> Outer = SimplifyRing(Rings[OuterIndex], EffectiveSimplifyCm);
		if (Outer.Num() < 3)
		{
			continue;
		}
		if (SilhouetteSignedArea2(Outer) < 0.0)
		{
			Algo::Reverse(Outer);
		}

		FSilhouettePolygon2D Poly;
		Poly.OuterRingCm = Outer;
		Poly.AreaM2 = PolygonAreaAbsM2(Outer, UnrealUnitsPerMeter);
		Poly.HeightM = FMath::Max(0.0, ForcedHeightM);

		for (int32 I = 0; I < Rings.Num(); ++I)
		{
			if (I == OuterIndex)
			{
				continue;
			}
			TArray<FVector2D> Hole = SimplifyRing(Rings[I], EffectiveSimplifyCm);
			if (Hole.Num() < 3)
			{
				continue;
			}
			FVector2D Centroid = FVector2D::ZeroVector;
			for (const FVector2D& P : Hole)
			{
				Centroid += P;
			}
			Centroid /= static_cast<double>(Hole.Num());
			if (!PointInRing(Centroid, Outer))
			{
				continue;
			}
			if (SilhouetteSignedArea2(Hole) > 0.0)
			{
				Algo::Reverse(Hole);
			}
			Poly.HoleRingsCm.Add(MoveTemp(Hole));
			Poly.AreaM2 -= PolygonAreaAbsM2(Poly.HoleRingsCm.Last(), UnrealUnitsPerMeter);
		}

		if (Poly.AreaM2 > 0.0)
		{
			Result.Add(MoveTemp(Poly));
		}
	}

	return Result;
}
