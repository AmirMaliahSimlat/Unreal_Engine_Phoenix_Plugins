#include "WaterPlacerPrivatePCH.h"
#include "WaterSurface.h"
#include "WaterPlacerLog.h"

#include "HAL/PlatformTime.h"
#include "Math/NumericLimits.h"

namespace
{
	constexpr double DuplicateEpsDeg = 1.0e-10;
	constexpr int32 MinControlPoints = 8;
	constexpr int32 MaxTpsSolveCenters = 900;

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

	double RingDistSqDeg(const FVector2D& P, const TArray<FVector2D>& Ring)
	{
		const int32 N = Ring.Num();
		if (N <= 0)
		{
			return 1.0e30;
		}
		double Best = 1.0e30;
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D& A = Ring[I];
			const FVector2D& B = Ring[(I + 1) % N];
			const FVector2D AB = B - A;
			const double LenSq = AB.SizeSquared();
			double T = 0.0;
			if (LenSq > 1.0e-30)
			{
				T = FMath::Clamp(FVector2D::DotProduct(P - A, AB) / LenSq, 0.0, 1.0);
			}
			Best = FMath::Min(Best, FVector2D::DistSquared(P, A + AB * T));
		}
		return Best;
	}

	double PerpDistSq(const FVector2D& Point, const FVector2D& A, const FVector2D& B)
	{
		const FVector2D AB = B - A;
		const double LenSq = AB.SizeSquared();
		if (LenSq < 1.0e-30)
		{
			return FVector2D::DistSquared(Point, A);
		}
		const double T = FMath::Clamp(FVector2D::DotProduct(Point - A, AB) / LenSq, 0.0, 1.0);
		return FVector2D::DistSquared(Point, A + AB * T);
	}

	void RdpKeep(const TArray<FVector2D>& Pts, int32 Start, int32 End, double EpsSq, TArray<uint8>& Keep)
	{
		double MaxD = -1.0;
		int32 MaxI = Start;
		for (int32 I = Start + 1; I < End; ++I)
		{
			const double D = PerpDistSq(Pts[I], Pts[Start], Pts[End]);
			if (D > MaxD)
			{
				MaxD = D;
				MaxI = I;
			}
		}
		if (MaxD > EpsSq && MaxI > Start && MaxI < End)
		{
			RdpKeep(Pts, Start, MaxI, EpsSq, Keep);
			RdpKeep(Pts, MaxI, End, EpsSq, Keep);
		}
		else
		{
			Keep[Start] = 1;
			Keep[End] = 1;
		}
	}

	void ApplyRdp(const TArray<FVector2D>& Unique, double EpsDeg, TArray<FVector2D>& Out)
	{
		Out.Reset();
		if (Unique.Num() < 3 || EpsDeg <= DuplicateEpsDeg)
		{
			Out = Unique;
			return;
		}
		TArray<uint8> Keep;
		Keep.Init(0, Unique.Num());
		RdpKeep(Unique, 0, Unique.Num() - 1, EpsDeg * EpsDeg, Keep);
		Keep[0] = 1;
		Keep.Last() = 1;
		Out.Reserve(Unique.Num());
		for (int32 I = 0; I < Unique.Num(); ++I)
		{
			if (Keep[I])
			{
				Out.Add(Unique[I]);
			}
		}
		if (Out.Num() < 3)
		{
			Out = Unique;
		}
	}

	void UniformSample(const TArray<FVector2D>& In, int32 MaxPoints, TArray<FVector2D>& Out)
	{
		Out.Reset();
		if (In.Num() == 0 || MaxPoints < 3)
		{
			return;
		}
		if (In.Num() <= MaxPoints)
		{
			Out = In;
			return;
		}
		Out.Reserve(MaxPoints);
		for (int32 I = 0; I < MaxPoints; ++I)
		{
			const int32 Src = (I * (In.Num() - 1)) / (MaxPoints - 1);
			Out.Add(In[Src]);
		}
		if (Out.Num() >= 2 && Out[0].Equals(Out.Last(), DuplicateEpsDeg))
		{
			Out.Pop();
		}
	}

	void ChaikinClosed(const TArray<FVector2D>& In, TArray<FVector2D>& Out)
	{
		Out.Reset();
		const int32 N = In.Num();
		if (N < 3)
		{
			Out = In;
			return;
		}
		Out.Reserve(N * 2);
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D& A = In[I];
			const FVector2D& B = In[(I + 1) % N];
			Out.Add(A * 0.75 + B * 0.25);
			Out.Add(A * 0.25 + B * 0.75);
		}
	}

	void DecimateRing(const TArray<FVector2D>& In, int32 MaxPoints, double SmoothMeters, TArray<FVector2D>& Out)
	{
		Out.Reset();
		if (In.Num() == 0)
		{
			return;
		}
		TArray<FVector2D> Unique;
		Unique.Reserve(In.Num());
		for (const FVector2D& P : In)
		{
			if (Unique.Num() == 0 || !Unique.Last().Equals(P, DuplicateEpsDeg))
			{
				Unique.Add(P);
			}
		}
		if (Unique.Num() >= 2 && Unique[0].Equals(Unique.Last(), DuplicateEpsDeg))
		{
			Unique.Pop();
		}

		TArray<FVector2D> Working;
		if (SmoothMeters > 0.0)
		{
			ApplyRdp(Unique, SmoothMeters / 111320.0, Working);
			TArray<FVector2D> Rounded;
			ChaikinClosed(Working, Rounded);
			ChaikinClosed(Rounded, Working);
		}
		else
		{
			Working = MoveTemp(Unique);
		}

		if (MaxPoints <= 0 || Working.Num() <= MaxPoints)
		{
			Out = MoveTemp(Working);
			return;
		}
		ApplyRdp(Working, FMath::Max(SmoothMeters, 1.0) / 111320.0, Out);
		if (Out.Num() > MaxPoints)
		{
			UniformSample(Out, MaxPoints, Working);
			Out = MoveTemp(Working);
		}
	}

	void DensifyClosedRing(const TArray<FVector2D>& In, double MaxEdgeMeters, TArray<FVector2D>& Out)
	{
		Out.Reset();
		if (In.Num() < 3)
		{
			Out = In;
			return;
		}
		const double MaxDeg = FMath::Max(MaxEdgeMeters, 5.0) / 111320.0;
		const int32 N = In.Num();
		Out.Reserve(N * 2);
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D A = In[I];
			const FVector2D B = In[(I + 1) % N];
			Out.Add(A);
			const double DistDeg = FVector2D::Distance(A, B);
			const int32 Segments = FMath::Max(
				1,
				FMath::CeilToInt(static_cast<float>(DistDeg / FMath::Max(MaxDeg, 1.0e-12))));
			for (int32 S = 1; S < Segments; ++S)
			{
				Out.Add(A + (B - A) * (static_cast<double>(S) / Segments));
			}
		}
	}

	double EdgeMetersLonLat(double Lon0, double Lat0, double Lon1, double Lat1)
	{
		const double MidLat = 0.5 * (Lat0 + Lat1);
		const double MetersLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(MidLat)), 0.05);
		const double Dx = (Lon0 - Lon1) * MetersLon;
		const double Dy = (Lat0 - Lat1) * 110540.0;
		return FMath::Sqrt(Dx * Dx + Dy * Dy);
	}

	struct FLocalFrame
	{
		double OriginLon = 0.0;
		double OriginLat = 0.0;
		double MetersLon = 111320.0;
		double MetersLat = 110540.0;

		FVector2D ToXY(double Lon, double Lat) const
		{
			return FVector2D((Lon - OriginLon) * MetersLon, (Lat - OriginLat) * MetersLat);
		}

		void FromXY(const FVector2D& XY, double& OutLon, double& OutLat) const
		{
			OutLon = OriginLon + XY.X / MetersLon;
			OutLat = OriginLat + XY.Y / MetersLat;
		}
	};

	FLocalFrame MakeFrame(const TArray<FVector2D>& Ring)
	{
		FLocalFrame Frame;
		if (Ring.Num() == 0)
		{
			return Frame;
		}
		double Lon = 0.0;
		double Lat = 0.0;
		for (const FVector2D& P : Ring)
		{
			Lon += P.X;
			Lat += P.Y;
		}
		Frame.OriginLon = Lon / static_cast<double>(Ring.Num());
		Frame.OriginLat = Lat / static_cast<double>(Ring.Num());
		Frame.MetersLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(Frame.OriginLat)), 0.05);
		Frame.MetersLat = 110540.0;
		return Frame;
	}

	struct FElevGrid
	{
		double OriginLon = 0.0;
		double OriginLat = 0.0;
		double CellDeg = 0.0002;
		TMap<uint64, TArray<int32>> Cells;

		static uint64 Key(int32 X, int32 Y)
		{
			return (static_cast<uint64>(static_cast<uint32>(X)) << 32) | static_cast<uint32>(Y);
		}

		void Build(const TArray<FWaterShapefilePoint>& Pts, const FBox2D& Bounds)
		{
			OriginLon = Bounds.bIsValid ? Bounds.Min.X : 0.0;
			OriginLat = Bounds.bIsValid ? Bounds.Min.Y : 0.0;
			for (int32 I = 0; I < Pts.Num(); ++I)
			{
				const int32 CX = FMath::FloorToInt((Pts[I].LonDeg - OriginLon) / CellDeg);
				const int32 CY = FMath::FloorToInt((Pts[I].LatDeg - OriginLat) / CellDeg);
				Cells.FindOrAdd(Key(CX, CY)).Add(I);
			}
		}

		int32 Nearest(
			double Lon,
			double Lat,
			const TArray<FWaterShapefilePoint>& Pts,
			double MaxMeters,
			double& OutDistM) const
		{
			const double MidLat = Lat;
			const double MetersLon = 111320.0 * FMath::Max(FMath::Cos(FMath::DegreesToRadians(MidLat)), 0.05);
			const double MaxDeg = MaxMeters / FMath::Min(MetersLon, 110540.0);
			const int32 Pad = FMath::Max(1, FMath::CeilToInt(static_cast<float>(MaxDeg / CellDeg)) + 1);
			const int32 CX = FMath::FloorToInt((Lon - OriginLon) / CellDeg);
			const int32 CY = FMath::FloorToInt((Lat - OriginLat) / CellDeg);
			int32 Best = INDEX_NONE;
			double BestM = MaxMeters;
			for (int32 DY = -Pad; DY <= Pad; ++DY)
			{
				for (int32 DX = -Pad; DX <= Pad; ++DX)
				{
					const TArray<int32>* Bucket = Cells.Find(Key(CX + DX, CY + DY));
					if (!Bucket)
					{
						continue;
					}
					for (const int32 I : *Bucket)
					{
						const double D = EdgeMetersLonLat(Lon, Lat, Pts[I].LonDeg, Pts[I].LatDeg);
						if (D < BestM)
						{
							BestM = D;
							Best = I;
						}
					}
				}
			}
			OutDistM = BestM;
			return Best;
		}
	};

	double TpsPhi(double R)
	{
		if (R < 1.0e-9)
		{
			return 0.0;
		}
		return R * R * FMath::Loge(R);
	}

	bool GaussSolve(TArray<double>& A, TArray<double>& B, int32 N)
	{
		if (N <= 0 || A.Num() != N * N || B.Num() != N)
		{
			return false;
		}
		for (int32 K = 0; K < N; ++K)
		{
			int32 Pivot = K;
			double Best = FMath::Abs(A[K * N + K]);
			for (int32 I = K + 1; I < N; ++I)
			{
				const double V = FMath::Abs(A[I * N + K]);
				if (V > Best)
				{
					Best = V;
					Pivot = I;
				}
			}
			if (Best < 1.0e-12)
			{
				return false;
			}
			if (Pivot != K)
			{
				for (int32 J = K; J < N; ++J)
				{
					Swap(A[K * N + J], A[Pivot * N + J]);
				}
				Swap(B[K], B[Pivot]);
			}
			const double Diag = A[K * N + K];
			for (int32 I = K + 1; I < N; ++I)
			{
				const double F = A[I * N + K] / Diag;
				A[I * N + K] = 0.0;
				for (int32 J = K + 1; J < N; ++J)
				{
					A[I * N + J] -= F * A[K * N + J];
				}
				B[I] -= F * B[K];
			}
		}
		for (int32 I = N - 1; I >= 0; --I)
		{
			double Sum = B[I];
			for (int32 J = I + 1; J < N; ++J)
			{
				Sum -= A[I * N + J] * B[J];
			}
			const double Diag = A[I * N + I];
			if (FMath::Abs(Diag) < 1.0e-12)
			{
				return false;
			}
			B[I] = Sum / Diag;
		}
		return true;
	}

	struct FTpsFit
	{
		TArray<FVector2D> CtrlXY;
		TArray<double> Weights;
		double A0 = 0.0;
		double AX = 0.0;
		double AY = 0.0;
		FVector2D Origin = FVector2D::ZeroVector;
		double Scale = 1.0;

		FVector2D Normalize(const FVector2D& XY) const
		{
			return FVector2D((XY.X - Origin.X) / Scale, (XY.Y - Origin.Y) / Scale);
		}

		bool Fit(const TArray<FVector2D>& XY, const TArray<double>& Z, FString& OutError)
		{
			const int32 C = XY.Num();
			if (C < 3 || Z.Num() != C)
			{
				OutError = TEXT("Need at least 3 outline height controls for the water surface.");
				return false;
			}

			Origin = FVector2D::ZeroVector;
			for (const FVector2D& P : XY)
			{
				Origin += P;
			}
			Origin /= static_cast<double>(C);
			Scale = 1.0;
			for (const FVector2D& P : XY)
			{
				Scale = FMath::Max(Scale, FVector2D::Distance(P, Origin));
			}
			Scale = FMath::Max(Scale, 1.0);

			TArray<FVector2D> Data;
			Data.SetNum(C);
			for (int32 I = 0; I < C; ++I)
			{
				Data[I] = Normalize(XY[I]);
			}

			const int32 M = FMath::Min(C, MaxTpsSolveCenters);
			CtrlXY.SetNum(M);
			for (int32 I = 0; I < M; ++I)
			{
				const int32 Src = (C == 1) ? 0 : (I * (C - 1)) / FMath::Max(M - 1, 1);
				CtrlXY[I] = Data[Src];
			}

			const int32 U = M + 3;
			const double Lambda = 1.0e-4;
			TArray<double> A;
			A.SetNumZeroed(U * U);
			TArray<double> B;
			B.SetNumZeroed(U);

			if (C <= MaxTpsSolveCenters)
			{
				for (int32 I = 0; I < C; ++I)
				{
					for (int32 J = 0; J < C; ++J)
					{
						double V = TpsPhi(FVector2D::Distance(CtrlXY[I], CtrlXY[J]));
						if (I == J)
						{
							V += Lambda;
						}
						A[I * U + J] = V;
					}
					A[I * U + C] = 1.0;
					A[I * U + C + 1] = CtrlXY[I].X;
					A[I * U + C + 2] = CtrlXY[I].Y;
					A[C * U + I] = 1.0;
					A[(C + 1) * U + I] = CtrlXY[I].X;
					A[(C + 2) * U + I] = CtrlXY[I].Y;
					B[I] = Z[I];
				}
			}
			else
			{
				TArray<double> Row;
				Row.SetNum(U);
				for (int32 I = 0; I < C; ++I)
				{
					for (int32 J = 0; J < M; ++J)
					{
						Row[J] = TpsPhi(FVector2D::Distance(Data[I], CtrlXY[J]));
					}
					Row[M] = 1.0;
					Row[M + 1] = Data[I].X;
					Row[M + 2] = Data[I].Y;
					for (int32 R = 0; R < U; ++R)
					{
						B[R] += Row[R] * Z[I];
						for (int32 Col = R; Col < U; ++Col)
						{
							A[R * U + Col] += Row[R] * Row[Col];
						}
					}
				}
				for (int32 R = 0; R < U; ++R)
				{
					for (int32 Col = 0; Col < R; ++Col)
					{
						A[R * U + Col] = A[Col * U + R];
					}
				}
				for (int32 J = 0; J < M; ++J)
				{
					A[J * U + J] += Lambda;
				}
				UE_LOG(
					LogWaterPlacer,
					Display,
					TEXT("Thin-plate least-squares using all %d height samples with %d kernel centers."),
					C,
					M);
			}

			if (!GaussSolve(A, B, U))
			{
				OutError = TEXT("Failed to solve the water thin-plate surface (singular control set).");
				return false;
			}

			Weights.SetNum(M);
			for (int32 I = 0; I < M; ++I)
			{
				Weights[I] = B[I];
			}
			A0 = B[M];
			AX = B[M + 1];
			AY = B[M + 2];
			return true;
		}

		double Evaluate(const FVector2D& XY) const
		{
			const FVector2D U = Normalize(XY);
			double H = A0 + AX * U.X + AY * U.Y;
			for (int32 I = 0; I < CtrlXY.Num(); ++I)
			{
				H += Weights[I] * TpsPhi(FVector2D::Distance(U, CtrlXY[I]));
			}
			return H;
		}
	};

	double Orient2D(const FVector2D& A, const FVector2D& B, const FVector2D& C)
	{
		return (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
	}

	bool CircumcircleContains(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& P)
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

	bool BuildTin(
		const TArray<FWaterSample>& Samples,
		const TArray<FVector2D>& MaskRing,
		const FBox2D& MaskBounds,
		FWaterTin& OutTin,
		FString& OutError,
		TFunction<bool(float, const TCHAR*)> Progress)
	{
		OutTin = FWaterTin();
		if (Samples.Num() < 3)
		{
			OutError = TEXT("Need at least 3 water samples to triangulate.");
			return false;
		}

		TArray<FWaterSample> Pts;
		Pts.Reserve(Samples.Num());
		TSet<uint64> Seen;
		auto QuantKey = [](double Lon, double Lat) -> uint64
		{
			const int32 Qx = FMath::RoundToInt(Lon * 1.0e7);
			const int32 Qy = FMath::RoundToInt(Lat * 1.0e7);
			return (static_cast<uint64>(static_cast<uint32>(Qx)) << 32) | static_cast<uint32>(Qy);
		};
		for (const FWaterSample& S : Samples)
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
			OutError = TEXT("All water samples collapsed to fewer than 3 unique points.");
			return false;
		}

		double MinX = Pts[0].Lon, MaxX = Pts[0].Lon, MinY = Pts[0].Lat, MaxY = Pts[0].Lat;
		for (const FWaterSample& P : Pts)
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
		for (const FWaterSample& P : Pts)
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
		Pts.Add(FWaterSample());
		Pts.Add(FWaterSample());
		Pts.Add(FWaterSample());

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
		const int32 InsertStride = FMath::Max(S0 / 200, 32);
		for (int32 Pi = 0; Pi < S0; ++Pi)
		{
			if ((Pi % InsertStride) == 0 || Pi + 1 == S0)
			{
				const float Frac = 0.15f + 0.70f * static_cast<float>(Pi + 1) / static_cast<float>(S0);
				if (!Report(Frac, TEXT("triangulating water")))
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

		TArray<int32> Remap;
		Remap.Init(INDEX_NONE, S0);
		TArray<uint8> Used;
		Used.Init(0, S0);
		for (int32 Ti = 0; Ti < Tris.Num(); ++Ti)
		{
			if (!Alive[Ti])
			{
				continue;
			}
			const FDelTri& Tri = Tris[Ti];
			if (Tri.V[0] >= S0 || Tri.V[1] >= S0 || Tri.V[2] >= S0)
			{
				continue;
			}
			const FWaterSample& A = Pts[Tri.V[0]];
			const FWaterSample& B = Pts[Tri.V[1]];
			const FWaterSample& C = Pts[Tri.V[2]];
			const FVector2D Centroid(
				(A.Lon + B.Lon + C.Lon) / 3.0,
				(A.Lat + B.Lat + C.Lat) / 3.0);
			if (MaskBounds.bIsValid && !BoxContains(MaskBounds, Centroid))
			{
				continue;
			}
			if (!PointInRing(Centroid, MaskRing))
			{
				continue;
			}
			Used[Tri.V[0]] = 1;
			Used[Tri.V[1]] = 1;
			Used[Tri.V[2]] = 1;
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

		for (int32 I = 0; I < S0; ++I)
		{
			if (Used[I])
			{
				Remap[I] = OutTin.Vertices.Add(Pts[I]);
			}
		}
		for (int32& Idx : OutTin.Triangles)
		{
			Idx = Remap[Idx];
		}

		if (OutTin.Triangles.Num() < 3)
		{
			OutError = TEXT("No triangles remained inside the water mask.");
			return false;
		}
		return true;
	}
}

bool WaterSurface::BuildInterpolatedLake(
	const FWaterShapefilePolygon& Mask,
	const TArray<FWaterShapefilePoint>& ElevationPoints,
	int32 MaxOutlineVertices,
	double OutlineSmoothMeters,
	double InteriorSpacingMeters,
	double ControlSpacingMeters,
	FWaterSurfaceResult& OutSurface,
	FString& OutError,
	TFunction<bool(float, const TCHAR*)> Progress)
{
	OutSurface = FWaterSurfaceResult();
	const int32 OutlineCap = MaxOutlineVertices;
	const double SmoothM = FMath::Max(OutlineSmoothMeters, 0.0);
	const double SpacingM = FMath::Max(InteriorSpacingMeters, 1.0);
	const double CtrlSpacingM = FMath::Max(ControlSpacingMeters, 0.0);

	auto Report = [&](float Frac, const TCHAR* Stage) -> bool
	{
		if (!Progress)
		{
			return true;
		}
		return Progress(FMath::Clamp(Frac, 0.0f, 1.0f), Stage);
	};

	if (!Report(0.02f, TEXT("simplifying shoreline")))
	{
		OutError = TEXT("Cancelled.");
		return false;
	}

	TArray<FVector2D> Outline;
	DecimateRing(Mask.OuterRingLonLat, OutlineCap, SmoothM, Outline);
	if (Outline.Num() < 3)
	{
		OutError = TEXT("Water mask outline has fewer than 3 vertices after simplify.");
		return false;
	}

	const FLocalFrame Frame = MakeFrame(Outline);
	FBox2D PadBounds = Mask.Bounds;
	if (PadBounds.bIsValid)
	{
		const double PadDeg = 40.0 / 111320.0;
		PadBounds.Min.X -= PadDeg;
		PadBounds.Min.Y -= PadDeg;
		PadBounds.Max.X += PadDeg;
		PadBounds.Max.Y += PadDeg;
	}

	TArray<int32> NearbyPts;
	NearbyPts.Reserve(1024);
	for (int32 I = 0; I < ElevationPoints.Num(); ++I)
	{
		const FVector2D P(ElevationPoints[I].LonDeg, ElevationPoints[I].LatDeg);
		if (PadBounds.bIsValid && !BoxContains(PadBounds, P))
		{
			continue;
		}
		NearbyPts.Add(I);
	}
	if (NearbyPts.Num() < 3)
	{
		OutError = FString::Printf(
			TEXT("Mask record %d has fewer than 3 elevation points in its padded bounds."),
			Mask.RecordIndex);
		return false;
	}

	FElevGrid Grid;
	Grid.Build(ElevationPoints, PadBounds);

	TArray<FVector2D> CtrlXY;
	TArray<double> CtrlZ;
	TSet<uint64> CtrlSeen;
	auto TryAddControl = [&](double Lon, double Lat, double HeightM)
	{
		const int32 Qx = FMath::RoundToInt(Lon * 2.0e5);
		const int32 Qy = FMath::RoundToInt(Lat * 2.0e5);
		const uint64 Key = (static_cast<uint64>(static_cast<uint32>(Qx)) << 32) | static_cast<uint32>(Qy);
		if (CtrlSeen.Contains(Key))
		{
			return;
		}
		CtrlSeen.Add(Key);
		CtrlXY.Add(Frame.ToXY(Lon, Lat));
		CtrlZ.Add(HeightM);
	};

	if (CtrlSpacingM <= 0.0)
	{
		for (const int32 I : NearbyPts)
		{
			TryAddControl(ElevationPoints[I].LonDeg, ElevationPoints[I].LatDeg, ElevationPoints[I].HeightM);
		}
	}
	else
	{
		TArray<FVector2D> ControlWalk;
		DensifyClosedRing(Outline, CtrlSpacingM, ControlWalk);
		for (const FVector2D& P : ControlWalk)
		{
			double DistM = 0.0;
			const int32 Idx = Grid.Nearest(P.X, P.Y, ElevationPoints, 40.0, DistM);
			if (Idx != INDEX_NONE)
			{
				TryAddControl(ElevationPoints[Idx].LonDeg, ElevationPoints[Idx].LatDeg, ElevationPoints[Idx].HeightM);
			}
		}
	}

	int32 IMin = NearbyPts[0];
	int32 IMax = NearbyPts[0];
	for (const int32 I : NearbyPts)
	{
		if (ElevationPoints[I].HeightM < ElevationPoints[IMin].HeightM)
		{
			IMin = I;
		}
		if (ElevationPoints[I].HeightM > ElevationPoints[IMax].HeightM)
		{
			IMax = I;
		}
	}
	TryAddControl(ElevationPoints[IMin].LonDeg, ElevationPoints[IMin].LatDeg, ElevationPoints[IMin].HeightM);
	TryAddControl(ElevationPoints[IMax].LonDeg, ElevationPoints[IMax].LatDeg, ElevationPoints[IMax].HeightM);

	if (CtrlXY.Num() < MinControlPoints && NearbyPts.Num() >= MinControlPoints)
	{
		const int32 Step = FMath::Max(1, NearbyPts.Num() / MinControlPoints);
		for (int32 I = 0; I < NearbyPts.Num() && CtrlXY.Num() < MinControlPoints; I += Step)
		{
			const FWaterShapefilePoint& P = ElevationPoints[NearbyPts[I]];
			TryAddControl(P.LonDeg, P.LatDeg, P.HeightM);
		}
	}

	if (CtrlXY.Num() < 3)
	{
		OutError = TEXT("Could not collect enough shoreline height controls.");
		return false;
	}

	if (!Report(0.10f, TEXT("fitting water surface")))
	{
		OutError = TEXT("Cancelled.");
		return false;
	}

	FTpsFit Fit;
	if (!Fit.Fit(CtrlXY, CtrlZ, OutError))
	{
		return false;
	}

	double Rms = 0.0;
	for (int32 I = 0; I < CtrlXY.Num(); ++I)
	{
		const double D = Fit.Evaluate(CtrlXY[I]) - CtrlZ[I];
		Rms += D * D;
	}
	Rms = FMath::Sqrt(Rms / static_cast<double>(CtrlXY.Num()));

	double MinX = 0.0, MaxX = 0.0, MinY = 0.0, MaxY = 0.0;
	{
		const FVector2D P0 = Frame.ToXY(Outline[0].X, Outline[0].Y);
		MinX = MaxX = P0.X;
		MinY = MaxY = P0.Y;
		for (const FVector2D& LonLat : Outline)
		{
			const FVector2D P = Frame.ToXY(LonLat.X, LonLat.Y);
			MinX = FMath::Min(MinX, P.X);
			MaxX = FMath::Max(MaxX, P.X);
			MinY = FMath::Min(MinY, P.Y);
			MaxY = FMath::Max(MaxY, P.Y);
		}
	}
	TArray<FWaterSample> Samples;
	Samples.Reserve(Outline.Num() + 256);
	OutSurface.Outline.Reserve(Outline.Num());
	for (const FVector2D& LonLat : Outline)
	{
		FWaterSample S;
		S.Lon = LonLat.X;
		S.Lat = LonLat.Y;
		S.HeightM = Fit.Evaluate(Frame.ToXY(LonLat.X, LonLat.Y));
		Samples.Add(S);
		OutSurface.Outline.Add(S);
	}

	const double InsetM = 0.45 * SpacingM;
	const double InsetDegSq = FMath::Square(InsetM / 111320.0);
	int32 Interior = 0;
	for (double Y = MinY + 0.5 * SpacingM; Y < MaxY; Y += SpacingM)
	{
		for (double X = MinX + 0.5 * SpacingM; X < MaxX; X += SpacingM)
		{
			double Lon = 0.0;
			double Lat = 0.0;
			Frame.FromXY(FVector2D(X, Y), Lon, Lat);
			const FVector2D LonLat(Lon, Lat);
			if (Mask.Bounds.bIsValid && !BoxContains(Mask.Bounds, LonLat))
			{
				continue;
			}
			if (!PointInRing(LonLat, Outline))
			{
				continue;
			}
			if (RingDistSqDeg(LonLat, Outline) < InsetDegSq)
			{
				continue;
			}
			FWaterSample S;
			S.Lon = Lon;
			S.Lat = Lat;
			S.HeightM = Fit.Evaluate(FVector2D(X, Y));
			Samples.Add(S);
			++Interior;
		}
	}

	if (!Report(0.14f, TEXT("triangulating water")))
	{
		OutError = TEXT("Cancelled.");
		return false;
	}

	if (!BuildTin(Samples, Outline, Mask.Bounds, OutSurface.Tin, OutError, Progress))
	{
		return false;
	}

	OutSurface.ControlPoints = CtrlXY.Num();
	OutSurface.InteriorVertices = Interior;
	OutSurface.ControlRmsM = Rms;
	OutSurface.HeightMinM = OutSurface.Tin.Vertices[0].HeightM;
	OutSurface.HeightMaxM = OutSurface.Tin.Vertices[0].HeightM;
	for (const FWaterSample& V : OutSurface.Tin.Vertices)
	{
		OutSurface.HeightMinM = FMath::Min(OutSurface.HeightMinM, V.HeightM);
		OutSurface.HeightMaxM = FMath::Max(OutSurface.HeightMaxM, V.HeightM);
	}

	UE_LOG(
		LogWaterPlacer,
		Display,
		TEXT("Lake %d: %d outline, %d interior, %d TPS controls (rms %.3f m), surface Z %.3f..%.3f m, %d tris, spacing %.1f m."),
		Mask.RecordIndex,
		Outline.Num(),
		Interior,
		OutSurface.ControlPoints,
		OutSurface.ControlRmsM,
		OutSurface.HeightMinM,
		OutSurface.HeightMaxM,
		OutSurface.Tin.Triangles.Num() / 3,
		SpacingM);
	return true;
}
