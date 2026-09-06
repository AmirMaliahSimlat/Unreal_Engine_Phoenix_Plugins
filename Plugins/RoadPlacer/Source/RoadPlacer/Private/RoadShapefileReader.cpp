#include "RoadShapefileReader.h"
#include "RoadPlacerLog.h"

#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
	int32 ReadInt32LE(const TArray<uint8>& Data, int32 Offset)
	{
		return static_cast<int32>(Data[Offset])
			| (static_cast<int32>(Data[Offset + 1]) << 8)
			| (static_cast<int32>(Data[Offset + 2]) << 16)
			| (static_cast<int32>(Data[Offset + 3]) << 24);
	}

	int32 ReadInt32BE(const TArray<uint8>& Data, int32 Offset)
	{
		return (static_cast<int32>(Data[Offset]) << 24)
			| (static_cast<int32>(Data[Offset + 1]) << 16)
			| (static_cast<int32>(Data[Offset + 2]) << 8)
			| static_cast<int32>(Data[Offset + 3]);
	}

	double ReadDoubleLE(const TArray<uint8>& Data, int32 Offset)
	{
		uint64 Bits = 0;
		for (int32 I = 0; I < 8; ++I)
		{
			Bits |= (static_cast<uint64>(Data[Offset + I]) << (8 * I));
		}
		double Value = 0.0;
		FMemory::Memcpy(&Value, &Bits, sizeof(double));
		return Value;
	}

	FString NormalizeShpPath(const FString& InPath)
	{
		FString Path = InPath.TrimStartAndEnd();
		while ((Path.StartsWith(TEXT("\"")) && Path.EndsWith(TEXT("\"")) && Path.Len() >= 2)
			|| (Path.StartsWith(TEXT("'")) && Path.EndsWith(TEXT("'")) && Path.Len() >= 2))
		{
			Path = Path.Mid(1, Path.Len() - 2).TrimStartAndEnd();
		}
		const FString Ext = FPaths::GetExtension(Path);
		if (Ext.Equals(TEXT("shp"), ESearchCase::IgnoreCase)
			|| Ext.Equals(TEXT("shx"), ESearchCase::IgnoreCase)
			|| Ext.Equals(TEXT("dbf"), ESearchCase::IgnoreCase)
			|| Ext.Equals(TEXT("prj"), ESearchCase::IgnoreCase))
		{
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path));
		}
		return Path;
	}

	bool ValidateEpsg4326Prj(const FString& PrjPath, FString& OutError)
	{
		if (!FPaths::FileExists(PrjPath))
		{
			UE_LOG(LogRoadPlacer, Display, TEXT("No .prj found; assuming EPSG:4326 (WGS84 lon/lat degrees)."));
			return true;
		}

		FString PrjText;
		if (!FFileHelper::LoadFileToString(PrjText, *PrjPath))
		{
			UE_LOG(LogRoadPlacer, Warning, TEXT("Could not read .prj; assuming EPSG:4326."));
			return true;
		}

		const FString Upper = PrjText.ToUpper();
		const bool bLooks4326 =
			Upper.Contains(TEXT("EPSG\",\"4326"))
			|| Upper.Contains(TEXT("AUTHORITY[\"EPSG\",\"4326\"]"))
			|| Upper.Contains(TEXT("WGS_1984"))
			|| Upper.Contains(TEXT("WGS 84"))
			|| Upper.Contains(TEXT("GCS_WGS_1984"))
			|| Upper.Contains(TEXT("WGS84"));

		if (bLooks4326)
		{
			return true;
		}

		if (Upper.Contains(TEXT("PROJCS[")) || Upper.Contains(TEXT("PROJCS \"")))
		{
			OutError = FString::Printf(
				TEXT("Shapefile .prj is a projected CRS, not EPSG:4326. Reproject to EPSG:4326 before import. File: %s"),
				*PrjPath);
			return false;
		}

		UE_LOG(LogRoadPlacer, Warning, TEXT("Unrecognized .prj; proceeding as lon/lat degrees. File: %s"), *PrjPath);
		return true;
	}

	struct FDbfField
	{
		FString Name;
		ANSICHAR Type = 'C';
		int32 Length = 0;
		int32 Decimal = 0;
		int32 OffsetInRecord = 0;
	};

	bool LoadDbf(
		const FString& DbfPath,
		TArray<FDbfField>& OutFields,
		TArray<TArray<uint8>>& OutRecords,
		FString& OutError)
	{
		TArray<uint8> Data;
		if (!FFileHelper::LoadFileToArray(Data, *DbfPath) || Data.Num() < 32)
		{
			OutError = FString::Printf(TEXT("Failed to read DBF: %s"), *DbfPath);
			return false;
		}

		const uint8 HeaderLenLo = Data[8];
		const uint8 HeaderLenHi = Data[9];
		const int32 HeaderBytes = static_cast<int32>(HeaderLenLo) | (static_cast<int32>(HeaderLenHi) << 8);
		const int32 RecordBytes = static_cast<int32>(Data[10]) | (static_cast<int32>(Data[11]) << 8);
		const int32 NumRecords = ReadInt32LE(Data, 4);
		if (HeaderBytes < 33 || RecordBytes < 1 || NumRecords < 0)
		{
			OutError = TEXT("Invalid DBF header.");
			return false;
		}

		int32 FieldOffset = 32;
		int32 InRecord = 1;
		while (FieldOffset + 32 <= HeaderBytes && Data[FieldOffset] != 0x0D)
		{
			FDbfField Field;
			char NameBuf[12];
			FMemory::Memzero(NameBuf, 12);
			FMemory::Memcpy(NameBuf, Data.GetData() + FieldOffset, 11);
			Field.Name = ANSI_TO_TCHAR(NameBuf);
			Field.Name.TrimStartAndEndInline();
			Field.Type = static_cast<ANSICHAR>(Data[FieldOffset + 11]);
			Field.Length = Data[FieldOffset + 16];
			Field.Decimal = Data[FieldOffset + 17];
			Field.OffsetInRecord = InRecord;
			InRecord += Field.Length;
			OutFields.Add(Field);
			FieldOffset += 32;
		}

		const int32 DataStart = HeaderBytes;
		OutRecords.Reset();
		OutRecords.Reserve(NumRecords);
		for (int32 R = 0; R < NumRecords; ++R)
		{
			const int32 Off = DataStart + R * RecordBytes;
			if (Off + RecordBytes > Data.Num())
			{
				break;
			}
			TArray<uint8> Rec;
			Rec.Append(Data.GetData() + Off, RecordBytes);
			OutRecords.Add(MoveTemp(Rec));
		}
		return true;
	}

	const FDbfField* FindField(const TArray<FDbfField>& Fields, const FString& Name)
	{
		for (const FDbfField& F : Fields)
		{
			if (F.Name.Equals(Name, ESearchCase::IgnoreCase))
			{
				return &F;
			}
		}
		return nullptr;
	}

	bool ParseNumericField(const TArray<uint8>& Rec, const FDbfField& Field, double& Out)
	{
		if (Field.OffsetInRecord + Field.Length > Rec.Num())
		{
			return false;
		}
		FString Text;
		Text.Reserve(Field.Length);
		for (int32 I = 0; I < Field.Length; ++I)
		{
			Text.AppendChar(static_cast<TCHAR>(Rec[Field.OffsetInRecord + I]));
		}
		Text.TrimStartAndEndInline();
		if (Text.IsEmpty() || Text == TEXT("*"))
		{
			return false;
		}
		Out = FCString::Atod(*Text);
		return FMath::IsFinite(Out);
	}

	void StripClose(FRoadShapefileRing& Ring)
	{
		if (Ring.LonLat.Num() >= 2 && Ring.LonLat[0].Equals(Ring.LonLat.Last(), 1.0e-12))
		{
			Ring.LonLat.Pop();
			if (Ring.HeightM.Num() == Ring.LonLat.Num() + 1)
			{
				Ring.HeightM.Pop();
			}
		}
	}

	double SignedArea(const TArray<FVector2D>& Ring)
	{
		double A = 0.0;
		const int32 N = Ring.Num();
		for (int32 I = 0; I < N; ++I)
		{
			const FVector2D& P = Ring[I];
			const FVector2D& Q = Ring[(I + 1) % N];
			A += P.X * Q.Y - Q.X * P.Y;
		}
		return 0.5 * A;
	}

	FVector2D RingCentroid(const TArray<FVector2D>& Ring)
	{
		FVector2D C = FVector2D::ZeroVector;
		for (const FVector2D& P : Ring)
		{
			C += P;
		}
		return C / static_cast<double>(FMath::Max(Ring.Num(), 1));
	}

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

}

bool RoadShapefileReader::ReadMaskPolygons(
	const FString& ShapefilePath,
	TArray<FRoadShapefileMask>& OutMasks,
	FString& OutError)
{
	OutMasks.Reset();
	const FString Base = NormalizeShpPath(ShapefilePath);
	const FString ShpPath = Base + TEXT(".shp");
	const FString PrjPath = Base + TEXT(".prj");
	if (!FPaths::FileExists(ShpPath))
	{
		OutError = FString::Printf(TEXT("SHP not found: %s"), *ShpPath);
		return false;
	}
	if (!ValidateEpsg4326Prj(PrjPath, OutError))
	{
		return false;
	}

	TArray<uint8> ShpData;
	if (!FFileHelper::LoadFileToArray(ShpData, *ShpPath) || ShpData.Num() < 100)
	{
		OutError = FString::Printf(TEXT("Failed to read SHP: %s"), *ShpPath);
		return false;
	}
	if (ReadInt32BE(ShpData, 0) != 9994)
	{
		OutError = TEXT("Not a valid ESRI shapefile (bad file code).");
		return false;
	}

	const int32 ShapeType = ReadInt32LE(ShpData, 32);
	if (ShapeType != 5 && ShapeType != 15 && ShapeType != 25)
	{
		OutError = FString::Printf(TEXT("Road mask must be Polygon / PolygonZ (type %d)."), ShapeType);
		return false;
	}

	int32 Offset = 100;
	int32 RecordIndex = 0;
	while (Offset + 8 <= ShpData.Num())
	{
		const int32 ContentBytes = ReadInt32BE(ShpData, Offset + 4) * 2;
		Offset += 8;
		if (ContentBytes <= 0 || Offset + ContentBytes > ShpData.Num())
		{
			break;
		}

		const int32 RecType = ReadInt32LE(ShpData, Offset);
		if (RecType == 0 || (RecType != 5 && RecType != 15 && RecType != 25))
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}

		int32 Cursor = Offset + 4 + 32;
		if (Cursor + 8 > Offset + ContentBytes)
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}

		const int32 NumParts = ReadInt32LE(ShpData, Cursor);
		const int32 NumPoints = ReadInt32LE(ShpData, Cursor + 4);
		Cursor += 8;
		if (NumParts <= 0 || NumPoints < 3 || Cursor + NumParts * 4 + NumPoints * 16 > Offset + ContentBytes)
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}

		TArray<int32> Parts;
		Parts.SetNum(NumParts);
		for (int32 P = 0; P < NumParts; ++P)
		{
			Parts[P] = ReadInt32LE(ShpData, Cursor + P * 4);
		}
		Cursor += NumParts * 4;

		TArray<FVector2D> AllXY;
		AllXY.SetNum(NumPoints);
		for (int32 P = 0; P < NumPoints; ++P)
		{
			AllXY[P] = FVector2D(ReadDoubleLE(ShpData, Cursor + P * 16), ReadDoubleLE(ShpData, Cursor + P * 16 + 8));
		}
		Cursor += NumPoints * 16;

		TArray<double> AllZ;
		if (RecType == 15 && Cursor + 16 + NumPoints * 8 <= Offset + ContentBytes)
		{
			Cursor += 16;
			AllZ.SetNum(NumPoints);
			for (int32 P = 0; P < NumPoints; ++P)
			{
				AllZ[P] = ReadDoubleLE(ShpData, Cursor + P * 8);
			}
		}

		TArray<FRoadShapefileRing> Rings;
		for (int32 Part = 0; Part < NumParts; ++Part)
		{
			const int32 Start = Parts[Part];
			const int32 End = (Part + 1 < NumParts) ? Parts[Part + 1] : NumPoints;
			if (End - Start < 3)
			{
				continue;
			}
			FRoadShapefileRing Ring;
			Ring.LonLat.Reserve(End - Start);
			if (AllZ.Num() == NumPoints)
			{
				Ring.HeightM.Reserve(End - Start);
			}
			for (int32 P = Start; P < End; ++P)
			{
				Ring.LonLat.Add(AllXY[P]);
				if (AllZ.Num() == NumPoints)
				{
					Ring.HeightM.Add(AllZ[P]);
				}
			}
			StripClose(Ring);
			if (Ring.LonLat.Num() >= 3)
			{
				Rings.Add(MoveTemp(Ring));
			}
		}

		if (Rings.Num() == 0)
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}

		// Emit one mask per uncontained ring (multipart outers).
		TArray<double> AbsArea;
		for (const FRoadShapefileRing& R : Rings)
		{
			AbsArea.Add(FMath::Abs(SignedArea(R.LonLat)));
		}
		TArray<int32> OuterIdx;
		TArray<int32> HoleIdx;
		for (int32 I = 0; I < Rings.Num(); ++I)
		{
			bool bHole = false;
			const FVector2D C = RingCentroid(Rings[I].LonLat);
			for (int32 J = 0; J < Rings.Num(); ++J)
			{
				if (I == J || AbsArea[J] <= AbsArea[I])
				{
					continue;
				}
				if (PointInRing(C, Rings[J].LonLat))
				{
					bHole = true;
					break;
				}
			}
			if (bHole)
			{
				HoleIdx.Add(I);
			}
			else
			{
				OuterIdx.Add(I);
			}
		}

		for (const int32 O : OuterIdx)
		{
			FRoadShapefileMask Mask;
			Mask.RecordIndex = RecordIndex;
			Mask.Outer = Rings[O];
			for (const int32 H : HoleIdx)
			{
				const FVector2D C = RingCentroid(Rings[H].LonLat);
				if (PointInRing(C, Rings[O].LonLat))
				{
					Mask.Holes.Add(Rings[H]);
				}
			}
			if (Mask.Outer.LonLat.Num() >= 3)
			{
				OutMasks.Add(MoveTemp(Mask));
			}
		}

		Offset += ContentBytes;
		++RecordIndex;
	}

	if (OutMasks.Num() == 0)
	{
		OutError = TEXT("No polygon rings found in the road mask shapefile.");
		return false;
	}

	UE_LOG(LogRoadPlacer, Display, TEXT("Read %d road-mask polygon(s) from '%s'."), OutMasks.Num(), *ShpPath);
	return true;
}

bool RoadShapefileReader::ReadElevationPoints(
	const FString& ShapefilePath,
	const FString& OptionalAltitudeFieldName,
	TArray<FRoadShapefilePoint>& OutPoints,
	FString& OutError)
{
	OutPoints.Reset();
	const FString Base = NormalizeShpPath(ShapefilePath);
	const FString ShpPath = Base + TEXT(".shp");
	const FString DbfPath = Base + TEXT(".dbf");
	const FString PrjPath = Base + TEXT(".prj");
	if (!FPaths::FileExists(ShpPath))
	{
		OutError = FString::Printf(TEXT("SHP not found: %s"), *ShpPath);
		return false;
	}
	if (!ValidateEpsg4326Prj(PrjPath, OutError))
	{
		return false;
	}

	TArray<uint8> ShpData;
	if (!FFileHelper::LoadFileToArray(ShpData, *ShpPath) || ShpData.Num() < 100)
	{
		OutError = FString::Printf(TEXT("Failed to read SHP: %s"), *ShpPath);
		return false;
	}
	if (ReadInt32BE(ShpData, 0) != 9994)
	{
		OutError = TEXT("Not a valid ESRI shapefile (bad file code).");
		return false;
	}

	const int32 ShapeType = ReadInt32LE(ShpData, 32);
	// 1 Point, 8 MultiPoint, 11 PointZ, 18 MultiPointZ, 21 PointM
	if (ShapeType != 1 && ShapeType != 8 && ShapeType != 11 && ShapeType != 18 && ShapeType != 21)
	{
		OutError = FString::Printf(
			TEXT("Elevation shapefile must be Point / PointZ (type %d)."),
			ShapeType);
		return false;
	}

	TArray<FDbfField> DbfFields;
	TArray<TArray<uint8>> DbfRecords;
	const FDbfField* ElevField = nullptr;
	if (FPaths::FileExists(DbfPath))
	{
		if (!LoadDbf(DbfPath, DbfFields, DbfRecords, OutError))
		{
			return false;
		}
		if (!OptionalAltitudeFieldName.IsEmpty())
		{
			ElevField = FindField(DbfFields, OptionalAltitudeFieldName);
			if (!ElevField)
			{
				OutError = FString::Printf(TEXT("DBF altitude field '%s' not found."), *OptionalAltitudeFieldName);
				return false;
			}
		}
	}

	auto AddPoint = [&](double X, double Y, double Z, int32 RecordIndex)
	{
		FRoadShapefilePoint Pt;
		Pt.LonDeg = X;
		Pt.LatDeg = Y;
		Pt.HeightM = Z;
		Pt.RecordIndex = RecordIndex;
		if (ElevField && RecordIndex < DbfRecords.Num())
		{
			double DbfZ = 0.0;
			if (ParseNumericField(DbfRecords[RecordIndex], *ElevField, DbfZ))
			{
				Pt.HeightM = DbfZ;
			}
		}
		OutPoints.Add(Pt);
	};

	int32 Offset = 100;
	int32 RecordIndex = 0;
	while (Offset + 8 <= ShpData.Num())
	{
		const int32 ContentBytes = ReadInt32BE(ShpData, Offset + 4) * 2;
		Offset += 8;
		if (ContentBytes <= 0 || Offset + ContentBytes > ShpData.Num())
		{
			break;
		}

		const int32 RecType = ReadInt32LE(ShpData, Offset);
		if (RecType == 0)
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}

		if (RecType == 1 || RecType == 11 || RecType == 21)
		{
			if (ContentBytes < 20)
			{
				Offset += ContentBytes;
				++RecordIndex;
				continue;
			}
			const double X = ReadDoubleLE(ShpData, Offset + 4);
			const double Y = ReadDoubleLE(ShpData, Offset + 12);
			double Z = 0.0;
			if (RecType == 11 && ContentBytes >= 28)
			{
				Z = ReadDoubleLE(ShpData, Offset + 20);
			}
			AddPoint(X, Y, Z, RecordIndex);
		}
		else if (RecType == 8 || RecType == 18)
		{
			int32 Cursor = Offset + 4 + 32;
			if (Cursor + 4 > Offset + ContentBytes)
			{
				Offset += ContentBytes;
				++RecordIndex;
				continue;
			}
			const int32 NumPoints = ReadInt32LE(ShpData, Cursor);
			Cursor += 4;
			if (NumPoints <= 0 || Cursor + NumPoints * 16 > Offset + ContentBytes)
			{
				Offset += ContentBytes;
				++RecordIndex;
				continue;
			}
			TArray<FVector2D> XYs;
			XYs.SetNum(NumPoints);
			for (int32 P = 0; P < NumPoints; ++P)
			{
				XYs[P] = FVector2D(ReadDoubleLE(ShpData, Cursor + P * 16), ReadDoubleLE(ShpData, Cursor + P * 16 + 8));
			}
			Cursor += NumPoints * 16;
			TArray<double> Zs;
			Zs.Init(0.0, NumPoints);
			if (RecType == 18 && Cursor + 16 + NumPoints * 8 <= Offset + ContentBytes)
			{
				Cursor += 16;
				for (int32 P = 0; P < NumPoints; ++P)
				{
					Zs[P] = ReadDoubleLE(ShpData, Cursor + P * 8);
				}
			}
			for (int32 P = 0; P < NumPoints; ++P)
			{
				AddPoint(XYs[P].X, XYs[P].Y, Zs[P], RecordIndex);
			}
		}

		Offset += ContentBytes;
		++RecordIndex;
	}

	if (OutPoints.Num() == 0)
	{
		OutError = TEXT("No point records found in the elevation shapefile.");
		return false;
	}

	UE_LOG(LogRoadPlacer, Display, TEXT("Read %d elevation point(s) from '%s'."), OutPoints.Num(), *ShpPath);
	return true;
}
