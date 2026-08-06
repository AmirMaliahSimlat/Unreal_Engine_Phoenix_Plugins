#include "BuildingShapefileReader.h"
#include "BuildingExtruderLog.h"

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

	/** Returns false if .prj is present and clearly not EPSG:4326 / WGS84 geographic. */
	bool ValidateEpsg4326Prj(const FString& PrjPath, FString& OutError)
	{
		if (!FPaths::FileExists(PrjPath))
		{
			UE_LOG(
				LogBuildingExtruder,
				Display,
				TEXT("No .prj found; assuming EPSG:4326 (WGS84 lon/lat degrees)."));
			return true;
		}

		FString PrjText;
		if (!FFileHelper::LoadFileToString(PrjText, *PrjPath))
		{
			UE_LOG(LogBuildingExtruder, Warning, TEXT("Could not read .prj; assuming EPSG:4326."));
			return true;
		}

		const FString Upper = PrjText.ToUpper();
		const bool bLooks4326 =
			Upper.Contains(TEXT("EPSG\",\"4326"))
			|| Upper.Contains(TEXT("EPSG\",\" 4326"))
			|| Upper.Contains(TEXT("AUTHORITY[\"EPSG\",\"4326\"]"))
			|| Upper.Contains(TEXT("WGS_1984"))
			|| Upper.Contains(TEXT("WGS 84"))
			|| Upper.Contains(TEXT("GCS_WGS_1984"))
			|| Upper.Contains(TEXT("WGS84"));

		if (bLooks4326)
		{
			UE_LOG(LogBuildingExtruder, Display, TEXT("CRS check OK: .prj looks like EPSG:4326 / WGS84."));
			return true;
		}

		// Projected systems (UTM, etc.) must not be fed in as lon/lat.
		if (Upper.Contains(TEXT("PROJCS[")) || Upper.Contains(TEXT("PROJCS \"")))
		{
			OutError = FString::Printf(
				TEXT("Shapefile .prj is a projected CRS, not EPSG:4326. Reproject to EPSG:4326 (WGS84 lon/lat) before import. File: %s"),
				*PrjPath);
			return false;
		}

		UE_LOG(
			LogBuildingExtruder,
			Warning,
			TEXT("Unrecognized .prj (expected EPSG:4326 / WGS84). Proceeding as lon/lat degrees. File: %s"),
			*PrjPath);
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

		const int32 HeaderBytes = static_cast<uint8>(Data[8]) | (static_cast<uint8>(Data[9]) << 8);
		const int32 RecordBytes = static_cast<uint8>(Data[10]) | (static_cast<uint8>(Data[11]) << 8);
		const int32 NumRecords = ReadInt32LE(Data, 4);
		if (HeaderBytes < 33 || RecordBytes <= 0 || NumRecords < 0)
		{
			OutError = TEXT("Invalid DBF header.");
			return false;
		}

		OutFields.Reset();
		int32 FieldOffset = 1; // skip delete flag
		for (int32 Desc = 32; Desc + 32 <= HeaderBytes - 1; Desc += 32)
		{
			if (Data[Desc] == 0x0D)
			{
				break;
			}

			FDbfField Field;
			char NameBuf[12];
			FMemory::Memzero(NameBuf, sizeof(NameBuf));
			FMemory::Memcpy(NameBuf, &Data[Desc], 11);
			Field.Name = ANSI_TO_TCHAR(NameBuf);
			Field.Name.TrimStartAndEndInline();
			Field.Type = static_cast<ANSICHAR>(Data[Desc + 11]);
			Field.Length = Data[Desc + 16];
			Field.Decimal = Data[Desc + 17];
			Field.OffsetInRecord = FieldOffset;
			FieldOffset += Field.Length;
			OutFields.Add(Field);
		}

		OutRecords.Reset();
		OutRecords.Reserve(NumRecords);
		int32 Cursor = HeaderBytes;
		for (int32 R = 0; R < NumRecords; ++R)
		{
			if (Cursor + RecordBytes > Data.Num())
			{
				break;
			}
			TArray<uint8> Rec;
			Rec.Append(&Data[Cursor], RecordBytes);
			OutRecords.Add(MoveTemp(Rec));
			Cursor += RecordBytes;
		}

		return true;
	}

	bool ParseNumericField(const TArray<uint8>& Record, const FDbfField& Field, double& OutValue)
	{
		if (Field.OffsetInRecord + Field.Length > Record.Num())
		{
			return false;
		}

		if (Field.Type == 'F' || Field.Type == 'N' || Field.Type == 'B')
		{
			// Binary double in some DBFs uses type 'B' / 'O' — handle text N/F first.
		}

		if (Field.Type == 'B' && Field.Length == 8)
		{
			OutValue = ReadDoubleLE(Record, Field.OffsetInRecord);
			return true;
		}

		FString Text;
		Text.Reserve(Field.Length);
		for (int32 I = 0; I < Field.Length; ++I)
		{
			const ANSICHAR C = static_cast<ANSICHAR>(Record[Field.OffsetInRecord + I]);
			if (C != 0)
			{
				Text.AppendChar(C);
			}
		}
		Text.TrimStartAndEndInline();
		if (Text.IsEmpty())
		{
			return false;
		}
		OutValue = FCString::Atod(*Text);
		return true;
	}

	const FDbfField* FindField(const TArray<FDbfField>& Fields, const FString& Name)
	{
		for (const FDbfField& Field : Fields)
		{
			if (Field.Name.Equals(Name, ESearchCase::IgnoreCase))
			{
				return &Field;
			}
		}
		return nullptr;
	}
}

bool BuildingShapefileReader::ReadPolygonBuildings(
	const FString& ShapefilePath,
	const FString& HeightFieldName,
	const FString& ElevationFieldName,
	TArray<FBuildingShapefileFeature>& OutFeatures,
	FString& OutError)
{
	OutFeatures.Reset();
	const FString BasePath = NormalizeShpPath(ShapefilePath);
	const FString ShpPath = BasePath + TEXT(".shp");
	const FString DbfPath = BasePath + TEXT(".dbf");
	const FString PrjPath = BasePath + TEXT(".prj");

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

	const int32 FileCode = ReadInt32BE(ShpData, 0);
	if (FileCode != 9994)
	{
		OutError = TEXT("Not a valid ESRI shapefile (bad file code).");
		return false;
	}

	const int32 ShapeType = ReadInt32LE(ShpData, 32);
	// 5 = Polygon, 15 = PolygonZ, 25 = PolygonM
	if (ShapeType != 5 && ShapeType != 15 && ShapeType != 25)
	{
		OutError = FString::Printf(TEXT("Unsupported shape type %d (need Polygon)."), ShapeType);
		return false;
	}

	TArray<FDbfField> DbfFields;
	TArray<TArray<uint8>> DbfRecords;
	if (!LoadDbf(DbfPath, DbfFields, DbfRecords, OutError))
	{
		return false;
	}

	const FDbfField* HeightField = FindField(DbfFields, HeightFieldName);
	const FDbfField* ElevField = ElevationFieldName.IsEmpty()
		? nullptr
		: FindField(DbfFields, ElevationFieldName);
	if (!HeightField)
	{
		OutError = FString::Printf(TEXT("DBF height field '%s' not found."), *HeightFieldName);
		return false;
	}

	int32 Offset = 100;
	int32 RecordIndex = 0;
	while (Offset + 8 <= ShpData.Num())
	{
		const int32 ContentWords = ReadInt32BE(ShpData, Offset + 4);
		const int32 ContentBytes = ContentWords * 2;
		Offset += 8;
		if (ContentBytes <= 0 || Offset + ContentBytes > ShpData.Num())
		{
			break;
		}

		const int32 RecShapeType = ReadInt32LE(ShpData, Offset);
		if (RecShapeType == 0)
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}
		if (RecShapeType != 5 && RecShapeType != 15 && RecShapeType != 25)
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}

		// Polygon layout: type(4) + bbox(32) + numParts(4) + numPoints(4) + parts[] + points[]
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

		TArray<FVector2D> AllPoints;
		AllPoints.SetNum(NumPoints);
		for (int32 P = 0; P < NumPoints; ++P)
		{
			const double X = ReadDoubleLE(ShpData, Cursor + P * 16);
			const double Y = ReadDoubleLE(ShpData, Cursor + P * 16 + 8);
			AllPoints[P] = FVector2D(X, Y);
		}

		// First ring = outer (ESRI). Ignore holes in v1.
		const int32 Start = Parts[0];
		const int32 End = (NumParts > 1) ? Parts[1] : NumPoints;
		if (Start < 0 || End > NumPoints || End - Start < 3)
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}

		FBuildingShapefileFeature Feature;
		Feature.RecordIndex = RecordIndex;
		Feature.OuterRingLonLat.Reserve(End - Start);
		for (int32 P = Start; P < End; ++P)
		{
			Feature.OuterRingLonLat.Add(AllPoints[P]);
		}
		if (Feature.OuterRingLonLat.Num() >= 2
			&& Feature.OuterRingLonLat[0].Equals(Feature.OuterRingLonLat.Last(), 1.0e-12))
		{
			Feature.OuterRingLonLat.Pop();
		}

		if (RecordIndex < DbfRecords.Num())
		{
			const TArray<uint8>& Rec = DbfRecords[RecordIndex];
			if (Rec.Num() > 0 && Rec[0] != '*')
			{
				ParseNumericField(Rec, *HeightField, Feature.HeightM);
				if (ElevField)
				{
					ParseNumericField(Rec, *ElevField, Feature.ElevationM);
				}
			}
		}

		// Keep footprints even if RELATIVE_F / height is anomalous; caller replaces those.
		if (Feature.OuterRingLonLat.Num() >= 3)
		{
			OutFeatures.Add(MoveTemp(Feature));
		}
		else
		{
			UE_LOG(
				LogBuildingExtruder,
				Verbose,
				TEXT("Skipping record %d (verts=%d)."),
				RecordIndex,
				Feature.OuterRingLonLat.Num());
		}

		Offset += ContentBytes;
		++RecordIndex;
	}

	if (OutFeatures.Num() == 0)
	{
		OutError = TEXT("No valid polygon buildings found in shapefile.");
		return false;
	}

	UE_LOG(LogBuildingExtruder, Display, TEXT("Read %d building footprints from %s (EPSG:4326 lon/lat)"), OutFeatures.Num(), *ShpPath);
	return true;
}
