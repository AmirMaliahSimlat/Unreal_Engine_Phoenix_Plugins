#include "TreeShapefileReader.h"
#include "TreePlacerLog.h"

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

	bool ValidateEpsg4326Prj(const FString& PrjPath, FString& OutError)
	{
		if (!FPaths::FileExists(PrjPath))
		{
			UE_LOG(LogTreePlacer, Display, TEXT("No .prj found; assuming EPSG:4326 (WGS84 lon/lat degrees)."));
			return true;
		}

		FString PrjText;
		if (!FFileHelper::LoadFileToString(PrjText, *PrjPath))
		{
			UE_LOG(LogTreePlacer, Warning, TEXT("Could not read .prj; assuming EPSG:4326."));
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
			UE_LOG(LogTreePlacer, Display, TEXT("CRS check OK: .prj looks like EPSG:4326 / WGS84."));
			return true;
		}

		if (Upper.Contains(TEXT("PROJCS[")) || Upper.Contains(TEXT("PROJCS \"")))
		{
			OutError = FString::Printf(
				TEXT("Shapefile .prj is a projected CRS, not EPSG:4326. Reproject to EPSG:4326 (WGS84 lon/lat) before import. File: %s"),
				*PrjPath);
			return false;
		}

		UE_LOG(
			LogTreePlacer,
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
		int32 FieldOffset = 1;
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

	bool ParseByteField(const TArray<uint8>& Record, const FDbfField& Field, uint8& OutByte)
	{
		double Value = 0.0;
		if (!ParseNumericField(Record, Field, Value))
		{
			return false;
		}
		OutByte = static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(Value), 0, 255));
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

bool TreeShapefileReader::ReadPoints(
	const FString& ShapefilePath,
	const FString& AltitudeFieldName,
	TArray<FTreeShapefilePoint>& OutPoints,
	FString& OutError,
	const FString& RedFieldName,
	const FString& GreenFieldName,
	const FString& BlueFieldName)
{
	OutPoints.Reset();
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
	// 1 = Point, 11 = PointZ, 21 = PointM
	if (ShapeType != 1 && ShapeType != 11 && ShapeType != 21)
	{
		OutError = FString::Printf(
			TEXT("Unsupported shape type %d (need Point / PointZ / PointM)."),
			ShapeType);
		return false;
	}

	TArray<FDbfField> DbfFields;
	TArray<TArray<uint8>> DbfRecords;
	if (!LoadDbf(DbfPath, DbfFields, DbfRecords, OutError))
	{
		return false;
	}

	if (AltitudeFieldName.IsEmpty())
	{
		OutError = TEXT("DBF altitude field name is empty.");
		return false;
	}

	const FDbfField* ElevField = FindField(DbfFields, AltitudeFieldName);
	if (!ElevField)
	{
		OutError = FString::Printf(TEXT("DBF altitude field '%s' not found."), *AltitudeFieldName);
		return false;
	}

	const bool bWantRgb = !RedFieldName.IsEmpty() && !GreenFieldName.IsEmpty() && !BlueFieldName.IsEmpty();
	const FDbfField* RedField = nullptr;
	const FDbfField* GreenField = nullptr;
	const FDbfField* BlueField = nullptr;
	if (bWantRgb)
	{
		RedField = FindField(DbfFields, RedFieldName);
		GreenField = FindField(DbfFields, GreenFieldName);
		BlueField = FindField(DbfFields, BlueFieldName);
		if (!RedField || !GreenField || !BlueField)
		{
			OutError = FString::Printf(
				TEXT("DBF RGB fields not found (need '%s', '%s', '%s')."),
				*RedFieldName,
				*GreenFieldName,
				*BlueFieldName);
			return false;
		}
	}

	int32 Offset = 100;
	int32 RecordIndex = 0;
	int32 NullAltitudeCount = 0;
	int32 NullRgbCount = 0;
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
		if (RecShapeType != 1 && RecShapeType != 11 && RecShapeType != 21)
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}

		if (ContentBytes < 20)
		{
			Offset += ContentBytes;
			++RecordIndex;
			continue;
		}

		const double X = ReadDoubleLE(ShpData, Offset + 4);
		const double Y = ReadDoubleLE(ShpData, Offset + 12);

		FTreeShapefilePoint Point;
		Point.RecordIndex = RecordIndex;
		Point.LonDeg = X;
		Point.LatDeg = Y;
		Point.AltitudeM = 0.0;

		if (RecordIndex < DbfRecords.Num())
		{
			const TArray<uint8>& Rec = DbfRecords[RecordIndex];
			if (Rec.Num() > 0 && Rec[0] != '*')
			{
				if (!ParseNumericField(Rec, *ElevField, Point.AltitudeM))
				{
					++NullAltitudeCount;
					Point.AltitudeM = 0.0;
				}
				if (bWantRgb)
				{
					uint8 Rv = 0;
					uint8 Gv = 0;
					uint8 Bv = 0;
					if (ParseByteField(Rec, *RedField, Rv)
						&& ParseByteField(Rec, *GreenField, Gv)
						&& ParseByteField(Rec, *BlueField, Bv))
					{
						Point.bHasRgb = true;
						Point.R = Rv;
						Point.G = Gv;
						Point.B = Bv;
					}
					else
					{
						++NullRgbCount;
					}
				}
			}
		}

		OutPoints.Add(Point);
		Offset += ContentBytes;
		++RecordIndex;
	}

	if (OutPoints.Num() == 0)
	{
		OutError = TEXT("No point records found in shapefile.");
		return false;
	}

	UE_LOG(
		LogTreePlacer,
		Display,
		TEXT("Read %d tree points from '%s' (null/empty altitude -> 0: %d, missing RGB: %d)"),
		OutPoints.Num(),
		*ShpPath,
		NullAltitudeCount,
		NullRgbCount);
	return true;
}
