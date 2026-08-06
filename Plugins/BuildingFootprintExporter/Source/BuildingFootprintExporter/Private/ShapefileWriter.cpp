#include "ShapefileWriter.h"
#include "FootprintGeometryUtils.h"
#include "BuildingFootprintExporterLog.h"

#include "Algo/Reverse.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
	void WriteInt32LE(TArray<uint8>& Buffer, int32 Value)
	{
		Buffer.Add(static_cast<uint8>(Value & 0xFF));
		Buffer.Add(static_cast<uint8>((Value >> 8) & 0xFF));
		Buffer.Add(static_cast<uint8>((Value >> 16) & 0xFF));
		Buffer.Add(static_cast<uint8>((Value >> 24) & 0xFF));
	}

	void WriteInt32BE(TArray<uint8>& Buffer, int32 Value)
	{
		Buffer.Add(static_cast<uint8>((Value >> 24) & 0xFF));
		Buffer.Add(static_cast<uint8>((Value >> 16) & 0xFF));
		Buffer.Add(static_cast<uint8>((Value >> 8) & 0xFF));
		Buffer.Add(static_cast<uint8>(Value & 0xFF));
	}

	void WriteDoubleLE(TArray<uint8>& Buffer, double Value)
	{
		uint64 Bits = 0;
		FMemory::Memcpy(&Bits, &Value, sizeof(double));
		for (int32 I = 0; I < 8; ++I)
		{
			Buffer.Add(static_cast<uint8>((Bits >> (8 * I)) & 0xFF));
		}
	}

	void WriteZeros(TArray<uint8>& Buffer, int32 Count)
	{
		for (int32 I = 0; I < Count; ++I)
		{
			Buffer.Add(0);
		}
	}

	TArray<FVector2D> EnsureClosedRing(const TArray<FVector2D>& Ring)
	{
		TArray<FVector2D> Out = Ring;
		if (Out.Num() == 0)
		{
			return Out;
		}
		if (!Out[0].Equals(Out.Last(), 1.0e-12))
		{
			const FVector2D First = Out[0];
			Out.Add(First);
		}
		return Out;
	}

	double ShapefileSignedArea2(const TArray<FVector2D>& Ring)
	{
		double Area2 = 0.0;
		for (int32 I = 0; I + 1 < Ring.Num(); ++I)
		{
			Area2 += Ring[I].X * Ring[I + 1].Y - Ring[I + 1].X * Ring[I].Y;
		}
		return Area2;
	}

	/** ESRI outer rings should be clockwise. */
	TArray<FVector2D> EnsureClockwise(TArray<FVector2D> Ring)
	{
		if (Ring.Num() < 3)
		{
			return Ring;
		}
		if (ShapefileSignedArea2(Ring) > 0.0)
		{
			Algo::Reverse(Ring);
		}
		return Ring;
	}

	/** ESRI hole rings should be counter-clockwise. */
	TArray<FVector2D> EnsureCounterClockwise(TArray<FVector2D> Ring)
	{
		if (Ring.Num() < 3)
		{
			return Ring;
		}
		if (ShapefileSignedArea2(Ring) < 0.0)
		{
			Algo::Reverse(Ring);
		}
		return Ring;
	}

	void ExpandBounds(double& MinX, double& MinY, double& MaxX, double& MaxY, const TArray<FVector2D>& Ring)
	{
		for (const FVector2D& P : Ring)
		{
			MinX = FMath::Min(MinX, P.X);
			MinY = FMath::Min(MinY, P.Y);
			MaxX = FMath::Max(MaxX, P.X);
			MaxY = FMath::Max(MaxY, P.Y);
		}
	}

	bool WriteAllBytes(const FString& Path, const TArray<uint8>& Bytes, FString& OutError)
	{
		if (!FFileHelper::SaveArrayToFile(Bytes, *Path))
		{
			OutError = FString::Printf(TEXT("Failed to write file: %s"), *Path);
			return false;
		}
		return true;
	}

	void WriteFieldDescriptor(TArray<uint8>& Dbf, const char* Name, char Type, uint8 Length, uint8 Decimals)
	{
		char NameBuf[11];
		FMemory::Memzero(NameBuf, 11);
		const int32 NameLen = FCStringAnsi::Strlen(Name);
		FMemory::Memcpy(NameBuf, Name, FMath::Min(NameLen, 10));
		for (int32 I = 0; I < 11; ++I)
		{
			Dbf.Add(static_cast<uint8>(NameBuf[I]));
		}
		Dbf.Add(static_cast<uint8>(Type));
		WriteZeros(Dbf, 4);
		Dbf.Add(Length);
		Dbf.Add(Decimals);
		WriteZeros(Dbf, 14);
	}

	void WriteFixedString(TArray<uint8>& Buffer, const FString& Text, int32 Width)
	{
		FTCHARToUTF8 Converter(*Text);
		const char* Src = Converter.Get();
		const int32 SrcLen = Converter.Length();
		for (int32 I = 0; I < Width; ++I)
		{
			Buffer.Add(I < SrcLen ? static_cast<uint8>(Src[I]) : static_cast<uint8>(' '));
		}
	}

	void WriteFixedNumber(TArray<uint8>& Buffer, const FString& Text, int32 Width)
	{
		FString Padded = Text;
		if (Padded.Len() > Width)
		{
			Padded = Padded.Left(Width);
		}
		while (Padded.Len() < Width)
		{
			Padded = TEXT(" ") + Padded;
		}
		for (int32 I = 0; I < Width; ++I)
		{
			Buffer.Add(static_cast<uint8>(Padded[I]));
		}
	}

	void PatchShapefileHeader(
		TArray<uint8>& FileBytes,
		int32 FileLengthWords,
		double MinX,
		double MinY,
		double MaxX,
		double MaxY)
	{
		TArray<uint8> Header;
		Header.Reserve(100);
		WriteInt32BE(Header, 9994);
		WriteZeros(Header, 20);
		WriteInt32BE(Header, FileLengthWords);
		WriteInt32LE(Header, 1000);
		WriteInt32LE(Header, 5); // Polygon
		WriteDoubleLE(Header, MinX);
		WriteDoubleLE(Header, MinY);
		WriteDoubleLE(Header, MaxX);
		WriteDoubleLE(Header, MaxY);
		WriteDoubleLE(Header, 0.0);
		WriteDoubleLE(Header, 0.0);
		WriteDoubleLE(Header, 0.0);
		WriteDoubleLE(Header, 0.0);
		check(Header.Num() == 100);
		FMemory::Memcpy(FileBytes.GetData(), Header.GetData(), 100);
	}
}

bool ShapefileWriter::WritePolygons(
	const FString& OutputPathWithoutExtension,
	const TArray<FBuildingFootprintPolygon>& Footprints,
	const FString& MapName,
	FString& OutError)
{
	if (OutputPathWithoutExtension.IsEmpty())
	{
		OutError = TEXT("Output path is empty.");
		UE_LOG(LogBuildingFootprintExporter, Error, TEXT("Shapefile write: %s"), *OutError);
		return false;
	}

	if (Footprints.Num() == 0)
	{
		OutError = TEXT("No footprints to write.");
		UE_LOG(LogBuildingFootprintExporter, Error, TEXT("Shapefile write: %s"), *OutError);
		return false;
	}

	UE_LOG(
		LogBuildingFootprintExporter,
		Display,
		TEXT("Shapefile write begin | footprints=%d map='%s' path='%s'"),
		Footprints.Num(),
		*MapName,
		*OutputPathWithoutExtension);

	const FString Dir = FPaths::GetPath(OutputPathWithoutExtension);
	if (!Dir.IsEmpty())
	{
		IFileManager::Get().MakeDirectory(*Dir, true);
	}

	struct FValidRecord
	{
		TArray<TArray<FVector2D>> Rings; // [0]=outer CW, others=holes CCW
		FString ActorLabel;
		double AreaM2 = 0.0;
		double HeightM = 0.0;
	};

	TArray<FValidRecord> Records;
	Records.Reserve(Footprints.Num());

	double MinX = TNumericLimits<double>::Max();
	double MinY = TNumericLimits<double>::Max();
	double MaxX = TNumericLimits<double>::Lowest();
	double MaxY = TNumericLimits<double>::Lowest();

	for (const FBuildingFootprintPolygon& Footprint : Footprints)
	{
		if (Footprint.LonLatRing.Num() < 3)
		{
			continue;
		}

		TArray<FVector2D> Outer = EnsureClockwise(EnsureClosedRing(Footprint.LonLatRing));
		if (Outer.Num() < 4)
		{
			continue;
		}

		FValidRecord Record;
		Record.Rings.Add(MoveTemp(Outer));
		ExpandBounds(MinX, MinY, MaxX, MaxY, Record.Rings[0]);

		for (const TArray<FVector2D>& HoleIn : Footprint.HoleLonLatRings)
		{
			if (HoleIn.Num() < 3)
			{
				continue;
			}
			TArray<FVector2D> Hole = EnsureCounterClockwise(EnsureClosedRing(HoleIn));
			if (Hole.Num() < 4)
			{
				continue;
			}
			ExpandBounds(MinX, MinY, MaxX, MaxY, Hole);
			Record.Rings.Add(MoveTemp(Hole));
		}

		Record.ActorLabel = Footprint.SourceActorLabel;
		Record.AreaM2 = Footprint.AreaM2;
		Record.HeightM = Footprint.HeightM;
		Records.Add(MoveTemp(Record));
	}

	if (Records.Num() == 0)
	{
		OutError = TEXT("All footprints were invalid (need >= 3 unique vertices).");
		return false;
	}

	const FString ShpPath = OutputPathWithoutExtension + TEXT(".shp");
	const FString ShxPath = OutputPathWithoutExtension + TEXT(".shx");
	const FString DbfPath = OutputPathWithoutExtension + TEXT(".dbf");
	const FString PrjPath = OutputPathWithoutExtension + TEXT(".prj");

	TArray<uint8> Shp;
	TArray<uint8> Shx;
	Shp.Reserve(1024 * Records.Num());
	Shx.Reserve(100 + 8 * Records.Num());
	WriteZeros(Shp, 100);
	WriteZeros(Shx, 100);

	int32 RecordNumber = 1;
	for (const FValidRecord& Record : Records)
	{
		int32 NumPoints = 0;
		for (const TArray<FVector2D>& Ring : Record.Rings)
		{
			NumPoints += Ring.Num();
		}
		const int32 NumParts = Record.Rings.Num();
		const int32 ContentBytes = 4 + 32 + 4 + 4 + (4 * NumParts) + (16 * NumPoints);
		const int32 ContentWords = ContentBytes / 2;
		const int32 RecordOffsetWords = Shp.Num() / 2;

		WriteInt32BE(Shp, RecordNumber);
		WriteInt32BE(Shp, ContentWords);
		WriteInt32BE(Shx, RecordOffsetWords);
		WriteInt32BE(Shx, ContentWords);

		double RMinX = Record.Rings[0][0].X;
		double RMinY = Record.Rings[0][0].Y;
		double RMaxX = RMinX;
		double RMaxY = RMinY;
		for (const TArray<FVector2D>& Ring : Record.Rings)
		{
			for (const FVector2D& P : Ring)
			{
				RMinX = FMath::Min(RMinX, P.X);
				RMinY = FMath::Min(RMinY, P.Y);
				RMaxX = FMath::Max(RMaxX, P.X);
				RMaxY = FMath::Max(RMaxY, P.Y);
			}
		}

		WriteInt32LE(Shp, 5);
		WriteDoubleLE(Shp, RMinX);
		WriteDoubleLE(Shp, RMinY);
		WriteDoubleLE(Shp, RMaxX);
		WriteDoubleLE(Shp, RMaxY);
		WriteInt32LE(Shp, NumParts);
		WriteInt32LE(Shp, NumPoints);

		int32 PartOffset = 0;
		for (const TArray<FVector2D>& Ring : Record.Rings)
		{
			WriteInt32LE(Shp, PartOffset);
			PartOffset += Ring.Num();
		}
		for (const TArray<FVector2D>& Ring : Record.Rings)
		{
			for (const FVector2D& P : Ring)
			{
				WriteDoubleLE(Shp, P.X);
				WriteDoubleLE(Shp, P.Y);
			}
		}

		++RecordNumber;
	}

	PatchShapefileHeader(Shp, Shp.Num() / 2, MinX, MinY, MaxX, MaxY);
	PatchShapefileHeader(Shx, Shx.Num() / 2, MinX, MinY, MaxX, MaxY);

	if (!WriteAllBytes(ShpPath, Shp, OutError) || !WriteAllBytes(ShxPath, Shx, OutError))
	{
		return false;
	}

	const int32 NumRecords = Records.Num();
	const uint8 NumFields = 5;
	const int16 HeaderSize = static_cast<int16>(32 + NumFields * 32 + 1);
	const int16 RecordSize = static_cast<int16>(1 + 10 + 64 + 64 + 18 + 18);

	TArray<uint8> Dbf;
	Dbf.Reserve(HeaderSize + RecordSize * NumRecords);
	Dbf.Add(0x03);
	Dbf.Add(25);
	Dbf.Add(1);
	Dbf.Add(1);
	WriteInt32LE(Dbf, NumRecords);
	Dbf.Add(static_cast<uint8>(HeaderSize & 0xFF));
	Dbf.Add(static_cast<uint8>((HeaderSize >> 8) & 0xFF));
	Dbf.Add(static_cast<uint8>(RecordSize & 0xFF));
	Dbf.Add(static_cast<uint8>((RecordSize >> 8) & 0xFF));
	WriteZeros(Dbf, 20);

	WriteFieldDescriptor(Dbf, "ID", 'N', 10, 0);
	WriteFieldDescriptor(Dbf, "MAP", 'C', 64, 0);
	WriteFieldDescriptor(Dbf, "ACTOR", 'C', 64, 0);
	WriteFieldDescriptor(Dbf, "AREA_M2", 'N', 18, 3);
	WriteFieldDescriptor(Dbf, "HEIGHT_M", 'N', 18, 3);
	Dbf.Add(0x0D);

	for (int32 I = 0; I < Records.Num(); ++I)
	{
		Dbf.Add(0x20);
		WriteFixedNumber(Dbf, FString::FromInt(I + 1), 10);
		WriteFixedString(Dbf, MapName, 64);
		WriteFixedString(Dbf, Records[I].ActorLabel, 64);
		WriteFixedNumber(Dbf, FString::Printf(TEXT("%.3f"), Records[I].AreaM2), 18);
		WriteFixedNumber(Dbf, FString::Printf(TEXT("%.3f"), Records[I].HeightM), 18);
	}

	if (!WriteAllBytes(DbfPath, Dbf, OutError))
	{
		return false;
	}

	const FString Wgs84Wkt =
		TEXT("GEOGCS[\"GCS_WGS_1984\",DATUM[\"D_WGS_1984\",SPHEROID[\"WGS_1984\",6378137.0,298.257223563]],")
		TEXT("PRIMEM[\"Greenwich\",0.0],UNIT[\"Degree\",0.0174532925199433]]");
	if (!FFileHelper::SaveStringToFile(Wgs84Wkt, *PrjPath))
	{
		OutError = FString::Printf(TEXT("Failed to write file: %s"), *PrjPath);
		UE_LOG(LogBuildingFootprintExporter, Error, TEXT("Shapefile write: %s"), *OutError);
		return false;
	}

	UE_LOG(LogBuildingFootprintExporter, Display, TEXT("Shapefile write OK | %s (.shp/.shx/.dbf/.prj)"), *OutputPathWithoutExtension);
	return true;
}
