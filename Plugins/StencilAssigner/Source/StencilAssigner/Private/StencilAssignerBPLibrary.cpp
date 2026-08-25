#include "StencilAssignerBPLibrary.h"

#include "StencilAssignerLog.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Editor.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "FoliageType.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "InstancedFoliage.h"
#include "InstancedFoliageActor.h"
#include "Internationalization/Internationalization.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopedSlowTask.h"
#include "Runtime/Launch/Resources/Version.h"

namespace
{
	FString SanitizeFilePath(const FString& InPath)
	{
		FString Path = InPath.TrimStartAndEnd();
		while ((Path.StartsWith(TEXT("\"")) && Path.EndsWith(TEXT("\"")) && Path.Len() >= 2)
			|| (Path.StartsWith(TEXT("'")) && Path.EndsWith(TEXT("'")) && Path.Len() >= 2))
		{
			Path = Path.Mid(1, Path.Len() - 2).TrimStartAndEnd();
		}
		return Path;
	}

	UWorld* ResolveEditorWorld(UObject* WorldContextObject)
	{
		UWorld* World = nullptr;
		if (WorldContextObject)
		{
			World = GEngine ? GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::ReturnNull) : nullptr;
		}
		if (!World && GEditor)
		{
			World = GEditor->GetEditorWorldContext().World();
		}
		return World;
	}

	FString ActorLabel(const AActor& Actor)
	{
#if WITH_EDITOR
		const FString Label = Actor.GetActorLabel();
		if (!Label.IsEmpty())
		{
			return Label;
		}
#endif
		return Actor.GetName();
	}

	FString JoinComponentTags(const UActorComponent& Comp)
	{
		if (Comp.ComponentTags.Num() == 0)
		{
			return TEXT("(none)");
		}
		FString Out;
		for (int32 I = 0; I < Comp.ComponentTags.Num(); ++I)
		{
			if (I > 0)
			{
				Out += TEXT(", ");
			}
			Out += Comp.ComponentTags[I].ToString();
		}
		return Out;
	}

	struct FParsedMapping
	{
		FName Tag;
		int32 StencilValue = 0;
	};

	bool ParseMappings(const FString& Text, TArray<FParsedMapping>& OutMappings, FString& OutError)
	{
		OutMappings.Reset();
		FString Normalized = Text;
		Normalized.ReplaceInline(TEXT("\r\n"), TEXT("\n"));
		Normalized.ReplaceInline(TEXT("\r"), TEXT("\n"));
		Normalized.ReplaceInline(TEXT(";"), TEXT(","));
		Normalized.ReplaceInline(TEXT("\n"), TEXT(","));

		TArray<FString> Parts;
		Normalized.ParseIntoArray(Parts, TEXT(","), /*bCullEmpty*/ true);
		for (FString& Part : Parts)
		{
			Part.TrimStartAndEndInline();
			if (Part.IsEmpty())
			{
				continue;
			}

			FString TagText;
			FString ValueText;
			if (!Part.Split(TEXT("="), &TagText, &ValueText)
				&& !Part.Split(TEXT(":"), &TagText, &ValueText))
			{
				OutError = FString::Printf(
					TEXT("Mapping '%s' is not Tag=Value. Use e.g. Wall=1,Roof=2,Tree=3,RoofObject=4"),
					*Part);
				return false;
			}
			TagText.TrimStartAndEndInline();
			ValueText.TrimStartAndEndInline();
			if (TagText.IsEmpty())
			{
				OutError = TEXT("Mapping has an empty tag. Use e.g. Wall=1,Roof=2,Tree=3,RoofObject=4");
				return false;
			}

			int32 Value = INDEX_NONE;
			if (!LexTryParseString(Value, *ValueText) || Value < 0 || Value > 255)
			{
				OutError = FString::Printf(
					TEXT("Mapping '%s' stencil value must be an integer 0–255."),
					*Part);
				return false;
			}

			FParsedMapping Entry;
			Entry.Tag = FName(*TagText);
			Entry.StencilValue = Value;
			OutMappings.Add(Entry);
		}

		if (OutMappings.Num() == 0)
		{
			OutError = TEXT("Mappings is empty. Paste e.g. Wall=1,Roof=2,Tree=3,RoofObject=4");
			return false;
		}
		return true;
	}

	bool ComponentOrAncestorsHaveTag(const USceneComponent& Comp, FName Tag)
	{
		const USceneComponent* Walk = &Comp;
		while (Walk)
		{
			if (Walk->ComponentTags.Contains(Tag))
			{
				return true;
			}
			Walk = Walk->GetAttachParent();
		}
		return false;
	}

	bool TryMatchStencil(const UPrimitiveComponent& Comp, const TArray<FParsedMapping>& Mappings, int32& OutValue)
	{
		for (const FParsedMapping& Mapping : Mappings)
		{
			if (Mapping.Tag.IsNone())
			{
				continue;
			}
			if (ComponentOrAncestorsHaveTag(Comp, Mapping.Tag))
			{
				OutValue = Mapping.StencilValue;
				return true;
			}
		}
		return false;
	}

	void WarnIfNanite(const UPrimitiveComponent& Comp)
	{
#if ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION <= 1
		const UStaticMesh* Mesh = nullptr;
		if (const UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(&Comp))
		{
			Mesh = SMC->GetStaticMesh();
		}
		if (Mesh && Mesh->NaniteSettings.bEnabled)
		{
			UE_LOG(
				LogStencilAssigner,
				Warning,
				TEXT("Nanite is enabled on mesh '%s' (component '%s'). UE 5.1 Custom Stencil often skips Nanite meshes."),
				*Mesh->GetName(),
				*Comp.GetName());
		}
#else
		(void)Comp;
#endif
	}

	void ApplyStencilToPrimitive(UPrimitiveComponent& Comp, int32 StencilValue)
	{
		AActor* Owner = Comp.GetOwner();
		if (Owner)
		{
			Owner->Modify();
		}
		Comp.Modify();
		Comp.SetRenderCustomDepth(true);
		Comp.SetCustomDepthStencilValue(StencilValue);
		Comp.SetCustomDepthStencilWriteMask(ERendererStencilMask::ERSM_Default);
		Comp.bHiddenInSceneCapture = false;
		Comp.MarkRenderStateDirty();
		WarnIfNanite(Comp);

		AInstancedFoliageActor* IFA = Owner ? Cast<AInstancedFoliageActor>(Owner) : nullptr;
		if (!IFA)
		{
			return;
		}

		// Foliage copies Type -> HISM on refresh; component-only stencil gets wiped.
		IFA->ForEachFoliageInfo([&](UFoliageType* Type, FFoliageInfo& Info)
		{
			if (!Type || Info.GetComponent() != &Comp)
			{
				return true;
			}
			Type->Modify();
			Type->bRenderCustomDepth = true;
			Type->CustomDepthStencilValue = StencilValue;
			Type->CustomDepthStencilWriteMask = ERendererStencilMask::ERSM_Default;
			return true;
		});
	}

	FString FormatUnmatchedLine(const AActor& Actor, const UPrimitiveComponent& Comp)
	{
		return FString::Printf(
			TEXT("%s\t%s\t%s\t%s\t%s"),
			*ActorLabel(Actor),
			*Actor.GetClass()->GetName(),
			*Comp.GetName(),
			*Comp.GetClass()->GetName(),
			*JoinComponentTags(Comp));
	}
}

FStencilAssignResult UStencilAssignerBPLibrary::ApplyCustomStencilByTags(
	UObject* WorldContextObject,
	const FString& Mappings,
	const FString& UnmatchedReportPath)
{
	FStencilAssignResult Result;
	const double StartTime = FPlatformTime::Seconds();
	const FString ReportPath = SanitizeFilePath(UnmatchedReportPath);

	UE_LOG(LogStencilAssigner, Display, TEXT("========== Stencil Assign START =========="));

	TArray<FParsedMapping> ParsedMappings;
	FString ParseError;
	if (!ParseMappings(Mappings, ParsedMappings, ParseError))
	{
		Result.Message = ParseError;
		UE_LOG(LogStencilAssigner, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	if (ReportPath.IsEmpty())
	{
		Result.Message = TEXT("UnmatchedReportPath is empty. Provide a .txt path for unmatched components.");
		UE_LOG(LogStencilAssigner, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	UWorld* World = ResolveEditorWorld(WorldContextObject);
	if (!World)
	{
		Result.Message = TEXT("Could not resolve an editor world. Open a map first.");
		UE_LOG(LogStencilAssigner, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	int32 MappingCount = ParsedMappings.Num();
	for (const FParsedMapping& Mapping : ParsedMappings)
	{
		UE_LOG(
			LogStencilAssigner,
			Display,
			TEXT("Mapping: tag='%s' stencil=%d"),
			*Mapping.Tag.ToString(),
			Mapping.StencilValue);
	}

	TArray<AActor*> Actors;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (IsValid(Actor))
		{
			Actors.Add(Actor);
		}
	}

	FScopedSlowTask SlowTask(
		static_cast<float>(FMath::Max(Actors.Num(), 1)),
		NSLOCTEXT("StencilAssigner", "AssignProgress", "Assigning custom stencil by component tags..."));
	SlowTask.MakeDialog(true);

	int32 Matched = 0;
	TArray<FString> UnmatchedLines;
	UnmatchedLines.Add(TEXT("ActorLabel\tActorClass\tComponent\tComponentClass\tComponentTags"));

	for (AActor* Actor : Actors)
	{
		SlowTask.EnterProgressFrame(1.0f, FText::FromString(ActorLabel(*Actor)));
		if (SlowTask.ShouldCancel())
		{
			Result.ComponentsMatched = Matched;
			Result.ComponentsUnmatched = UnmatchedLines.Num() - 1;
			Result.UnmatchedReportPath = ReportPath;
			Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
			Result.Message = FString::Printf(
				TEXT("Cancelled. Matched=%d unmatched=%d. Elapsed: %.2fs."),
				Matched,
				Result.ComponentsUnmatched,
				Result.ElapsedSeconds);
			UE_LOG(LogStencilAssigner, Warning, TEXT("%s"), *Result.Message);
			return Result;
		}

		TArray<UPrimitiveComponent*> Primitives;
		Actor->GetComponents<UPrimitiveComponent>(Primitives, /*bIncludeFromChildActors*/ true);
		for (UPrimitiveComponent* Comp : Primitives)
		{
			if (!Comp)
			{
				continue;
			}

			int32 StencilValue = 0;
			if (TryMatchStencil(*Comp, ParsedMappings, StencilValue))
			{
				ApplyStencilToPrimitive(*Comp, StencilValue);
				++Matched;
			}
			else
			{
				const FString Line = FormatUnmatchedLine(*Actor, *Comp);
				UnmatchedLines.Add(Line);
				UE_LOG(
					LogStencilAssigner,
					Display,
					TEXT("Unmatched: actor='%s' (%s) component='%s' (%s) tags=[%s]"),
					*ActorLabel(*Actor),
					*Actor->GetClass()->GetName(),
					*Comp->GetName(),
					*Comp->GetClass()->GetName(),
					*JoinComponentTags(*Comp));
			}
		}
	}

	const int32 UnmatchedCount = UnmatchedLines.Num() - 1;
	FString ReportBody = FString::Printf(
		TEXT("Unmatched custom-stencil components: %d\nMatched: %d\nMappings: %d\n\n"),
		UnmatchedCount,
		Matched,
		MappingCount);
	for (const FString& Line : UnmatchedLines)
	{
		ReportBody += Line;
		ReportBody += TEXT("\n");
	}

	FString WritePath = ReportPath;
	if (FPaths::GetExtension(WritePath).IsEmpty())
	{
		WritePath += TEXT(".txt");
	}
	const FString Directory = FPaths::GetPath(WritePath);
	if (!Directory.IsEmpty())
	{
		IFileManager::Get().MakeDirectory(*Directory, /*Tree*/ true);
	}

	if (!FFileHelper::SaveStringToFile(ReportBody, *WritePath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		Result.ComponentsMatched = Matched;
		Result.ComponentsUnmatched = UnmatchedCount;
		Result.UnmatchedReportPath = WritePath;
		Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
		Result.Message = FString::Printf(
			TEXT("Assigned stencil to %d components (%d unmatched) but failed to write report '%s'."),
			Matched,
			UnmatchedCount,
			*WritePath);
		UE_LOG(LogStencilAssigner, Error, TEXT("%s"), *Result.Message);
		return Result;
	}

	World->MarkPackageDirty();

	Result.bSuccess = true;
	Result.ComponentsMatched = Matched;
	Result.ComponentsUnmatched = UnmatchedCount;
	Result.UnmatchedReportPath = WritePath;
	Result.ElapsedSeconds = FPlatformTime::Seconds() - StartTime;
	Result.Message = FString::Printf(
		TEXT("Assigned stencil to %d components, %d unmatched. Report: %s Elapsed: %.2fs."),
		Matched,
		UnmatchedCount,
		*WritePath,
		Result.ElapsedSeconds);
	UE_LOG(LogStencilAssigner, Display, TEXT("%s"), *Result.Message);
	UE_LOG(LogStencilAssigner, Display, TEXT("========== Stencil Assign END =========="));
	return Result;
}
