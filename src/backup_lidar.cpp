#include "Carla.h"
#include "Carla/Sensor/THILidar.h"
#include "Actor/ActorBlueprintFunctionLibrary.h"
#include "ConstructorHelpers.h"
#include "DrawDebugHelpers.h"
#include "PhysicsInterfaceUtilsCore.h"
#include "Engine/Engine.h"
#include "SceneTypes.h"
#include "PhysicalMaterials/RoadviewPhysicalMaterial.h"
#include "Materials/MaterialParameterCollection.h"
#include "Async/ParallelFor.h"
#include <random>
#include <math.h>
#include <cmath>
#include "Eigen/Dense"
#include "Math/UnrealMathUtility.h" 
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/GameplayStatics.h"

namespace crp = carla::rpc;
std::default_random_engine generator_thi;

FActorDefinition ATHILidar::GetSensorDefinition()
{
	return UActorBlueprintFunctionLibrary::MakeTHILidarDefinition(TEXT("thi_lidar"));
}

ATHILidar::ATHILidar(const FObjectInitializer &ObjectInitializer) : Super(ObjectInitializer)
{
	PrimaryActorTick.bCanEverTick = true;
	
	LidarOrigin = CreateDefaultSubobject<USceneComponent>(TEXT("Lidar Origin"));
	LidarOrigin->SetupAttachment(RootComponent);

	Weather = (AWeather*)UGameplayStatics::GetActorOfClass(GetWorld(), AWeather::StaticClass());
	
	//static ConstructorHelpers::FObjectFinder<UTextureRenderTarget2D> RoadRenderTargetObj(TEXT("/Game/Carla/Static/Road/CARISSMA/Snow/RVT_Snow.RVT_Snow"));
	//RoadRenderTarget = RoadRenderTargetObj.Object;
}

void ATHILidar::Set(const FActorDescription& ActorDescription)
{
	Super::Set(ActorDescription);
	FTHILidarDescription LidarDescription;
	UActorBlueprintFunctionLibrary::SetTHILidar(ActorDescription, LidarDescription);
	Set(LidarDescription);
}

void ATHILidar::Set(const FTHILidarDescription& LidarDescription)
{
	Description = LidarDescription;
	LidarData = FTHILidarData(Description.ScannedLines);
}

void ATHILidar::SetOwner(AActor* NewOwner)
{
	Super::SetOwner(NewOwner);
}

void ATHILidar::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(ATHILidar::PostPhysTick);
	LidarScan(DeltaSeconds);
	
	// Write detections to LidarData
	for (auto Detection : Detections)
	{
		LidarData.WritePointSync(Detection);
	}
	
	{
		TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
		auto DataStream = GetDataStream(*this);
		DataStream.Send(*this, LidarData, DataStream.PopBufferFromPool());
	}
}
	

void ATHILidar::LidarScan(const float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE( ATHILidar::LidarScan);
	// Parameters for raycasts
	const uint32 ScannedLines = Description.ScannedLines;
	const uint32 HorizontalPointCount = (uint32)Description.LidarHorizontalFoV / Description.LidarHorizontalResolution;

	const float LidarStartAngle = Description.LidarVerticalFoV / 2.0f;
	const float LidarMaxRange = Description.LidarMaxRange * 100.0f;

	FRotator LidarRotation = LidarOrigin->GetComponentRotation();

	FCollisionQueryParams CollisionQueryParams;
	CollisionQueryParams.bReturnPhysicalMaterial = true;
	CollisionQueryParams.bTraceComplex = true;	
	
	LidarData.ResetMemory(ScannedLines * HorizontalPointCount);
	Detections.clear();
	Detections.reserve(ScannedLines * HorizontalPointCount);

	FCriticalSection WriteDataSection;
	GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
		ParallelFor((int)ScannedLines, [&](int i)
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);
			FHitResult OutHit;
			FVector HitPoint;

			float x, y, z = 0.0f;			// Location of the detection
			float intensity = 0.0f;			// Intensity of the detection
			FVector Start = LidarOrigin->GetComponentLocation();
			FVector End;
			std::cout << "\n -> Original Pitch: " << LidarRotation.Pitch;
			// Get the direction of the ray on the horizontal plane. Ray pitch is determined later. !!!!!!
			float LidarPitch = FMath::DegreesToRadians(LidarRotation.Pitch - (float)i * Description.LidarHorizontalResolution);
			End.Z = sin(LidarPitch);

			// Raster Scanning
			if(i % 2 == 0){
				std::cout << "\n -> Even line: " << i;

				// Column data processing
				for(int j = 0; j < (int)HorizontalPointCount; j++)
				{
					//float LidarYaw = (LidarRotation.Yaw + ((float)j * Description.LidarHorizontalResolution - (Description.LidarHorizontalFoV/2.0f))) * PI / 180;
					float LidarYaw = FMath::DegreesToRadians(LidarRotation.Yaw + (float)j * Description.LidarHorizontalResolution);
					End.X = cos(LidarYaw);
					End.Y = sin(LidarYaw);

					// Get LidarPitch value and shoot line trace.
					// Convert line trace impact point from world space to local space
					bool Trace = GetWorld()->ParallelLineTraceSingleByChannel(OutHit, Start, Start + (End * LidarMaxRange), ECC_Visibility, CollisionQueryParams);
					HitPoint = GetTransform().InverseTransformPosition(OutHit.ImpactPoint);

					// If the rain intensity is 0, impact point in local space is written to LidarData
					// If the rain intensity is greater than 0, noise is added to the detection
					std::cout << "\n -> LidarPitch: " << LidarPitch;
					std::cout << "\n -> LidarYaw: " << LidarYaw;
					if(Description.LidarRainIntensity <= 0)
					{
						x = HitPoint.X;
						y = HitPoint.Y;
						z = HitPoint.Z;
						intensity = 1.0f;
					} else
					{
						FVector NoiseVector;
						AddRainNoise(OutHit, LidarPitch, LidarYaw, i, NoiseVector, intensity);
						x = NoiseVector.X;
						y = NoiseVector.Y;
						z = NoiseVector.Z;
					}
					if(((int)HorizontalPointCount/4) == j){

						// Write detection to LidarData. The section is locked to prevent issues.
						FTHILidarDetection detection {x,y,z,intensity};
						WriteDataSection.Lock();
						Detections.emplace_back(detection);
						WriteDataSection.Unlock();
						std::cout << "\n Column Even: " << j;
						std::cout << "\n Column Even: " << j;
					}
				}
			}
			else{
				std::cout << "\n -> ODD line: " << i;
				// Column data processing
				for(int j = (int)HorizontalPointCount; j >= 0; j--)
				{
					float LidarYaw = FMath::DegreesToRadians(LidarRotation.Yaw + (float)j * Description.LidarHorizontalResolution);
					End.X = cos(LidarYaw);
					End.Y = sin(LidarYaw);

					// Get LidarPitch value and shoot line trace.
					// Convert line trace impact point from world space to local space
					bool Trace = GetWorld()->ParallelLineTraceSingleByChannel(OutHit, Start, Start + (End * LidarMaxRange), ECC_Visibility, CollisionQueryParams);
					HitPoint = GetTransform().InverseTransformPosition(OutHit.ImpactPoint);

					// If the rain intensity is 0, impact point in local space is written to LidarData
					// If the rain intensity is greater than 0, noise is added to the detection
					std::cout << "\n -> LidarPitch: " << LidarPitch;
					std::cout << "\n -> LidarYaw: " << LidarYaw;
					if(Description.LidarRainIntensity <= 0)
					{
						x = HitPoint.X;
						y = HitPoint.Y;
						z = HitPoint.Z;
						intensity = 0.5f;
					} else
					{
						FVector NoiseVector;
						AddRainNoise(OutHit, LidarPitch, LidarYaw, i, NoiseVector, intensity);
						x = NoiseVector.X;
						y = NoiseVector.Y;
						z = NoiseVector.Z;
					}

					// Write detection to LidarData. The section is locked to prevent issues.
					FTHILidarDetection detection {x,y,z,intensity};
					WriteDataSection.Lock();
					Detections.emplace_back(detection);
					WriteDataSection.Unlock();
					std::cout << "\n Column Odd: " << j;
				}

			}
			// Raster Scanning
		});
	}
	GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();
}



{
    TArray<THILidarDetection> LocalDetections;
    FHitResult OutHit;
    FVector HitPoint;

    float x, y, z = 0.0f; // Localização da detecção
    float intensity = 0.0f; // Intensidade da detecção
    FVector Start = LidarOrigin->GetComponentLocation();
    FVector End;

    float LidarPitch = FMath::DegreesToRadians(LidarRotation.Pitch - (float)i * Description.LidarHorizontalResolution);
    End.Z = FMath::Sin(LidarPitch);

    // Verifica se a linha atual é par
    bool isEvenLine = (i % 2 == 0);

    // Define o ponto de início da varredura: 0 para linhas pares, último ponto para linhas ímpares
    int start = isEvenLine ? 0 : (int)HorizontalPointCount - 1;
    int end = isEvenLine ? (int)HorizontalPointCount : -1;
    int step = isEvenLine ? 1 : -1;

    for(int j = start; j != end; j += step)
    {
        float LidarYaw = FMath::DegreesToRadians(LidarRotation.Yaw + (float)j * Description.LidarHorizontalResolution);
        End.X = FMath::Cos(LidarYaw);
        End.Y = FMath::Sin(LidarYaw);

        // Executa o traçado de linha
        bool Trace = GetWorld()->LineTraceSingleByChannel(OutHit, Start, Start + (End * LidarMaxRange), ECC_Visibility, CollisionQueryParams);
        if (!Trace) continue;

        HitPoint = GetTransform().InverseTransformPosition(OutHit.ImpactPoint);

        if(Description.LidarRainIntensity <= 0)
        {
            x = HitPoint.X;
            y = HitPoint.Y;
            z = HitPoint.Z;
            intensity = 1.0f;
        }
        else
        {
            FVector NoiseVector;
            AddRainNoise(OutHit, LidarPitch, LidarYaw, i, NoiseVector, intensity);
            x = NoiseVector.X;
            y = NoiseVector.Y;
            z = NoiseVector.Z;
        }

        // Adiciona a detecção à lista local usando o construtor correto
        LocalDetections.Add(THILidarDetection{x, y, z, intensity});
    }
}




/* void ATHILidar::RayTracing(FVector &End, FVector &HitPoint, int j, FHitResult &OutHit, FVector &Start, float LidarMaxRange, FCollisionQueryParams &CollisionQueryParams){
	float LidarYaw = FMath::DegreesToRadians(LidarRotation.Yaw + (float)j * Description.LidarHorizontalResolution);
	End.X = cos(LidarYaw);
	End.Y = sin(LidarYaw);

	// Get LidarPitch value and shoot line trace.
	// Convert line trace impact point from world space to local space
	bool Trace = GetWorld()->ParallelLineTraceSingleByChannel(OutHit, Start, Start + (End * LidarMaxRange), ECC_Visibility, CollisionQueryParams);
	HitPoint = GetTransform().InverseTransformPosition(OutHit.ImpactPoint);

	// If the rain intensity is 0, impact point in local space is written to LidarData
	// If the rain intensity is greater than 0, noise is added to the detection
	std::cout << "\n -> LidarYaw: " << LidarYaw;
	if(Description.LidarRainIntensity <= 0)
	{
		x = HitPoint.X;
		y = HitPoint.Y;
		z = HitPoint.Z;
		intensity = 0.5f;
	} else
	{
		FVector NoiseVector;
		AddRainNoise(OutHit, LidarPitch, LidarYaw, i, NoiseVector, intensity);
		x = NoiseVector.X;
		y = NoiseVector.Y;
		z = NoiseVector.Z;
	}

	// Write detection to LidarData. The section is locked to prevent issues.
	FTHILidarDetection detection {x,y,z,intensity};
	WriteDataSection.Lock();
	Detections.emplace_back(detection);
	WriteDataSection.Unlock();
	std::cout << "\n Column Odd: " << j;
}
 */
float ATHILidar::GetReflectivity(FHitResult &HitResult)
{
	// Default reflectivity value is 0.5, default material type is set to None
	// Weather actor contains SnowAmount and Wetness variables
	// If Weather actor is not found in the level, use 0 as the value.
	float Reflectivity = 0.5f;
	float SnowAmount = 0;
	float Wetness = 0;
	ERoadviewMaterialTypes MaterialType = ERoadviewMaterialTypes::None;
	float WetnessMultiplier = FMath::Lerp(1.0f, 0.5f, Wetness / 100.0f);
	if(Weather != NULL)
	{
		SnowAmount = Weather->GetCurrentWeather().SnowAmount;
		Wetness = Weather->GetCurrentWeather().Wetness;
	} else
	{
		GEngine->AddOnScreenDebugMessage(-1, 5, FColor::Red, "Weather class not found!");
	}

	// If detection has a RoadviewPhysicalMaterial, fetch reflectivity and material type from it
	// Reflectivity value is multiplied with wetness multiplier value
	URoadviewPhysicalMaterial* HitPhysMat = Cast<URoadviewPhysicalMaterial>(HitResult.PhysMaterial.Get());
	if(HitPhysMat != nullptr)
	{
		Reflectivity = HitPhysMat->Reflectivity;
		MaterialType = HitPhysMat->MaterialType;
	}
	float ReflectivityWetness = Reflectivity * WetnessMultiplier;

	// If snow amount is 0 return reflectivity value multiplied with wetness multiplier
	// If snow amount is greater than 0, check if the material type is either asphalt or snow.
	// If material type is neither or them, calculate if the ray hits an area covered in snow.
	if(SnowAmount <= 0)
	{
		return ReflectivityWetness;
	} else
	{
		if(MaterialType == ERoadviewMaterialTypes::Asphalt)
		{
			return ReflectivityWetness;
		} else if(MaterialType == ERoadviewMaterialTypes::Snow)
		{
			return HitPhysMat->Reflectivity;
		} else
		{
			// In snow covered objects' materials, snow is blended in using the dot product of face normal and up vector
			// The higher SnowAmount variable is in Weather actor, the more the objects are covered in snow
			// Using DotProduct of hit normal and up vector, we can calculate if the ray hit an area covered by snow
			// Snow starts to covered object when SnowAmount is 30, with the maximum value being 100.
			float DotProduct = FVector::DotProduct(HitResult.Normal, FVector::UpVector);
			float Lerp = FMath::Clamp((SnowAmount - 30.0f) / 70.0f, 0.0f, 1.0f);
			float Threshold = FMath::Lerp(0.99f, 0.2f, Lerp);
			
			if(DotProduct > Threshold)
			{
				return 0.85f;
			} else
			{
				return ReflectivityWetness;
			}
		}
	}

	// If none of the conditions above are met, return reflectivity multiplier with wetness multiplier
	return ReflectivityWetness;
}


void ATHILidar::AddRainNoise(FHitResult &HitResult, float LidarPitch, float LidarYaw, float ChannelIndex, FVector &OutVector, float &Intensity)
{
	// Rain parameters
	float RainIntensity = Description.LidarRainIntensity;
	float ThresholdMinusFive = Description.ThresholdMinusFive;
	float LidarMaxRange = Description.LidarMaxRange * 100.0f;
	FVector HitPoint = GetTransform().InverseTransformPosition(HitResult.ImpactPoint);
	float LidarRotation = FMath::DegreesToRadians(LidarOrigin->GetComponentRotation().Yaw);
	
	float BeamRadius = 0.0035f;
	int numsteps = 50;
	float int_sum = 0;

	// Sum of drops along the trace?
	for(int i = 0; i < numsteps; i++)
	{
		float stepsize = (6 - 0.5) / numsteps;
		float midpoint = ((0.5 + (i + 1) * stepsize) + (0.5 + (i * stepsize))) / 2;
		float func_int = 8000 * exp(-4.1 * pow(RainIntensity, -0.21) * midpoint);
		int_sum += func_int * stepsize;
	}

	// If the linetrace has hit something, noise is added if generated number is greater than 1
	// and if the DropIntensity is greater than threshold and greater than hit object intensity
	// If the linetrace hasn't hit anything, noise is added if generated number is greater than 2
	// and the DropIntensity is greater than threshold
	// Object reflectivity is stored in a material parameter, if no reflectivity parameter is found, value defaults to 0.5
	
	if(HitResult.bBlockingHit)
	{
		float VBeam = 3.1416f * FMath::Pow(BeamRadius, 2) * HitResult.Distance / 100.0f;
		float Mean_V = VBeam * int_sum;
		std::poisson_distribution<int> Distribution(Mean_V);
		int poisson_num = Distribution(generator_thi);
		float Dist_to_drop = (rand() % 90 + 10) * 0.01f * HitResult.Distance;

		// Default reflectivity value is 0.5, if the hit material has a Reflecivity Parameter, it is used
		float Reflectivity = GetReflectivity(HitResult);
		
		float ReturnedIntensityObj = 100000 * pow(0.00005f, poisson_num) * Reflectivity * (1 - exp(0.1f * ((HitResult.Distance) - (1.05f * LidarMaxRange))));
		float ReturnedIntensityDrop = 0.5f * (1 - exp(0.0005f * ((Dist_to_drop)-(1.05 * LidarMaxRange))));
		
		// Determine if noise is added or use hitpoint when the trace hits an object
		if ((poisson_num < 1) || (ReturnedIntensityObj > ThresholdMinusFive && ReturnedIntensityObj > ReturnedIntensityDrop)) {
			OutVector.X = HitPoint.X;
			OutVector.Y = HitPoint.Y;
			OutVector.Z = HitPoint.Z;
			Intensity = 1.0f;
		} else if (poisson_num > 1 && ReturnedIntensityDrop > ThresholdMinusFive && ReturnedIntensityObj < ReturnedIntensityDrop) {
			OutVector.X = 0.5 * cos(LidarPitch* (3.1416 / 180)) * cos(LidarYaw - LidarRotation) * Dist_to_drop;
			OutVector.Y = 0.5 * cos(LidarPitch* (3.1416 / 180)) * sin(LidarYaw - LidarRotation) * Dist_to_drop;
			OutVector.Z = 0.5 * sin(LidarPitch) * Dist_to_drop;
			Intensity = 1.0f;
		}
	} else {
		float VBeam = 3.1416f * FMath::Pow(BeamRadius, 2) * LidarMaxRange / 100.0f;
		float Mean_V = VBeam * int_sum;
		std::poisson_distribution<int> Distribution(Mean_V);
		int poisson_num = Distribution(generator_thi);
		float Dist_to_drop = (rand() % 90 + 10) * 0.01f * LidarMaxRange;
		float ReturnedIntensityDrop = 0.5f * (1 - exp(0.0005f * ((Dist_to_drop)-(1.05f * LidarMaxRange))));

		// Determine if noise is added when the trace doesn't hit anything
		if(poisson_num > 2 && ReturnedIntensityDrop > ThresholdMinusFive)
		{
			OutVector.X = 0.5 * cos(LidarPitch* (3.1416 / 180)) * cos(LidarYaw - LidarRotation) * Dist_to_drop;
			OutVector.Y = 0.5 * cos(LidarPitch* (3.1416 / 180)) * sin(LidarYaw - LidarRotation) * Dist_to_drop;
			OutVector.Z = 0.5 * sin(LidarPitch) * Dist_to_drop;
			Intensity = 1.0f;
		}
	}
}
