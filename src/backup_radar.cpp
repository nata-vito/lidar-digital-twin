#include "Carla.h"
#include "Carla/Sensor/ZFRADAR.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Actor/ActorBlueprintFunctionLibrary.h"
#include <random>
#include <complex>
#include <vector>
#include <math.h>
#include <cmath>
#include "Eigen/Dense"
#include "Math/UnrealMathUtility.h"
#include "DrawDebugHelpers.h"
#include "PhysicsInterfaceUtilsCore.h"
#include "Engine/Engine.h"
#include "SceneTypes.h"
#include "PhysicalMaterials/RoadviewPhysicalMaterial.h"

#include "Async/ParallelFor.h"

namespace crp = carla::rpc;

//std::default_random_engine generator;

FActorDefinition AZFRADAR::GetSensorDefinition()
{
    return UActorBlueprintFunctionLibrary::MakeZFRADARDefinition(TEXT("zf_radar"));
}

AZFRADAR::AZFRADAR(const FObjectInitializer &ObjectInitializer) : Super(ObjectInitializer)
{   // do one antenna wiht the gain and power form the
    PrimaryActorTick.bCanEverTick = true;

    RadarOrigin = CreateDefaultSubobject<USceneComponent>(TEXT("Radar Origin"));
    RadarOrigin->SetupAttachment(RootComponent);
}

void AZFRADAR::Set(const FActorDescription& ActorDescription)
{
    Super::Set(ActorDescription);
    FZFRADARDescription LidarDescription;
    UActorBlueprintFunctionLibrary::SetZFRADAR(ActorDescription, LidarDescription);
    Set(LidarDescription);
}

void AZFRADAR::Set(const FZFRADARDescription& LidarDescription)
{
    Description = LidarDescription;
    RadarData = FZFRADARData(Description.RadarChannels);
    PointsPerChannel.resize(Description.RadarChannels);
}


void AZFRADAR::SetOwner(AActor* NewOwner)
{
    Super::SetOwner(NewOwner);
}

void AZFRADAR::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds) {
    CalculateCurrentVelocity(DeltaSeconds);

    RadarScan(DeltaSeconds);
    auto DataStream = GetDataStream(*this);
    DataStream.SerializeAndSend(*this, RadarData, DataStream.PopBufferFromPool());
}

std::vector<float> AZFRADAR::linspace(float start, float end, int N) {
    std::vector<float> result(N);
    float step = (end - start) / (N - 1);

    for (int i = 0; i < N; ++i) {
        result[i] = start + i * step;
    }

    return result;
}


void AZFRADAR::RadarScan(const float DeltaTime)
{
    const TArray<double> patRx_El = {0.6412, 14.2568, 2.6143, 11.4898, 6.1267, 7.4028, 10.4961, 3.1397, 14.2166, 0.3796, 15.6873, 0.3796, 14.2166, 3.1397, 10.4961, 7.4028, 6.1267, 11.4898, 2.6143, 14.2568, 0.6412};
    const TArray<double> patTx_El = {0.1228, 0.1588, 1.9169, 5.1905, 1.7212, 0.1398, 0.3064, 0.0595, 0.2131, 5.5098, 12.3954, 5.5098, 0.2131, 0.0595, 0.3064, 0.1398, 1.7212, 5.1905, 1.9169, 0.1588, 0.1228};
    const TArray<double> patRx_Az = {0.7128, 0.3499, 0.0184, 0.2563, 1.6510, 4.4932, 8.4513, 12.4933, 15.2066, 15.4441, 12.9739, 8.7094, 4.2927, 1.2178, 0.0420, 0.2291, 0.7217, 0.7768, 0.3941, 0.0436, 0.0491, 0.2693, 0.3513, 0.1879, 0.0134, 0.0527, 0.2136, 0.2375, 0.0874, 0.0009, 0.1193, 0.2517, 0.1727, 0.0126, 0.0755, 0.3062, 0.3142, 0.0613, 0.0748, 0.5676, 0.8047, 0.2426, 0.2273, 3.2114, 9.3489, 14.7863, 14.9555, 9.6235, 3.2936, 0.2045, 0.2928, 0.8219, 0.4488, 0.0112, 0.1692, 0.3564, 0.1510, 0.0019, 0.1710, 0.2406, 0.0560, 0.0308, 0.2220, 0.1984, 0.0098, 0.1230, 0.3532, 0.1810, 0.0117, 0.4801, 0.8089, 0.1740, 0.5609, 5.0068, 12.1024, 15.6873, 12.1024, 5.0068, 0.5609, 0.1740, 0.8089, 0.4801, 0.0117, 0.1810, 0.3532, 0.1230, 0.0098, 0.1984, 0.2220, 0.0308, 0.0560, 0.2406, 0.1710, 0.0019, 0.1510, 0.3564, 0.1692, 0.0112, 0.4488, 0.8219, 0.2928, 0.2045, 3.2936, 9.6235, 14.9555, 14.7863, 9.3489, 3.2114, 0.2273, 0.2426, 0.8047, 0.5676, 0.0748, 0.0613, 0.3142, 0.3062, 0.0755, 0.0126, 0.1727, 0.2517, 0.1193, 0.0009, 0.0874, 0.2375, 0.2136, 0.0527, 0.0134, 0.1879, 0.3513, 0.2693, 0.0491, 0.0436, 0.3941, 0.7768, 0.7217, 0.2291, 0.0420, 1.2178, 4.2927, 8.7094, 12.9739, 15.4441, 15.2066, 12.4933, 8.4513, 4.4932, 1.6510, 0.2563, 0.0184, 0.3499, 0.7128};
    const TArray<double> patTx_Az = {4.9352, 6.2935, 5.0090, 2.0112, 0.0706, 1.5737, 6.1171, 10.3171, 10.5822, 6.5894, 1.8719, 0.3973, 2.4380, 4.4462, 3.4449, 0.7808, 0.2694, 2.8171, 4.8687, 3.3775, 0.4895, 0.6184, 3.6448, 4.7951, 2.1592, 0.0408, 1.9853, 4.6584, 3.5698, 1.2956, 3.0138, 6.4354, 5.1651, 0.7968, 0.8730, 4.8867, 4.8902, 0.8435, 0.8423, 4.8074, 4.4661, 0.5115, 1.2805, 4.9884, 3.6964, 1.1938, 4.8308, 7.9650, 3.4231, 0.0797, 4.2564, 5.8734, 1.1694, 1.0865, 5.8191, 4.2688, 0.0768, 3.3184, 7.0359, 3.3485, 0.4743, 2.2114, 1.4218, 0.2478, 3.6663, 4.3082, 0.4398, 1.7209, 5.1838, 2.0970, 0.3217, 4.6067, 3.7965, 0.0058, 6.0333, 12.3954, 6.0333, 0.0058, 3.7965, 4.6067, 0.3217, 2.0970, 5.1838, 1.7209, 0.4398, 4.3082, 3.6663, 0.2478, 1.4218, 2.2114, 0.4743, 3.3485, 7.0359, 3.3184, 0.0768, 4.2688, 5.8191, 1.0865, 1.1694, 5.8734, 4.2564, 0.0797, 3.4231, 7.9650, 4.8308, 1.1938, 3.6964, 4.9884, 1.2805, 0.5115, 4.4661, 4.8074, 0.8423, 0.8435, 4.8902, 4.8867, 0.8730, 0.7968, 5.1651, 6.4354, 3.0138, 1.2956, 3.5698, 4.6584, 1.9853, 0.0408, 2.1592, 4.7951, 3.6448, 0.6184, 0.4895, 3.3775, 4.8687, 2.8171, 0.2694, 0.7808, 3.4449, 4.4462, 2.4380, 0.3973, 1.8719, 6.5894, 10.5822, 10.3171, 6.1171, 1.5737, 0.0706, 2.0112, 5.0090, 6.2935, 4.9352};

    const double frequency = Description.Frequency * 1e9;

    const double wavelength = Description.Speed_of_Light / frequency; // 76.5GHz
    const double minimum_temperature_noise_loss = 4e-21 * frequency;
    const double wavelength_squared = pow(wavelength, 2);

    const double pi_times_four = 4 * M_PI;
    const double pi_times_four_cubed = pow(pi_times_four, 3);
    const double pi_times_four_squared = pow(pi_times_four, 2);

    const float air_permittivity = 1.0006f;

    const float f_start = 76.5e9;                // start frequency
    const float f_end = 77.5e9;                  // end frequency
    const float S = 60e6 / 1e-6;                 // slope MHz/µs
    const float T_adc = (f_end - f_start) / S;   // ADC sampling time
    const float N_ADC = 512;                     // number of ADC-samples
    const float c = 3e8;                         // speed of light [m/s]
    const float F_s = N_ADC / T_adc;             // sampling frequency
    const float maxRange = F_s * c / (2 * S);

    std::vector<float> chirp = linspace(f_start, f_end, N_ADC);

    std::vector<float> transmitted_wavelength(N_ADC);
    for (int i = 0; i < N_ADC; ++i) {
        transmitted_wavelength[i] = c / chirp[i];
    }

    std::vector<std::complex<double>> signal_Rx(N_ADC, std::complex<double>(0.0, 0.0));

    // Parameters for raycasts
    const uint32 RadarChannels = Description.RadarChannels;
    const uint32 HorizontalPointCount = Description.RadarHorizontalResolution;
    const float LidarStartAngle = 0;
    const float RadarMaxRange = Description.RadarMaxRange * 100.0f;
    const float EMMITED_POWER = Description.emmitted_power;//25118.86f;

    FRotator RadarRotation = RadarOrigin->GetComponentRotation();
    FCollisionQueryParams CollisionQueryParams;
    CollisionQueryParams.bReturnPhysicalMaterial = true;
    CollisionQueryParams.bReturnFaceIndex  = true;

    // Clear RecordedHits, LidarData, and Detections
    HitCount.resize(HorizontalPointCount);
    for (auto& hits : HitCount)
    {
        hits.clear();
        hits.reserve(RadarChannels);
    }

    for (int i = 0; i < (int)RadarChannels; i++)
        PointsPerChannel[i] = HitCount[i].size();
    RadarData.ResetMemory(PointsPerChannel);

    const int output_channels = 4 + 3;

    std::vector<float> RADAR_power_data(HorizontalPointCount * RadarChannels * output_channels, 0.0f);

    FCriticalSection WriteDataSection;
    GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
    ParallelFor(HorizontalPointCount, [&](int i) {
        FHitResult OutHit;
        FVector HitPoint;

        float x, y, z = 0.0f;           // Location of the detection
        float intensity = 0.0f;         // Intensity of the detection
        float power = 0.0f;
        float distance_to_hit = 0.0f;
        float current_antenna_gain_Tx = 0.0f;
        float current_antenna_gain_Rx = 0.0f;
        FVector Start = RadarOrigin->GetComponentLocation();
        FVector End;
        float RCS, SNR = 0;
        float noise = 0;
        float power_density_at_the_target, scattered_powered_density = 0; // Wm^-2
        float power_reflection_coefficient = 0;
        // Get the direction of the ray on the horizontal plane. Ray pitch is determined later.
        float LidarYaw = FMath::DegreesToRadians(RadarRotation.Yaw + (((float)i * Description.FoV_Azimuth / Description.RadarHorizontalResolution) - Description.FoV_Azimuth / 2)) ;

        End.X = cos(LidarYaw);
        End.Y = sin(LidarYaw);

        float RelativeVelocity = 0;

        for (int j = 0; j < (int)RadarChannels; j++) {
            // Get LidarPitch value and shoot linetrace.
            // Convert linetrace impact point from world space to local space
            float LidarPitch = FMath::DegreesToRadians((((float)j * Description.FoV_Elevation / Description.RadarChannels) - Description.FoV_Elevation / 2));

            End.Z = sin(LidarPitch);

            bool Trace = GetWorld()->ParallelLineTraceSingleByChannel(OutHit, Start, Start + (End * RadarMaxRange), ECC_Visibility, CollisionQueryParams);

            HitPoint = GetTransform().InverseTransformPosition(OutHit.ImpactPoint);

            // DrawDebugLine(GetWorld(), Start, Start + (End * RadarMaxRange), FColor::Red, false, 5.0f, 0, 0.1f);

            if (OutHit.Distance <= 100) {
                Trace = false;
            }
                float relative_permittivity = 2.0f;

                HitCount[i].emplace_back(1);

                const TWeakObjectPtr<AActor> HittedActor = OutHit.Actor;

                RelativeVelocity = CalculateRelativeVelocity(OutHit, Start);

                distance_to_hit = OutHit.Distance * 1e-2; // converting to METERS

                current_antenna_gain_Tx = patTx_Az[i] + patTx_El[j];
                current_antenna_gain_Rx = patRx_Az[i] + patRx_El[j];

                power_density_at_the_target = EMMITED_POWER * current_antenna_gain_Tx * current_antenna_gain_Rx * pow((wavelength / (pi_times_four * distance_to_hit)), 2); //Friis Equation   //EMMITED_POWER - noise;

                noise = EMMITED_POWER - power_density_at_the_target;

                URoadviewPhysicalMaterial* HitPhysMat = Cast<URoadviewPhysicalMaterial>(OutHit.PhysMaterial.Get());

                if (HitPhysMat != nullptr) {
                    relative_permittivity = HitPhysMat->RadarPermittivity;
                } else {
                    relative_permittivity = 2.0f;
                }

                // Get the triangle index if available
                int32 Item = OutHit.Item;	

                // Get the hit location
                FVector HitLocation = OutHit.ImpactPoint;

                // Get the normal of the hit surface
                FVector HitNormal = OutHit.ImpactNormal;

                TArray<FVector> Vertices;

                UStaticMeshComponent* HitComponent = Cast<UStaticMeshComponent>(OutHit.GetComponent());

                if (HitComponent && OutHit.FaceIndex != INDEX_NONE) {
                    if (HitComponent->GetStaticMesh()) {
                        const FStaticMeshLODResources& LODModel = HitComponent->GetStaticMesh()->RenderData->LODResources[0];
                        const uint32 VertIndex = OutHit.FaceIndex * 3;
                        for (int32 i = 0; i < 3; ++i) {
                            Vertices.Add(LODModel.VertexBuffers.PositionVertexBuffer.VertexPosition(LODModel.IndexBuffer.GetIndex(VertIndex + i)));
                        }
                    }
                    RCS = CalculateRCS(Vertices, HitNormal, wavelength, relative_permittivity);
                } else {
                    RCS = 1;
                }

                power = (EMMITED_POWER * pow(current_antenna_gain_Tx, 2) * wavelength_squared * RCS) / (pow(distance_to_hit, 4) * pi_times_four_cubed);

                SNR = power / noise;

            }

            int index = (i * RadarChannels + j) * output_channels;

            RADAR_power_data[index + 0] = RCS;
            RADAR_power_data[index + 1] = power;
            RADAR_power_data[index + 2] = SNR;
            RADAR_power_data[index + 3] = distance_to_hit;
            RADAR_power_data[index + 4] = HitPoint.X * 1e-2;
            RADAR_power_data[index + 5] = HitPoint.Y * 1e-2;
            RADAR_power_data[index + 6] = HitPoint.Z * 1e-2;

            if (Description.RadarRainIntensity <= 0) {
                x = HitPoint.X;
                y = HitPoint.Y;
                z = HitPoint.Z;
            }

        }
    });

    for (int i = 0; i < (int)HorizontalPointCount; i++) {
        for (int j = 0; j < (int)RadarChannels; j++) {
            int index = (i * RadarChannels + j) * output_channels;

            float amp_ADC = std::sqrt(RADAR_power_data[index + 0]) / (RADAR_power_data[index + 3] * RADAR_power_data[index + 3]);
            std::vector<float> phi_ADC(N_ADC);

            for (int k = 0; k < transmitted_wavelength.size(); ++k) {
                phi_ADC[k] = std::arg(std::exp(std::complex<float>(0.0, 2.0 * M_PI * 2.0 * RADAR_power_data[index + 3] / transmitted_wavelength[k])));
                signal_Rx[k] += amp_ADC * std::exp(std::complex<float>(0.0, phi_ADC[k]));
            }
        }
    }
    std::vector<std::complex<double>> signal_Rx_fft = signal_Rx;
    fft(signal_Rx_fft, N_ADC);

    std::vector<double> signal_abs;
    signal_abs.reserve(signal_Rx_fft.size());

    std::vector<float> range = linspace(0.001, maxRange, N_ADC);

    for (size_t i = 0; i < signal_Rx_fft.size(); ++i) {
        std::complex<double> number = signal_Rx_fft[i];
        signal_abs.push_back(pow(std::abs(number / (double) N_ADC), 2) * pow(range[i],4));
    }
    UE_LOG(LogTemp, Warning, TEXT("New Number incomming"));
    for (size_t i = 0; i < signal_abs.size(); ++i) {
        std::complex<double> complexNumber = signal_Rx[i];
        UE_LOG(LogTemp, Warning, TEXT("%d %f %f\n"), i,  complexNumber.real(), complexNumber.imag());
    }

    //FZFRADARDetection detection {x,y,z,RelativeVelocity,RCS,power,noise};
    //WriteDataSection.Lock();
    //RadarData.WritePointSync(detection);
    //WriteDataSection.Unlock();
    GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();
}

float AZFRADAR::CalculateRCS(const TArray<FVector>& Vertices,
                             const FVector& IncidentDirection,
                             const float Wavelength,
                             float relative_permittivity) {
    // add more triangles if the target is closeby :)
    //mounting position
    FVector incident_direction = FVector(0, 0, 1); // Assuming radar waves are coming from the positive z-direction

    float facet_area = (FVector::CrossProduct(Vertices[1] - Vertices[0], Vertices[2] - Vertices[0]).Size() / 2) * 1e-4;
    float dot_product = FMath::Abs(FVector::DotProduct(IncidentDirection, incident_direction)); // Dot product between normals and incident direction

    // Calculate RCS using Method of Moments
    float RCS = (4 * M_PI * facet_area * facet_area) / (Wavelength * Wavelength) * dot_product * dot_product * relative_permittivity;

    if (RCS == 0) {
        RCS = 1e-10;
    }

    //UE_LOG(LogTemp, Warning, TEXT("%f;%f;%f;%f %f %f;%f %f %f;%f %f %f;%f %f %f"),
    //                        RCS, facet_area, relative_permittivity,
    //                        IncidentDirection.X, IncidentDirection.Y, IncidentDirection.Z ,
    //                        Vertices[0].X, Vertices[0].Y, Vertices[0].Z,
    //                        Vertices[1].X, Vertices[1].Y, Vertices[1].Z,
    //                        Vertices[2].X, Vertices[2].Y, Vertices[2].Z);
    return RCS;
}

float AZFRADAR::CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& RadarLocation) {
    constexpr float TO_METERS = 1e-2;

    const TWeakObjectPtr<AActor> HittedActor = OutHit.Actor;
    const FVector TargetVelocity = HittedActor->GetVelocity();
    const FVector TargetLocation = OutHit.ImpactPoint;
    const FVector Direction = (TargetLocation - RadarLocation).GetSafeNormal();
    const FVector DeltaVelocity = (TargetVelocity - CurrentVelocity);
    const float V = TO_METERS * FVector::DotProduct(DeltaVelocity, Direction);

    return V;
}

void AZFRADAR::CalculateCurrentVelocity(const float DeltaTime) {
    const FVector RadarLocation = GetActorLocation();
    CurrentVelocity = (RadarLocation - PrevLocation) / DeltaTime;
    PrevLocation = RadarLocation;
}

void AZFRADAR::AddRainNoise(FHitResult OutHit, float LidarPitch, float LidarYaw, FVector &OutVector, float &Intensity) {
    // Rain parameters
    float RainIntensity = Description.RadarRainIntensity;
    float ThresholdMinusFive = Description.ThresholdMinusFive;
    float RadarMaxRange = Description.RadarMaxRange * 100.0f;
    FVector HitPoint = GetTransform().InverseTransformPosition(OutHit.ImpactPoint);
    float LidarRotation = FMath::DegreesToRadians(RadarOrigin->GetComponentRotation().Yaw);

    float BeamRadius = 0.0035f;
    int numsteps = 50;
    float int_sum = 0;

    // Sum of drops along the trace?
    for (int i = 0; i < numsteps; i++)
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
    if (OutHit.bBlockingHit)
    {
        float VBeam = 3.1416f * FMath::Pow(BeamRadius, 2) * OutHit.Distance / 100.0f;
        float Mean_V = VBeam * int_sum;
        std::poisson_distribution<int> Distribution(Mean_V);
        int poisson_num = Distribution(gen);
        float Dist_to_drop = (rand() % 90 + 10) * 0.01f * OutHit.Distance;

        // Default reflectivity value is 0.5, if the hit material has a Reflecivity Parameter, it is used
        float Reflectivity = 0.5f;
        int FaceIndex = OutHit.FaceIndex;
        int ElementIndex = OutHit.ElementIndex;
        URoadviewPhysicalMaterial* HitPhysMat = Cast<URoadviewPhysicalMaterial>(OutHit.PhysMaterial.Get());
        if (HitPhysMat != nullptr)
        {
            Reflectivity = HitPhysMat->Reflectivity;
        }

        float ReturnedIntensityObj = 100000 * pow(0.00005f, poisson_num) * Reflectivity * (1 - exp(0.1f * ((OutHit.Distance) - (1.05f * RadarMaxRange))));
        float ReturnedIntensityDrop = 0.5f * (1 - exp(0.0005f * ((Dist_to_drop) - (1.05 * RadarMaxRange))));

        // Determine if noise is added or use hitpoint when the trace hits an object
        if ((poisson_num < 1) || (ReturnedIntensityObj > ThresholdMinusFive && ReturnedIntensityObj > ReturnedIntensityDrop)) {
            OutVector.X = HitPoint.X;
            OutVector.Y = HitPoint.Y;
            OutVector.Z = HitPoint.Z;
            Intensity = 1.0f;
        } else if (poisson_num > 1 && ReturnedIntensityDrop > ThresholdMinusFive && ReturnedIntensityObj < ReturnedIntensityDrop) {
            OutVector.X = 0.5 * cos(LidarPitch) * cos(LidarYaw - LidarRotation) * Dist_to_drop;
            OutVector.Y = 0.5 * cos(LidarPitch) * sin(LidarYaw - LidarRotation) * Dist_to_drop;
            OutVector.Z = 0.5 * sin(LidarPitch) * Dist_to_drop;
            Intensity = 1.0f;
        }
    } else {
        float VBeam = 3.1416f * FMath::Pow(BeamRadius, 2) * RadarMaxRange / 100.0f;
        float Mean_V = VBeam * int_sum;
        std::poisson_distribution<int> Distribution(Mean_V);
        int poisson_num = Distribution(gen);
        float Dist_to_drop = (rand() % 90 + 10) * 0.01f * RadarMaxRange;
        float ReturnedIntensityDrop = 0.5f * (1 - exp(0.0005f * ((Dist_to_drop) - (1.05f * RadarMaxRange))));

        // Determine if noise is added when the trace doesn't hit anything
        if (poisson_num > 2 && ReturnedIntensityDrop > ThresholdMinusFive)
        {
            OutVector.X = 0.5 * cos(LidarPitch) * cos(LidarYaw - LidarRotation) * Dist_to_drop;
            OutVector.Y = 0.5 * cos(LidarPitch) * sin(LidarYaw - LidarRotation) * Dist_to_drop;
            OutVector.Z = 0.5 * sin(LidarPitch) * Dist_to_drop;
            Intensity = 1.0f;
        }
    }
}


void AZFRADAR::fft(std::vector<std::complex<double>> &a, int n) {
    int N = n == -1 ? a.size() : n;
    if (N <= 1) return;

    std::vector<std::complex<double>> a0(N / 2), a1(N / 2);
    for (int i = 0, j = 0; i < N; i += 2, ++j) {
        a0[j] = a[i];
        a1[j] = a[i + 1];
    }
    fft(a0, N / 2);
    fft(a1, N / 2);

    double ang = 2 * M_PI / N;
    std::complex<double> w(1), wn(cos(ang), sin(ang));
    for (int i = 0; i < N / 2; ++i) {
        a[i] = a0[i] + w * a1[i];
        a[i + N / 2] = a0[i] - w * a1[i];
        w *= wn;
    }
}

            if (Trace) {