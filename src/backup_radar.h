#pragma once

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/Sensor.h"
#include "Components/SceneComponent.h"
#include "Carla/Sensor/ZFRADARDescription.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/ZFRADARData.h>
#include <compiler/enable-ue4-macros.h>
#include <math.h>

#include <vector>
#include <complex>

#include "ZFRADAR.generated.h"

UCLASS()
class CARLA_API AZFRADAR : public ASensor
{
    GENERATED_BODY()

    using FZFRADARData = carla::sensor::data::ZFRADARData;
    using FZFRADARDetection = carla::sensor::data::ZFRADARDetection;

public:
    static FActorDefinition GetSensorDefinition();
    
    AZFRADAR(const FObjectInitializer &ObjectInitializer);

    virtual void Set(const FActorDescription& ActorDescription) override;
    virtual void Set(const FZFRADARDescription &LidarDescription);
    
    void SetOwner(AActor* NewOwner) override;
    //void Tick(float DeltaSeconds) override;
    void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds) override;


    UPROPERTY(EditAnywhere)
    FZFRADARDescription Description;
private:
    USceneComponent* RadarOrigin = nullptr;
    FVector PrevLocation;
    FVector CurrentVelocity;
    std::vector<uint32_t> PointsPerChannel;
    std::vector<std::vector<int>> HitCount;
    std::vector<FZFRADARDetection> Detections;
    FZFRADARData RadarData;

    void CalculateCurrentVelocity(const float DeltaTime);
    float CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& RadarLocation);
    float CalculateRCS(const TArray<FVector>& Vertices, const FVector& IncidentDirection,  const float Wavelength, float relative_permittivity);
    
    std::vector<float> linspace(float start, float end, int N);

    void RadarScan(const float DeltaTime);
    void AddRainNoise(FHitResult OutHit, float LidarPitch, float LidarYaw, FVector &OutVector, float &Intensity);
    
    void fft(std::vector<std::complex<double>> &a, int n);

    struct RayData {
        float Radius;
        float Angle;
        bool Hitted;
        float RelativeVelocity;
        FVector2D AzimuthAndElevation;
        float Distance;
        float RCS;
    };
};