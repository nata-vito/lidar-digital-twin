#pragma once

#include "ZFRADARDescription.generated.h"

USTRUCT()
struct CARLA_API FZFRADARDescription
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	uint32 RadarChannels = 21u;

	UPROPERTY(EditAnywhere)
	float RadarMaxRange = 350.0f;

	UPROPERTY(EditAnywhere)
	uint32 LidarUpdateRate = 10;

	UPROPERTY(EditAnywhere)
	float RadarHorizontalResolution = 151.0f;

	UPROPERTY(EditAnywhere)
	float RadarRainIntensity = 0.0f;

	UPROPERTY(EditAnywhere)
	float ThresholdMinusFive = 0.0f;

	UPROPERTY(EditAnywhere)
	double Frequency = 76.5;//GHz

	UPROPERTY(EditAnywhere)
	uint32 Speed_of_Light = 299792458u;
	
	UPROPERTY(EditAnywhere)
	float FoV_Azimuth = 160.0f;

	UPROPERTY(EditAnywhere)
	float FoV_Elevation = 30.0f;

	UPROPERTY(EditAnywhere)
	float emmitted_power = 2.50f;
};