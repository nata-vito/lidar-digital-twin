#pragma once

#include "carla/rpc/Location.h"
#include "carla/rpc/Location.h"
#include "carla/sensor/data/Array.h"
#include "carla/sensor/s11n/ZFRADARSerializer.h"

namespace carla {
namespace sensor {
namespace data {
    class ZFRADARMeasurement : public Array<data::ZFRADARDetection> {
        static_assert(sizeof(data::ZFRADARDetection) == 7u * sizeof(float), "Location size mismatch");
        using Super = Array<data::ZFRADARDetection>;

        protected:
            using Serializer = s11n::ZFRADARSerializer;
            friend Serializer;

            explicit ZFRADARMeasurement(RawData &&data)
             : Super(std::move(data), [](const RawData &d){
                return Serializer::GetHeaderOffset(d);
             }) {}

        private:

            auto GetHeader() const {
                return Serializer::DeserializeHeader(Super::GetRawData());
            }

        public:
            auto GetChannelCount() const {
                return GetHeader().GetChannelCount();
            }

            auto GetPointCount(size_t channel) const {
                return GetHeader().GetPointCount(channel);
            }
    };
}
}
}