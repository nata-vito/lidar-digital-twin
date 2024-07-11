#pragma once

#include "carla/rpc/Location.h"

#include <cstdint>
#include <vector>
#include <numeric>

namespace carla {
namespace sensor {

namespace s11n{
	class ZFRADARSerializer;
	class ZFRADARHeaderView;
}

namespace data {
		
	class ZFRADARDetection {
		public:
			geom::Location point;
			float velocity;
			float rcs;
			float power;
			float noise;

			ZFRADARDetection() :
				point(0.0f, 0.0f, 0.0f), velocity{ 0.0f }, rcs{ 0.0f }, power{ 0.0f }, noise{ 0.0f } { }
			ZFRADARDetection(float x, float y, float z, float velocity, float rcs, float power, float noise) :
				point(x, y, z), velocity{ velocity }, rcs{ rcs }, power{ power }, noise{ noise } { }
			ZFRADARDetection(geom::Location p, float velocity, float rcs, float power, float noise) :
				point(p), velocity{velocity}, rcs{ rcs }, power{ power }, noise{ noise } { }

			void WritePlyHeaderInfo(std::ostream& out) const {
				out << "property float32 x\n" \
					   "property float32 y\n" \
					   "property float32 z\n" \
					   "property float32 intensity";
			}

			void WriteDetection(std::ostream& out) const {
				out << point.x << ' ' << point.y << ' ' << point.z << ' ' << rcs;
			}
	};

	class ZFRADARData //: public SensorData
	{
	public:
		//explicit ZFRADARData(RawData &&data) : SensorData(std::move(data)) {}
		enum Index : size_t {
			LidarUpdateRate,
			LidarChannels,
			LidarMaxRange,
			LidarAperture,
			SIZE
		};

		explicit ZFRADARData (uint32_t ChannelCount = 0u)
		: _header(Index::SIZE + ChannelCount, 0u){
			_header[Index::LidarChannels] = ChannelCount;
		}

		ZFRADARData &operator=(ZFRADARData &&) = default;

		virtual ~ZFRADARData(){}

		uint32_t GetChannelCount() const {
			return _header[Index::LidarChannels];
		}

		virtual void ResetMemory(std::vector<uint32_t> points_per_channel) {
		DEBUG_ASSERT(GetChannelCount() > points_per_channel.size());
		std::memset(_header.data() + Index::SIZE, 0, sizeof(uint32_t) * GetChannelCount());

		uint32_t total_points = static_cast<uint32_t>(
			std::accumulate(points_per_channel.begin(), points_per_channel.end(), 0));

		_points.clear();
		_points.reserve(total_points * 7);
		}

		void WritePointSync(ZFRADARDetection &detection){
			_points.emplace_back(detection.point.x);
			_points.emplace_back(detection.point.y);
			_points.emplace_back(detection.point.z);
			_points.emplace_back(detection.velocity);
			_points.emplace_back(detection.rcs);
			_points.emplace_back(detection.power);
			_points.emplace_back(detection.noise);
		}

	private:
		std::vector<uint32_t> _header;
		std::vector<float> _points;

		friend class s11n::ZFRADARSerializer;
		friend class s11n::ZFRADARHeaderView;
	};
}
}
}