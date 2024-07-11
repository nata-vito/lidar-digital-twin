#pragma once

#include "carla/Debug.h"
#include "carla/Memory.h"
#include "carla/sensor/RawData.h"
#include "carla/sensor/data/ZFRADARData.h"

namespace carla {
namespace sensor {

	class SensorData;

namespace s11n {

class ZFRADARHeaderView {
	using Index = data::ZFRADARData::Index;

	public:
		uint32_t GetChannelCount() const{
			return _begin[Index::LidarChannels];
		}

		uint32_t GetPointCount(size_t channel) const {
			DEBUG_ASSERT(channel < GetChannelCount());
			return _begin[Index::SIZE + channel];
		}

	protected:		static ZFRADARHeaderView DeserializeHeader(const RawData &data){

		friend class ZFRADARSerializer;

		explicit ZFRADARHeaderView(const uint32_t *begin) : _begin(begin){
			DEBUG_ASSERT(_begin != nullptr);
		}

		const uint32_t *_begin;
};

class ZFRADARSerializer {
	public:

		static ZFRADARHeaderView DeserializeHeader(const RawData &data){
			return ZFRADARHeaderView{reinterpret_cast<const uint32_t*>(data.begin())};
		}

		static size_t GetHeaderOffset(const RawData &data){
			auto View = DeserializeHeader(data);
			return sizeof(uint32_t) * (View.GetChannelCount() + data::ZFRADARData::Index::SIZE);
		}

		template <typename Sensor>
		static Buffer Serialize(
			const Sensor &sensor,
			const data::ZFRADARData &data,
			Buffer &&output);

		static SharedPtr<SensorData> Deserialize(RawData &&data);
		/*template <typename Sensor, typename Float>
		static Buffer Serialize(
			const Sensor &,
			std::vector<float> data)
		{
			return std::move(data);
		}

		static SharedPtr<SensorData> Deserialize(RawData &&data);*/
	};	

	template <typename Sensor>
	inline Buffer ZFRADARSerializer::Serialize(
		const Sensor &,
		const data::ZFRADARData &data,
		Buffer &&output) {
			std::array<boost::asio::const_buffer, 2u> seq = {
				boost::asio::buffer(data._header),
				boost::asio::buffer(data._points)};
			output.copy_from(seq);
			return std::move(output);
		}
}
}
}