// Code : UTF - 8
#ifndef ROPO_SENSOR_RASPBERRY_PI_ADAPTER_HPP
#define ROPO_SENSOR_RASPBERRY_PI_ADAPTER_HPP

#include "SerialCore.hpp"
#include "UnionBuffer.hpp"
#include "SafeObject.hpp"
#include <string.h>

namespace RopoSensor {
	static const std::uint8_t HEAD[2] = {0xFC, 0xFD};

	struct Data{
		int a;
		bool c;
		double b;
	};

	class RaspberryPiAdapter : public SerialCore {
	public:
		RaspberryPiAdapter(SerialID _id, std::int32_t _baudrate, int _samplingDelay = 5)
		 : SerialCore(_id, _baudrate, _samplingDelay), message{ }
		{ }
		Data message;
	protected:
		virtual void Update() override
		{
			enum State {
				WAITING_HEAD_0 = 0,
				WAITING_HEAD_1,
				READING,
			} state = WAITING_HEAD_0;
			int cnt = 0;
			std::array<std::uint8_t, sizeof(SafeObject<Data>)> buffer;
			while (Receive.PeekByte() != -1) {
				auto ch = Receive.ReadByte();
				switch (state) {
					case WAITING_HEAD_0 :
						if (ch == HEAD[0]) {
							state = WAITING_HEAD_1;
						}
						break;
					case WAITING_HEAD_1 :
						if (ch == HEAD[1]) {
							state = READING;
							cnt = 0;
						} else {
							state = WAITING_HEAD_0;
						}
						break;
					case READING :
						buffer[cnt++] = ch;
						if (cnt == buffer.size()) {
							UnionBuffer<SafeObject<Data>> unionBuffer(buffer);
							if (unionBuffer.ToObject().CheckSafe()) {
								message = unionBuffer.ToObject().object;
							}
							state = WAITING_HEAD_0;
						}
						break;
				}
			}
		}
	};
}

#endif // ROPO_SENSOR_RASPBERRY_PI_ADAPTER_HPP