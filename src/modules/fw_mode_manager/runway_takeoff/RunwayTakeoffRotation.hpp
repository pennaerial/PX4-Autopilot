/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once

#include <float.h>
#include <cmath>
#include <stdint.h>

namespace runwaytakeoff
{

/** Rotation decision only. No throttle, abort, or wind-estimation behaviour. */
class RunwayTakeoffRotation
{
public:
	struct Config {
		int source{0}; // 0: calibrated airspeed, 1: GPS velocity along the runway
		float airspeed{-1.f};
		float groundspeed{-1.f};
		float confirmation_time{0.5f};
	};

	struct GpsVelocity {
		uint64_t timestamp{0}; // measurement time, not the time this input was read
		float north{NAN};
		float east{NAN};
		float speed_accuracy{NAN};
		uint8_t fix_type{0};
		bool valid{false};
	};

	void reset(const Config &config)
	{
		_config = config;
		_first_above = 0;
		_last_sample = 0;
		_not_before = 0;
	}

	void setAirspeedThreshold(float airspeed) { _config.airspeed = airspeed; }

	void startConfirmation(uint64_t now)
	{
		_first_above = 0;
		_last_sample = 0;
		_not_before = now;
	}

	bool update(uint64_t now, float takeoff_airspeed, float calibrated_airspeed,
		    const GpsVelocity &gps, float runway_bearing)
	{
		if (_config.source == 0) {
			// Preserve the existing threshold and strict comparison in airspeed mode.
			const float threshold = _config.airspeed > FLT_EPSILON
						? (_config.airspeed < takeoff_airspeed ? _config.airspeed : takeoff_airspeed) : 0.9f * takeoff_airspeed;
			return calibrated_airspeed > threshold;
		}

		if (_config.source != 1 || !std::isfinite(_config.groundspeed) || _config.groundspeed <= 0.f
		    || !std::isfinite(_config.confirmation_time) || _config.confirmation_time < 0.f
		    || _config.confirmation_time > 5.f || !std::isfinite(runway_bearing)
		    || !gps.valid || gps.fix_type < 3 || gps.fix_type > 6
		    || !std::isfinite(gps.north) || !std::isfinite(gps.east)
		    || !std::isfinite(gps.speed_accuracy) || gps.speed_accuracy < 0.f || gps.speed_accuracy > 1.f
		    || gps.timestamp == 0 || gps.timestamp < _not_before || gps.timestamp > now || now - gps.timestamp > kMaxSampleAge) {
			_first_above = 0;
			// A rejected future timestamp must not block subsequent valid samples.
			_last_sample = gps.timestamp <= now ? gps.timestamp : 0;
			return false;
		}

		if (gps.timestamp <= _last_sample) {
			if (gps.timestamp < _last_sample) {
				_first_above = 0;
			}

			return false; // Re-reading a sample cannot advance the confirmation timer.
		}

		if (_last_sample != 0 && gps.timestamp - _last_sample > kMaxSampleAge) {
			_first_above = 0;
		}

		_last_sample = gps.timestamp;
		const float forward_speed = gps.north * cosf(runway_bearing) + gps.east * sinf(runway_bearing);

		if (!std::isfinite(forward_speed) || forward_speed <= _config.groundspeed) {
			_first_above = 0;
			return false;
		}

		if (_first_above == 0) {
			_first_above = gps.timestamp;
		}

		const uint64_t confirmation_us = static_cast<uint64_t>(_config.confirmation_time * 1.e6f);
		return gps.timestamp - _first_above >= confirmation_us;
	}

private:
	static constexpr uint64_t kMaxSampleAge = 500000;
	Config _config{};
	uint64_t _first_above{0};
	uint64_t _last_sample{0};
	uint64_t _not_before{0};
};

} // namespace runwaytakeoff
