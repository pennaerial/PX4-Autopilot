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

#include <cmath>
#include <stdint.h>

namespace runwaytakeoff
{

// One-shot permission to start a runway sequence despite VTOL's armed FW land report.
// This does not change global land detection or command throttle, pitch, or an abort.
class VtolRunwayStart
{
public:
	struct Sample {
		uint64_t timestamp{0};
		float x{0.f}, y{0.f}, z{0.f};
		float vx{0.f}, vy{0.f}, vz{0.f};
		uint8_t xy_reset{0}, z_reset{0}, vxy_reset{0}, vz_reset{0};
		bool valid{false};
	};

	void update(uint64_t now, bool enabled, bool armed, bool fixed_wing, bool transition,
		    bool excluded_mode, uint64_t land_timestamp, bool landed, const Sample &sample)
	{
		const bool fresh = sample.timestamp > 0 && sample.timestamp <= now
				   && now - sample.timestamp <= 500000;
		const bool finite = std::isfinite(sample.x) && std::isfinite(sample.y) && std::isfinite(sample.z)
				    && std::isfinite(sample.vx) && std::isfinite(sample.vy) && std::isfinite(sample.vz);
		const bool stationary = finite && sample.vx * sample.vx + sample.vy * sample.vy <= 1.f
					&& std::fabs(sample.vz) <= 0.5f;
		const bool continuous = _last_sample == 0 || (sample.timestamp >= _last_sample
					&& sample.timestamp - _last_sample <= 500000);
		const bool unchanged_frame = _last_sample == 0 || (sample.xy_reset == _anchor.xy_reset
					     && sample.z_reset == _anchor.z_reset && sample.vxy_reset == _anchor.vxy_reset
					     && sample.vz_reset == _anchor.vz_reset);
		const float dx = sample.x - _anchor.x;
		const float dy = sample.y - _anchor.y;
		const bool near_anchor = _last_sample == 0 || (dx * dx + dy * dy <= 25.f
					 && std::fabs(sample.z - _anchor.z) <= 1.f);

		if (!enabled || transition || excluded_mode || !fresh || !sample.valid || !stationary
		    || !continuous || !unchanged_frame || !near_anchor || (armed && !fixed_wing)) {
			reset();
			return;
		}

		if (!armed) {
			// Stable land-detector state is published at 1 Hz.
			if (!landed || land_timestamp == 0 || land_timestamp > now || now - land_timestamp > 2000000) {
				reset();
				return;
			}

			if (_last_sample == 0 || _was_armed) {
				_anchor = sample;
				_start = sample.timestamp;
				_ready = false;
			}

			_ready = sample.timestamp - _start >= 1000000;
		}

		// Arming alone must never create qualification.
		_last_sample = sample.timestamp;
		_was_armed = armed;
	}

	bool consume()
	{
		const bool ready = _ready && _was_armed;
		_ready = false;
		return ready;
	}

	void reset()
	{
		_start = 0;
		_last_sample = 0;
		_ready = false;
		_was_armed = false;
		_anchor = {};
	}

private:
	Sample _anchor{};
	uint64_t _start{0};
	uint64_t _last_sample{0};
	bool _ready{false};
	bool _was_armed{false};
};

} // namespace runwaytakeoff
