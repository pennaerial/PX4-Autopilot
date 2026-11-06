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
#include <gtest/gtest.h>
#include "VtolRunwayStart.hpp"

using runwaytakeoff::VtolRunwayStart;

class VtolRunwayStartTest : public ::testing::Test
{
protected:
	VtolRunwayStart start;
	VtolRunwayStart::Sample sample{1000000, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0, 0, 0, 0, true};
	uint64_t now{1000000};

	void update(bool armed = false, bool fixed_wing = true, bool transition = false,
		    bool excluded = false, bool enabled = true, bool landed = true)
	{
		start.update(now, enabled, armed, fixed_wing, transition, excluded, now, landed, sample);
	}

	void advance()
	{
		now += 100000;
		sample.timestamp = now;
	}

	void qualify()
	{
		for (int i = 0; i <= 10; ++i) {
			update();
			advance();
		}
	}
};

TEST_F(VtolRunwayStartTest, PreservesGroundStartAcrossArmingButOnlyOnce)
{
	qualify();
	update(true, true, false, false, true, false);
	EXPECT_TRUE(start.consume());
	EXPECT_FALSE(start.consume());
	advance();
	update(true);
	EXPECT_FALSE(start.consume());
}

TEST_F(VtolRunwayStartTest, PreselectedDisarmedTakeoffDoesNotConsume)
{
	qualify();
	// The mode manager must not consume until armed.
	advance();
	update(true);
	EXPECT_TRUE(start.consume());
}

TEST_F(VtolRunwayStartTest, AlreadyArmedNeverQualifies)
{
	for (int i = 0; i < 30; ++i) {
		update(true);
		advance();
	}

	EXPECT_FALSE(start.consume());
}

TEST_F(VtolRunwayStartTest, RequiresFullSecond)
{
	for (int i = 0; i < 10; ++i) {
		update();
		advance();
	}

	update(true);
	EXPECT_FALSE(start.consume());
}

TEST_F(VtolRunwayStartTest, StationaryManualArmingCanWaitForTakeoff)
{
	qualify();

	for (int i = 0; i < 50; ++i) {
		update(true);
		advance();
	}

	EXPECT_TRUE(start.consume());
}

TEST_F(VtolRunwayStartTest, MotionAfterArmingCannotRequalify)
{
	qualify();
	sample.vx = 2.f;
	update(true);
	sample.vx = 0.f;

	for (int i = 0; i < 30; ++i) {
		advance();
		update(true);
	}

	EXPECT_FALSE(start.consume());
}

TEST_F(VtolRunwayStartTest, RejectsDisplacementOrHeightChange)
{
	for (int axis = 0; axis < 2; ++axis) {
		start.reset();
		sample.x = sample.z = 0.f;
		qualify();

		if (axis == 0) { sample.x = 6.f; } else { sample.z = -1.1f; }

		update(true);
		EXPECT_FALSE(start.consume());
	}
}

TEST_F(VtolRunwayStartTest, RejectsEveryEstimatorReset)
{
	for (int field = 0; field < 4; ++field) {
		start.reset();
		qualify();

		switch (field) {
		case 0: ++sample.xy_reset; break;

		case 1: ++sample.z_reset; break;

		case 2: ++sample.vxy_reset; break;

		case 3: ++sample.vz_reset; break;
		}

		update(true);
		EXPECT_FALSE(start.consume());
	}
}

TEST_F(VtolRunwayStartTest, RejectsTransitionRotaryWingOffboardAndDisabledRunway)
{
	for (int mode = 0; mode < 4; ++mode) {
		start.reset();
		qualify();
		update(true, mode != 0, mode == 1, mode == 2, mode != 3);
		EXPECT_FALSE(start.consume());
	}
}

TEST_F(VtolRunwayStartTest, ExternalModeResetDiscardsQualification)
{
	qualify();
	start.reset();
	update(true);
	EXPECT_FALSE(start.consume());
}

TEST_F(VtolRunwayStartTest, RejectsStaleFutureInvalidAndNonfinitePosition)
{
	for (int fault = 0; fault < 4; ++fault) {
		start.reset();
		sample.valid = true;
		sample.vz = 0.f;
		sample.timestamp = now;
		qualify();

		if (fault == 0) { sample.timestamp = now - 500001; }

		if (fault == 1) { sample.timestamp = now + 1; }

		if (fault == 2) { sample.valid = false; }

		if (fault == 3) { sample.vz = NAN; }

		update(true);
		EXPECT_FALSE(start.consume());
	}
}

TEST_F(VtolRunwayStartTest, SampleGapInvalidatesArmedQualification)
{
	qualify();
	now += 600000;
	sample.timestamp = now;
	update(true);
	EXPECT_FALSE(start.consume());
}

TEST_F(VtolRunwayStartTest, DefaultOrStaleLandedCannotQualify)
{
	for (int i = 0; i < 20; ++i) {
		start.update(now, true, false, true, false, false, 0, true, sample);
		advance();
	}

	update(true);
	EXPECT_FALSE(start.consume());
}

TEST_F(VtolRunwayStartTest, RearmRequiresFreshQualification)
{
	qualify();
	update(true);
	EXPECT_TRUE(start.consume());
	advance();
	update(false);
	advance();
	update(true);
	EXPECT_FALSE(start.consume());
	advance();
	qualify();
	update(true);
	EXPECT_TRUE(start.consume());
}

TEST_F(VtolRunwayStartTest, AcceptsNormalOneHertzLandDetectorPublication)
{
	const uint64_t first_land_timestamp = now;

	for (int i = 0; i <= 15; ++i) {
		const uint64_t land_timestamp = first_land_timestamp + (i >= 10 ? 1000000 : 0);
		start.update(now, true, false, true, false, false, land_timestamp, true, sample);
		advance();
	}

	update(true);
	EXPECT_TRUE(start.consume());
}

TEST_F(VtolRunwayStartTest, RejectsOldLandDetectorPublication)
{
	qualify();
	now += 2100000;
	sample.timestamp = now;
	start.update(now, true, false, true, false, false, now - 2000001, true, sample);
	advance();
	update(true);
	EXPECT_FALSE(start.consume());
}

TEST_F(VtolRunwayStartTest, FutureLandTimestampCannotQualify)
{
	for (int i = 0; i < 20; ++i) {
		start.update(now, true, false, true, false, false, now + 1, true, sample);
		advance();
	}

	update(true);
	EXPECT_FALSE(start.consume());
}

TEST_F(VtolRunwayStartTest, DuplicatePositionSamplesCannotAdvanceQualification)
{
	for (int i = 0; i < 20; ++i) {
		update();
	}

	update(true);
	EXPECT_FALSE(start.consume());
}
