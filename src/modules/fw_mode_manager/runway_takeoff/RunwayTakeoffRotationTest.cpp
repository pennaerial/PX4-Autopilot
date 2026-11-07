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
#include "RunwayTakeoffRotation.hpp"

using runwaytakeoff::RunwayTakeoffRotation;

namespace
{
RunwayTakeoffRotation::GpsVelocity sample(uint64_t timestamp, float north = 20.f, float east = 0.f)
{
	return {timestamp, north, east, 0.2f, 3, true};
}

RunwayTakeoffRotation gpsRotation(float duration = 0.5f)
{
	RunwayTakeoffRotation rotation;
	rotation.reset({1, -1.f, 18.f, duration});
	return rotation;
}
}

TEST(RunwayTakeoffRotation, DefaultAirspeedDoesNotNeedGps)
{
	RunwayTakeoffRotation rotation;
	EXPECT_FALSE(rotation.update(1000000, 20.f, 18.f, {}, NAN));
	EXPECT_TRUE(rotation.update(1000000, 20.f, 18.1f, {}, NAN));
	EXPECT_FALSE(rotation.update(1000000, 20.f, NAN, sample(1000000, 100.f), 0.f));
}

TEST(RunwayTakeoffRotation, ExplicitAirspeedThresholdIsCappedAtTakeoffSpeed)
{
	RunwayTakeoffRotation rotation;
	rotation.reset({0, 25.f, -1.f, 0.5f});
	EXPECT_FALSE(rotation.update(1000000, 20.f, 20.f, {}, NAN));
	EXPECT_TRUE(rotation.update(1000000, 20.f, 20.1f, {}, NAN));
	rotation.reset({0, 15.f, -1.f, 0.5f});
	EXPECT_TRUE(rotation.update(1000000, 20.f, 15.1f, {}, NAN));
}

TEST(RunwayTakeoffRotation, ConfirmsFreshGpsWithoutAnyAirspeed)
{
	auto rotation = gpsRotation();

	for (uint64_t t = 1000000; t < 1500000; t += 100000) {
		EXPECT_FALSE(rotation.update(t, NAN, NAN, sample(t), 0.f));
	}

	EXPECT_TRUE(rotation.update(1500000, NAN, NAN, sample(1500000), 0.f));
}

TEST(RunwayTakeoffRotation, AirspeedCannotTriggerGpsMode)
{
	auto rotation = gpsRotation(0.f);
	EXPECT_FALSE(rotation.update(1000000, 20.f, 100.f, {}, 0.f));
	EXPECT_FALSE(rotation.update(1100000, 20.f, 100.f, sample(1100000, 18.f), 0.f));
	EXPECT_TRUE(rotation.update(1200000, 20.f, NAN, sample(1200000, 18.1f), 0.f));
}

TEST(RunwayTakeoffRotation, SpikeAndDropResetConfirmation)
{
	auto rotation = gpsRotation();
	EXPECT_FALSE(rotation.update(1000000, NAN, NAN, sample(1000000, 40.f), 0.f));
	EXPECT_FALSE(rotation.update(1100000, NAN, NAN, sample(1100000, 18.f), 0.f));
	EXPECT_FALSE(rotation.update(1200000, NAN, NAN, sample(1200000), 0.f));
	EXPECT_FALSE(rotation.update(1500000, NAN, NAN, sample(1500000), 0.f));
	EXPECT_TRUE(rotation.update(1700000, NAN, NAN, sample(1700000), 0.f));
}

TEST(RunwayTakeoffRotation, DuplicateSamplesCannotAdvanceTime)
{
	auto rotation = gpsRotation();
	const auto gps = sample(1000000);
	EXPECT_FALSE(rotation.update(1000000, NAN, NAN, gps, 0.f));
	EXPECT_FALSE(rotation.update(1500000, NAN, NAN, gps, 0.f));
	EXPECT_FALSE(rotation.update(1500001, NAN, NAN, gps, 0.f));
	EXPECT_FALSE(rotation.update(1600000, NAN, NAN, sample(1600000), 0.f));
}

TEST(RunwayTakeoffRotation, GapAndOutOfOrderSamplesResetConfirmation)
{
	auto rotation = gpsRotation();
	EXPECT_FALSE(rotation.update(1000000, NAN, NAN, sample(1000000), 0.f));
	EXPECT_FALSE(rotation.update(1600000, NAN, NAN, sample(1600000), 0.f));
	EXPECT_FALSE(rotation.update(1700000, NAN, NAN, sample(1500000), 0.f));
	EXPECT_FALSE(rotation.update(1800000, NAN, NAN, sample(1800000), 0.f));
	EXPECT_TRUE(rotation.update(2300000, NAN, NAN, sample(2300000), 0.f));
}

TEST(RunwayTakeoffRotation, RejectsOldFutureAndMissingTimestamps)
{
	auto rotation = gpsRotation(0.f);
	EXPECT_FALSE(rotation.update(1600000, NAN, NAN, sample(1000000), 0.f));
	EXPECT_FALSE(rotation.update(1600000, NAN, NAN, sample(1700000), 0.f));
	EXPECT_FALSE(rotation.update(1600000, NAN, NAN, sample(0), 0.f));
}

TEST(RunwayTakeoffRotation, RequiresThreeDimensionalFixAndValidVelocity)
{
	for (uint8_t fix : {0, 1, 2, 7, 8}) {
		auto rotation = gpsRotation(0.f);
		auto gps = sample(1000000);
		gps.fix_type = fix;
		EXPECT_FALSE(rotation.update(1000000, NAN, NAN, gps, 0.f));
	}

	for (uint8_t fix : {3, 4, 5, 6}) {
		auto rotation = gpsRotation(0.f);
		auto gps = sample(1000000);
		gps.fix_type = fix;
		EXPECT_TRUE(rotation.update(1000000, NAN, NAN, gps, 0.f));
	}
	auto rotation = gpsRotation(0.f);
	auto gps = sample(1000000);
	gps.valid = false;
	EXPECT_FALSE(rotation.update(1000000, NAN, NAN, gps, 0.f));
}

TEST(RunwayTakeoffRotation, RejectsPoorAccuracyAndNonFiniteData)
{
	for (float accuracy : {-1.f, 1.01f, NAN, INFINITY}) {
		auto rotation = gpsRotation(0.f);
		auto gps = sample(1000000);
		gps.speed_accuracy = accuracy;
		EXPECT_FALSE(rotation.update(1000000, NAN, NAN, gps, 0.f));
	}

	for (float invalid : {NAN, INFINITY, -INFINITY}) {
		auto rotation = gpsRotation(0.f);
		EXPECT_FALSE(rotation.update(1000000, NAN, NAN, sample(1000000, invalid), 0.f));
		EXPECT_FALSE(rotation.update(1100000, NAN, NAN, sample(1100000, 20.f, invalid), 0.f));
		EXPECT_FALSE(rotation.update(1200000, NAN, NAN, sample(1200000), invalid));
	}
}

TEST(RunwayTakeoffRotation, InvalidSampleRestartsConfirmation)
{
	auto rotation = gpsRotation();
	EXPECT_FALSE(rotation.update(1000000, NAN, NAN, sample(1000000), 0.f));
	auto gps = sample(1200000);
	gps.speed_accuracy = 2.f;
	EXPECT_FALSE(rotation.update(1200000, NAN, NAN, gps, 0.f));
	EXPECT_FALSE(rotation.update(1300000, NAN, NAN, sample(1300000), 0.f));
	EXPECT_FALSE(rotation.update(1500000, NAN, NAN, sample(1500000), 0.f));
	EXPECT_TRUE(rotation.update(1800000, NAN, NAN, sample(1800000), 0.f));
}

TEST(RunwayTakeoffRotation, UsesForwardProjectionNotHorizontalMagnitude)
{
	auto rotation = gpsRotation(0.f);
	EXPECT_FALSE(rotation.update(1000000, NAN, NAN, sample(1000000, -30.f), 0.f));
	EXPECT_FALSE(rotation.update(1100000, NAN, NAN, sample(1100000, 0.f, 30.f), 0.f));
	EXPECT_FALSE(rotation.update(1200000, NAN, NAN, sample(1200000, 17.f, 30.f), 0.f));
	EXPECT_TRUE(rotation.update(1300000, NAN, NAN, sample(1300000, 0.f, 20.f), 1.57079632679f));
}

TEST(RunwayTakeoffRotation, UnsetOrInvalidConfigurationCannotRotate)
{
	for (float speed : {-1.f, 0.f, NAN, INFINITY}) {
		RunwayTakeoffRotation rotation;
		rotation.reset({1, -1.f, speed, 0.f});
		EXPECT_FALSE(rotation.update(1000000, NAN, NAN, sample(1000000), 0.f));
	}

	for (float duration : {-1.f, 5.1f, NAN, INFINITY}) {
		auto rotation = gpsRotation(duration);
		EXPECT_FALSE(rotation.update(1000000, NAN, NAN, sample(1000000), 0.f));
	}
	RunwayTakeoffRotation rotation;
	rotation.reset({2, -1.f, 18.f, 0.f});
	EXPECT_FALSE(rotation.update(1000000, 20.f, 100.f, sample(1000000), 0.f));
}

TEST(RunwayTakeoffRotation, ResetDoesNotCarryConfirmationIntoNextAttempt)
{
	auto rotation = gpsRotation();
	EXPECT_FALSE(rotation.update(1000000, NAN, NAN, sample(1000000), 0.f));
	EXPECT_TRUE(rotation.update(1500000, NAN, NAN, sample(1500000), 0.f));
	rotation.reset({1, -1.f, 18.f, 0.5f});
	EXPECT_FALSE(rotation.update(1600000, NAN, NAN, sample(1600000), 0.f));
}

TEST(RunwayTakeoffRotation, ConfirmationCannotIncludeThrottleRampSamples)
{
	auto rotation = gpsRotation();
	rotation.startConfirmation(1500000);
	EXPECT_FALSE(rotation.update(1500000, NAN, NAN, sample(1400000), 0.f));
	EXPECT_FALSE(rotation.update(1600000, NAN, NAN, sample(1600000), 0.f));
	EXPECT_FALSE(rotation.update(1900000, NAN, NAN, sample(1900000), 0.f));
	EXPECT_TRUE(rotation.update(2100000, NAN, NAN, sample(2100000), 0.f));
}

TEST(RunwayTakeoffRotation, ExistingAirspeedThresholdCanStillBeUpdated)
{
	RunwayTakeoffRotation rotation;
	EXPECT_FALSE(rotation.update(1000000, 20.f, 17.f, {}, NAN));
	rotation.setAirspeedThreshold(16.f);
	EXPECT_TRUE(rotation.update(1100000, 20.f, 17.f, {}, NAN));
}

TEST(RunwayTakeoffRotation, RecoversAfterFutureTimestampWithFreshConfirmation)
{
	auto rotation = gpsRotation();
	EXPECT_FALSE(rotation.update(1000000, NAN, NAN, sample(1000000), 0.f));
	EXPECT_FALSE(rotation.update(1200000, NAN, NAN, sample(UINT64_MAX), 0.f));
	EXPECT_FALSE(rotation.update(1300000, NAN, NAN, sample(1300000), 0.f));
	EXPECT_FALSE(rotation.update(1500000, NAN, NAN, sample(1500000), 0.f));
	EXPECT_TRUE(rotation.update(1800000, NAN, NAN, sample(1800000), 0.f));
}
