/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
/**
 * @file kamikaze.cpp
 *
 * Kamikaze mission for a given target position. This mode developed for one of the mission in TEKNOFEST SAVAŞAN İHA UAV Competition
 * held in Türkiye. So applied logic specific for competition conditions. Detailed information for the competition and kamikaze mission
 *
 * https://cdn.teknofest.org/media/upload/userFormUpload/2025_SAVASAN_%C4%B0HA_YARISMA_SARTNAMES%C4%B0_TR_nbdHb.pdf
 * For up-do-date information please check the official website of the competition.
 * https://teknofest.org/tr/competitions/competition/33
 */

#pragma once
#include "navigator_mode.h"
#include "mission_block.h"
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
using namespace time_literals;

class Kamikaze : public	MissionBlock , public ModuleParams
{
public:
	Kamikaze(Navigator *navigator);
	~Kamikaze() = default;
	void on_activation() override;
	void on_active() override;
	void on_inactive() override;

private:

	/**
	 * @brief Calculate loiter position
	 * Calculate loiter position for the kamikaaze mission
	 */
	void calculate_loiter_position();


	/**
	 * @brief Parameters update
	 *
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	float _kkz_qr_lat = _param_kkz_qr_lat.get(); /**< QR code latitude */
	float _kkz_qr_lon = _param_kkz_qr_lon.get(); /**< QR code longitude */
	float _kkz_loiter_rad = _param_kkz_loiter_rad.get(); /**< QR code loiter radius */
	float _kkz_approach_distance = _param_kkz_approach_distance.get(); /**< Approach distance to the QR code */
	float _kkz_approach_angle = _param_kkz_approach_angle.get(); /**< Approach angle to the QR code */

};
