//
// Created by roman on 12.12.24.
//
#include "AirspeedReferenceController.hpp"
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

using matrix::Vector2f;
AirspeedReferenceController::AirspeedReferenceController()
{
	// Constructor
}

float AirspeedReferenceController::controlHeading(const float heading_sp, const float heading,
		const float airspeed) const
{

	const Vector2f airspeed_vector = Vector2f{cosf(heading), sinf(heading)} * airspeed;
	const Vector2f airspeed_sp_vector_unit = Vector2f{cosf(heading_sp), sinf(heading_sp)};

	const float dot_air_vel_err = airspeed_vector.dot(airspeed_sp_vector_unit);
	const float cross_air_vel_err = airspeed_vector.cross(airspeed_sp_vector_unit);

	if (dot_air_vel_err < 0.0f) {
		// hold max lateral acceleration command above 90 deg heading error
		return p_gain_ * ((cross_air_vel_err < 0.0f) ? -airspeed : airspeed);

	} else {
		// airspeed/airspeed_ref is used to scale any incremented airspeed reference back to the current airspeed
		// for acceleration commands in a "feedback" sense (i.e. at the current vehicle airspeed)
		return p_gain_ * cross_air_vel_err;
	}
}
