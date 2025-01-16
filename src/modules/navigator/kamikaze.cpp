#include "kamikaze.h"
#include "navigator.h"
#include <mathlib/mathlib.h>


using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector2d;
using matrix::Vector3f;
using matrix::wrap_pi;

Kamikaze::Kamikaze(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_KAMIKAZE),
	ModuleParams(navigator)
{

}

void Kamikaze::on_activation()
{
	PX4_INFO("Kamikaze mission activated");
}

void Kamikaze::on_active()
{
	PX4_INFO("Kamikaze mission active");
	position_setpoint_triplet_s *reposition_triplet = _navigator->get_reposition_triplet();
	_navigator->reset_position_setpoint(reposition_triplet->previous);
	_navigator->reset_position_setpoint(reposition_triplet->current);
	_navigator->reset_position_setpoint(reposition_triplet->next);

	_navigator->get_mission_result()->finished = true;
	_navigator->set_mission_result_updated();
	_navigator->mode_completed(getNavigatorStateId());

	parameters_update();
}

void Kamikaze::on_inactive()
{
	PX4_INFO("Kamikaze mission inactive");
	parameters_update();
}


void Kamikaze::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}





void Kamikaze::calculate_loiter_position()
{
	Vector2f prev_wp_local = Vector2f(0, 0);
	// Calculate loiter position
}
