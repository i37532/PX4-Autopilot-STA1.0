#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <geo/geo.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/sta_status.h>

#include "ISTA.hpp"



class SuperTwisting
{
private:


    	ISTA _ista_z{0.0f, 0.0f};   // z 轴的隐式 supertwisting 控制器

	/* data */
	float _mc_mass = 1.5;
	float _sta_sliding_c = 1.2;
	float _sta_C = 2 * CONSTANTS_ONE_G;
	float _sta_UM = 3 * CONSTANTS_ONE_G;
	// float _sta_alpha = (float)8.5 * _mc_mass * CONSTANTS_ONE_G / 150;
	// float _sta_lamada = (float)77.4 / 1000000;
	float _sta_alpha = (float)2.0;
	float _sta_lamada = (float)4.0;
	float _sta_w_dot = 0.0;
	float _sta_w = 0.0;

	float _sta_thrust = 0.0;
	float _sta_z_pos_error = 0;
	float _sta_deri_z_pos_error = 0;
	float _sta_ita = 0;
	float _last_z_pos = 0.0;

	float _sta_z_pos_error_up = 0.4;
	float _sta_ita_norm_up = 0.4;

	float _sta_int_z_pos_error = 0.0;
	float _sta_last_one_pos_sp = 0.0;
	float _sta_last_two_pos_sp = 0.0;
	float _sta_last_three_pos_sp = 0.0;
	uORB::Publication<sta_status_s> _publish_sta_status{ORB_ID(sta_status)};

	sta_status_s _sta_status;
	// uORB::Publication<sta_msg_s>  _publish_sta_msg{ORB_ID(sta_msg)};
public:
	SuperTwisting(/* args */);
	~SuperTwisting();

	void _staZPositionControl(const float dt, matrix::Vector3f& _pos, matrix::Vector3f& _pos_sp, matrix::Vector3f& _vel, matrix::Vector3f& _vel_sp, matrix::Vector3f& _acc_sp);
	void _set_sta_w(float less_num);
	float _getStaThrust();
	void _update_sta_sliding_c(float new_value);
	void _update_sta_sliding_z_error_up(float new_value);
	void _update_sta_ita_norm_up(float new_value);



};


