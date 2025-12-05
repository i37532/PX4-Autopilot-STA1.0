/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

#include "ISTA.hpp"
// ISTA _z_controller{10.0f, 6.0f}; // λ₁, λ₂
// Lambda1 和 Lambda2 需要匹配滑模面动力学
// 原来: ISTA _z_controller{10.0f, 6.0f};
// 建议尝试更温和的参数开始调试，或者根据 Super-Twisting 标准公式: L1 = 1.5*sqrt(L2), L2 = 1.1*U_max


// 改成：
ISTA _x_controller{2.0f, 4.0f};   // 参数先和 Z 一样，后面慢慢调
ISTA _y_controller{2.0f, 4.0f};
ISTA _z_controller{2.0f, 4.0f};


using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);

	// _super_twisting._set_sta_w((_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
	// 		+ CONSTANTS_ONE_G - _acc_sp(2));
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}




bool PositionControl::update(const float dt)
{
    if (!_inputValid()) {
        return false;
    }

    // 1) 原生位置环
    _positionControl();


// 2) 非 LAND 模式：用 ISTA 给 Z 轴加一个“有限的前馈加速度”
if (!_is_landing) {

	// 注意：这里我建议改成 pos - pos_sp，这样“在目标高度以下”误差为正，便于调试符号
	float z_pos_error = 0.0f;
	if (PX4_ISFINITE(_pos_sp(2)) && PX4_ISFINITE(_pos(2))) {
		z_pos_error = _pos(2) - _pos_sp(2);   // 改：原来是 _pos_sp(2) - _pos(2)
	}

	float z_vel_error = 0.0f;
	if (PX4_ISFINITE(_vel_sp(2)) && PX4_ISFINITE(_vel(2))) {
		z_vel_error = _vel(2) - _vel_sp(2);   // 同理，建议也统一成状态 - 期望
	}

	const float beta = 2.0f;
	const float sliding_surface = beta * z_pos_error + z_vel_error;

	// 1) 先算 ISTA 输出
	float ista_z_output = _z_controller.update(sliding_surface, dt);

	// 2) 做一个幅值限幅，避免一下子把 thrust 拉满
	const float ista_limit = 1.0f;        // 先从 ±2 m/s^2 这种比较小的值开始
	ista_z_output = math::constrain(ista_z_output, -ista_limit, ista_limit);

	// 3) 作为一个“微调项”叠加到 Z 加速度上，而不是完全覆盖
	//    注意：这里先不要加负号，保持最直观的关系
	_acc_sp(2) += ista_z_output;





// --- X 轴 ISTA：先只改速度期望，不直接改加速度 ---

float x_pos_error = 0.0f;
if (PX4_ISFINITE(_pos_sp(0)) && PX4_ISFINITE(_pos(0))) {
    x_pos_error = _pos(0) - _pos_sp(0);     // 和 Z 轴一样，用 “状态 - 期望”
}

float x_vel_error = 0.0f;
if (PX4_ISFINITE(_vel_sp(0)) && PX4_ISFINITE(_vel(0))) {
    x_vel_error = _vel(0) - _vel_sp(0);
}

// 构造 X 轴的滑模面
const float beta_xy = 1.0f;
const float s_x = beta_xy * x_pos_error + x_vel_error;

// ISTA 输出，这里我们把它当成“额外的速度修正量”
float ista_x_vel = _x_controller.update(-s_x, dt);

// 给一个比较小的限幅，避免速度指令乱跳
const float ista_vel_limit_xy = 0.15f;   // 单位 m/s，先从 ±0.3 开始试
ista_x_vel = math::constrain(ista_x_vel, -ista_vel_limit_xy, ista_vel_limit_xy);

// 叠加到速度期望，而不是加速度期望
_vel_sp(0) += ista_x_vel;




// Y 轴误差
float y_pos_error = 0.0f;
if (PX4_ISFINITE(_pos_sp(1)) && PX4_ISFINITE(_pos(1))) {
    y_pos_error = _pos(1) - _pos_sp(1);
}

float y_vel_error = 0.0f;
if (PX4_ISFINITE(_vel_sp(1)) && PX4_ISFINITE(_vel(1))) {
    y_vel_error = _vel(1) - _vel_sp(1);
}

const float s_y = beta_xy * y_pos_error + y_vel_error;
float ista_y_vel = _y_controller.update(-s_y, dt);
ista_y_vel = math::constrain(ista_y_vel, -ista_vel_limit_xy, ista_vel_limit_xy);
_vel_sp(1) += ista_y_vel;



} else {
// LAND 模式：完全不用 ISTA
// 如果你在 ISTA 里加了 reset，这里可以清一下内部状态
// _z_controller.reset();
}




    // 4) 速度环（原生）
    _velocityControl(dt);

    // 5) yaw/sp 处理 & 输出检查
    _yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
    _yaw_sp      = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;

    return _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}







void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN


	// acc_sp_velocity(2) = -_super_twisting._getStaThrust();


	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.
	if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
		const float arw_gain = 2.f / _gain_vel_p(0);
		const Vector2f acc_sp_xy = _acc_sp.xy();

		vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
	}

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}

// void PositionControl::_set_sta_param(float sta_sliding_c_new, float sta_z_error_up_new, float sta_ita_norm_up_new)
// {
// 	_super_twisting._update_sta_sliding_c(sta_sliding_c_new);
// 	_super_twisting._update_sta_sliding_z_error_up(sta_z_error_up_new);
// 	_super_twisting._update_sta_ita_norm_up(sta_ita_norm_up_new);
// }
