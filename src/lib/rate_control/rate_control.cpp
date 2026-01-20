/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file rate_control.cpp
 */

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}

void RateControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

void RateControl::setIstaEnabled(bool enabled)
{
	if (_ista_enabled != enabled) {
		_rate_int.zero();
		_ista_enabled = enabled;
	}
}

void RateControl::setIstaEpsilon(float epsilon)
{
	_ista_epsilon = math::max(epsilon, 0.f);
}

void RateControl::setIstaDeadband(float deadband)
{
	_ista_deadband = math::max(deadband, 0.f);
}

void RateControl::setIstaNuTimeConstant(float time_constant)
{
	_ista_nu_tc = math::max(time_constant, 0.f);
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	if (!PX4_ISFINITE(dt) || dt <= 0.f) {
		return Vector3f();
	}

	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	Vector3f torque = -_gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	if (_ista_enabled) {
		for (int i = 0; i < 3; i++) {
			const bool saturated_positive = _control_allocator_saturation_positive(i);
			const bool saturated_negative = _control_allocator_saturation_negative(i);
			const bool inhibit_update = (saturated_positive && rate_error(i) > 0.f)
						    || (saturated_negative && rate_error(i) < 0.f);
			const bool update_state = !landed && !inhibit_update;

			if (_ista_deadband > 0.f
			    && fabsf(rate_sp(i)) < _ista_deadband
			    && fabsf(rate_error(i)) < _ista_deadband) {
				if (update_state && _ista_nu_tc > 0.f) {
					const float decay = math::constrain(dt / _ista_nu_tc, 0.f, 1.f);
					_rate_int(i) -= _rate_int(i) * decay;
				}

				continue;
			}

			const float u = updateIstaAxis(-rate_error(i), dt, _gain_p(i), _gain_i(i), _rate_int(i), update_state);

			if (update_state) {
				_rate_int(i) = math::constrain(_rate_int(i), -_lim_int(i), _lim_int(i));
			}

			torque(i) += u;
		}

	} else {
		torque += _gain_p.emult(rate_error) + _rate_int;

		// update integral only if we are not landed
		if (!landed) {
			updateIntegral(rate_error, dt);
		}
	}

	return torque;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

float RateControl::updateIstaAxis(const float x, const float h, const float lambda1, const float lambda2,
				  float &nu, const bool update_state)
{
	if (!PX4_ISFINITE(x) || !PX4_ISFINITE(h) || h <= 0.f) {
		return 0.f;
	}

	float x_eff = x;
	if (_ista_epsilon > 0.f) {
		const float abs_x = fabsf(x);
		x_eff = (abs_x > 0.f) ? (x * abs_x / (abs_x + _ista_epsilon)) : 0.f;
	}

	const float lambda1_safe = math::max(lambda1, 0.f);
	const float lambda2_safe = math::max(lambda2, 0.f);

	if (lambda1_safe <= 0.f && lambda2_safe <= 0.f) {
		if (update_state) {
			nu = 0.f;
		}
		return 0.f;
	}

	const float a = h * lambda1_safe;
	const float b_k = -x_eff - h * nu;
	const float h2_lambda2 = h * h * lambda2_safe;

	float u = 0.f;

	if (b_k < -h2_lambda2) {
		const float discriminant = a * a - 4.f * (b_k + h2_lambda2);
		const float sqrt_disc = (discriminant > 0.f) ? sqrtf(discriminant) : 0.f;
		const float sqrt_x_tilde = (-a + sqrt_disc) * 0.5f;
		const float sqrt_x_tilde_safe = (sqrt_x_tilde > 0.f) ? sqrt_x_tilde : 0.f;

		const float nu_next = update_state ? (nu - h * lambda2_safe) : nu;
		if (update_state) {
			nu = nu_next;
		}

		u = -lambda1_safe * sqrt_x_tilde_safe + nu_next;

	} else if (b_k > h2_lambda2) {
		const float discriminant = a * a + 4.f * (b_k - h2_lambda2);
		const float sqrt_disc = (discriminant > 0.f) ? sqrtf(discriminant) : 0.f;
		const float sqrt_x_tilde = (-a + sqrt_disc) * 0.5f;
		const float sqrt_x_tilde_safe = (sqrt_x_tilde > 0.f) ? sqrt_x_tilde : 0.f;

		const float nu_next = update_state ? (nu + h * lambda2_safe) : nu;
		if (update_state) {
			nu = nu_next;
		}

		u = lambda1_safe * sqrt_x_tilde_safe + nu_next;

	} else {
		u = -x_eff / h;
		if (update_state) {
			nu = u;
		}
	}

	if (!PX4_ISFINITE(u)) {
		u = 0.f;
		if (update_state) {
			nu = 0.f;
		}
	}

	if (update_state && !PX4_ISFINITE(nu)) {
		nu = 0.f;
	}

	return u;
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
