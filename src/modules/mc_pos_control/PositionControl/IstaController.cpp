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
 * @file IstaController.cpp
 *
 * Implicit Super-Twisting Algorithm (ISTA) controller implementation.
 * Based on BBSTA paper: "The implicit discretization of the super-twisting
 * sliding-mode control algorithm", Section III.
 */

#include "IstaController.hpp"

float IstaController::update(float x, float h)
{
	// Safety: NaN input check
	if (!PX4_ISFINITE(x) || !PX4_ISFINITE(h) || h <= 0.0f) {
		_last_case = 0;
		return NAN;
	}

	// Ensure gains are positive (defensive)
	const float lambda1 = (_lambda1 > 0.0f) ? _lambda1 : 1.0f;
	const float lambda2 = (_lambda2 > 0.0f) ? _lambda2 : 1.0f;

	// Boundary layer on x to reduce chattering near zero.
	float x_eff = x;
	if (_epsilon > 0.0f) {
		const float abs_x = fabsf(x);
		x_eff = (abs_x > 0.0f) ? (x * abs_x / (abs_x + _epsilon)) : 0.0f;
	}

	// Helper quantities (paper Eq. 11-13)
	// a = h * lambda1 > 0
	// b_k = -x_{1,k} - h * nu_k
	const float a = h * lambda1;
	const float b_k = -x_eff - h * _nu;

	// Threshold for case selection
	const float h2_lambda2 = h * h * lambda2;

	float u;

	// Case split based on b_k (paper text below Fig. 2)
	if (b_k < -h2_lambda2) {
		// ===== Case 1: b_k < -h²λ2 =====
		// xi = 1, x_tilde > 0
		_last_case = 1;

		// sqrt(|x_tilde|) = (-a + sqrt(a² - 4*(b_k + λ2*h²))) / 2
		const float discriminant = a * a - 4.0f * (b_k + h2_lambda2);

		// Discriminant should be positive in Case 1; defensive check
		const float sqrt_disc = (discriminant > 0.0f) ? sqrtf(discriminant) : 0.0f;
		const float sqrt_x_tilde = (-a + sqrt_disc) * 0.5f;

		// Ensure non-negative (numerical safety)
		const float sqrt_x_tilde_safe = (sqrt_x_tilde > 0.0f) ? sqrt_x_tilde : 0.0f;

		// Update nu: nu_{k+1} = nu_k - h*λ2
		_nu = _nu - h * lambda2;

		// Control: u_k = -λ1 * sqrt(|x_tilde|) + nu_{k+1}
		u = -lambda1 * sqrt_x_tilde_safe + _nu;

	} else if (b_k > h2_lambda2) {
		// ===== Case 3: b_k > h²λ2 =====
		// xi = -1, x_tilde < 0
		_last_case = 3;

		// sqrt(|x_tilde|) = (-a + sqrt(a² + 4*(b_k - λ2*h²))) / 2
		const float discriminant = a * a + 4.0f * (b_k - h2_lambda2);

		// Discriminant should be positive in Case 3; defensive check
		const float sqrt_disc = (discriminant > 0.0f) ? sqrtf(discriminant) : 0.0f;
		const float sqrt_x_tilde = (-a + sqrt_disc) * 0.5f;

		// Ensure non-negative (numerical safety)
		const float sqrt_x_tilde_safe = (sqrt_x_tilde > 0.0f) ? sqrt_x_tilde : 0.0f;

		// Update nu: nu_{k+1} = nu_k + h*λ2
		_nu = _nu + h * lambda2;

		// Control: u_k = λ1 * sqrt(|x_tilde|) + nu_{k+1}
		u = lambda1 * sqrt_x_tilde_safe + _nu;

	} else {
		// ===== Case 2: b_k ∈ [-h²λ2, h²λ2] =====
		// x_tilde = 0 (sliding mode reached)
		_last_case = 2;

		// Control: u_k = nu_{k+1} = -x_{1,k} / h
		// This also updates nu implicitly
		// NOTE: This can produce very large values when h is small.
		u = -x_eff / h;
		_nu = u;
	}

	// Final NaN safety check on output
	if (!PX4_ISFINITE(u)) {
		u = 0.0f;
		_nu = 0.0f;
	}

	if (!PX4_ISFINITE(_nu)) {
		_nu = 0.0f;
	}

	return u;
}

void IstaController::setGains(float lambda1, float lambda2)
{
	// Ensure positive gains
	_lambda1 = (lambda1 > 0.0f) ? lambda1 : 1.0f;
	_lambda2 = (lambda2 > 0.0f) ? lambda2 : 1.0f;
}

void IstaController::setEpsilon(float epsilon)
{
	_epsilon = (epsilon > 0.0f) ? epsilon : 0.0f;
}

void IstaController::reset()
{
	_nu = 0.0f;
	_last_case = 0;
}

void IstaController::adjustNu(float delta)
{
	if (!PX4_ISFINITE(delta)) {
		return;
	}

	_nu += delta;

	if (!PX4_ISFINITE(_nu)) {
		_nu = 0.0f;
	}
}
