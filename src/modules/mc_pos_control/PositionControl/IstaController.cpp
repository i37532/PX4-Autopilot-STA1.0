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

	// Helper quantities (paper Eq. 11-13)
	// a = h * lambda1 > 0
	// b_k = -x_{1,k} - h * nu_k
	const float a = h * lambda1;
	const float b_k = -x - h * _nu;

	// Threshold for case selection
	const float h2_lambda2 = h * h * lambda2;

	float u;

	const float u_case2 = -x / h;
	const float nu_case2 = u_case2;
	const float eps = math::max(_epsilon, 0.f);

	float u_case1 = 0.f;
	float nu_case1 = 0.f;
	float u_case3 = 0.f;
	float nu_case3 = 0.f;

	const auto compute_case1 = [&]() {
		// sqrt(|x_tilde|) = (-a + sqrt(a² - 4*(b_k + λ2*h²))) / 2
		const float discriminant = a * a - 4.0f * (b_k + h2_lambda2);
		const float sqrt_disc = (discriminant > 0.0f) ? sqrtf(discriminant) : 0.0f;
		const float sqrt_x_tilde = (-a + sqrt_disc) * 0.5f;
		const float sqrt_x_tilde_safe = (sqrt_x_tilde > 0.0f) ? sqrt_x_tilde : 0.0f;

		nu_case1 = _nu - h * lambda2;
		u_case1 = -lambda1 * sqrt_x_tilde_safe + nu_case1;
	};

	const auto compute_case3 = [&]() {
		// sqrt(|x_tilde|) = (-a + sqrt(a² + 4*(b_k - λ2*h²))) / 2
		const float discriminant = a * a + 4.0f * (b_k - h2_lambda2);
		const float sqrt_disc = (discriminant > 0.0f) ? sqrtf(discriminant) : 0.0f;
		const float sqrt_x_tilde = (-a + sqrt_disc) * 0.5f;
		const float sqrt_x_tilde_safe = (sqrt_x_tilde > 0.0f) ? sqrt_x_tilde : 0.0f;

		nu_case3 = _nu + h * lambda2;
		u_case3 = lambda1 * sqrt_x_tilde_safe + nu_case3;
	};

	// Case split with optional boundary layer smoothing.
	if (eps <= 0.f) {
		if (b_k < -h2_lambda2) {
			_last_case = 1;
			compute_case1();
			_nu = nu_case1;
			u = u_case1;

		} else if (b_k > h2_lambda2) {
			_last_case = 3;
			compute_case3();
			_nu = nu_case3;
			u = u_case3;

		} else {
			_last_case = 2;
			_nu = nu_case2;
			u = u_case2;
		}

	} else {
		const float lower = -h2_lambda2;
		const float upper = h2_lambda2;

		if (b_k < lower) {
			_last_case = 1;

			if (b_k > (lower - eps)) {
				compute_case1();
				const float t = math::constrain((lower - b_k) / eps, 0.f, 1.f);
				_nu = nu_case2 + t * (nu_case1 - nu_case2);
				u = u_case2 + t * (u_case1 - u_case2);

			} else {
				compute_case1();
				_nu = nu_case1;
				u = u_case1;
			}

		} else if (b_k > upper) {
			_last_case = 3;

			if (b_k < (upper + eps)) {
				compute_case3();
				const float t = math::constrain((b_k - upper) / eps, 0.f, 1.f);
				_nu = nu_case2 + t * (nu_case3 - nu_case2);
				u = u_case2 + t * (u_case3 - u_case2);

			} else {
				compute_case3();
				_nu = nu_case3;
				u = u_case3;
			}

		} else {
			_last_case = 2;
			_nu = nu_case2;
			u = u_case2;
		}
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
