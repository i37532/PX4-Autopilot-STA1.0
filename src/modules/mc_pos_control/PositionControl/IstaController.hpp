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
 * @file IstaController.hpp
 *
 * Implicit Super-Twisting Algorithm (ISTA) controller for single axis.
 * Based on: "The implicit discretization of the super-twisting sliding-mode
 * control algorithm" (BBSTA paper).
 *
 * Replaces PID velocity control law with sliding-mode control.
 */

#pragma once

#include <cmath>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

class IstaController
{
public:
	IstaController() = default;
	~IstaController() = default;

	/**
	 * @brief Execute one ISTA control step
	 *
	 * @param x   Sliding variable (velocity error for velocity control), units: m/s
	 * @param h   Time step (dt), units: s
	 * @return    Control output u (acceleration setpoint), units: m/s^2
	 *
	 * Algorithm from BBSTA paper Section III, closed-form case split.
	 */
	float update(float x, float h);

	/**
	 * @brief Set ISTA gains
	 *
	 * @param lambda1  Proportional-like gain (affects sqrt term)
	 * @param lambda2  Integral-like gain (affects nu dynamics)
	 */
	void setGains(float lambda1, float lambda2);

	/**
	 * @brief Set boundary layer width for smooth sign (0 disables smoothing)
	 *
	 * @param epsilon boundary layer width (units: same as x, m/s)
	 */
	void setEpsilon(float epsilon);

	/**
	 * @brief Reset internal state (nu) to zero
	 *
	 * Call on disarm, mode change, or when PID integrator is reset.
	 */
	void reset();

	/**
	 * @brief Get current internal state nu
	 * @return nu value (units: m/s^2 for velocity control context)
	 */
	float getNu() const { return _nu; }

	/**
	 * @brief Get last executed case (1, 2, or 3) for debugging
	 * @return Case ID from last update() call
	 */
	int getLastCase() const { return _last_case; }

	/**
	 * @brief Get current lambda1
	 */
	float getLambda1() const { return _lambda1; }

	/**
	 * @brief Get current lambda2
	 */
	float getLambda2() const { return _lambda2; }

	/**
	 * @brief Adjust internal state (nu) by a delta
	 *
	 * Useful for compensating hover thrust updates.
	 */
	void adjustNu(float delta);

private:
	float _lambda1{1.0f};   ///< ISTA gain lambda1 (proportional-like)
	float _lambda2{1.0f};   ///< ISTA gain lambda2 (integral-like)
	float _epsilon{0.0f};   ///< Boundary layer width for smooth sign
	float _nu{0.0f};        ///< Internal controller state (persistent)
	int _last_case{0};      ///< Last executed case (1, 2, or 3) for debug
};
