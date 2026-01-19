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
 * @file mc_pos_control_ista_params.c
 *
 * Parameters for ISTA (Implicit Super-Twisting Algorithm) velocity control.
 * Based on BBSTA paper: "The implicit discretization of the super-twisting
 * sliding-mode control algorithm".
 */

/**
 * Enable ISTA velocity control
 *
 * Replace PID velocity control law with ISTA (Implicit Super-Twisting Algorithm).
 * When disabled (0), standard PID velocity control is used.
 * When enabled (1), ISTA replaces the P+I part of velocity control.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_VEL_ISTA_EN, 0);

/**
 * ISTA lambda1 gain for XY velocity
 *
 * Proportional-like gain affecting the sqrt term in ISTA.
 * Higher values give faster convergence but may cause oscillation.
 * Start with conservative values and tune gradually.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ISTA_L1_XY, 1.0f);

/**
 * ISTA lambda2 gain for XY velocity
 *
 * Integral-like gain affecting the nu dynamics in ISTA.
 * Higher values give faster disturbance rejection.
 * Start with conservative values and tune gradually.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ISTA_L2_XY, 0.5f);

/**
 * ISTA lambda1 gain for Z velocity
 *
 * Proportional-like gain affecting the sqrt term in ISTA for vertical axis.
 * Higher values give faster convergence but may cause oscillation.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ISTA_L1_Z, 1.2f);

/**
 * ISTA lambda2 gain for Z velocity
 *
 * Integral-like gain affecting the nu dynamics in ISTA for vertical axis.
 * Higher values give faster disturbance rejection.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ISTA_L2_Z, 0.8f);

/**
 * Keep D-term when using ISTA
 *
 * When enabled, the existing velocity D-term (-vel_dot * D_gain) is added
 * to ISTA output for additional damping. Recommended for first integration.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_ISTA_KEEP_D, 1);

/**
 * ISTA hover deadband for XY velocity
 *
 * If both velocity setpoint and velocity error are below this value,
 * XY ISTA output is zeroed and nu decays to prevent drift.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ISTA_HOV_DB, 0.05f);

/**
 * ISTA hover nu decay time constant
 *
 * Time constant (seconds) used to decay nu in hover deadband.
 * Set to 0 to disable decay.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ISTA_HOV_TC, 0.5f);

/**
 * ISTA boundary layer width (smooth sign)
 *
 * Replaces hard switching with a smoothed transition around zero to reduce chattering.
 * Set to 0 to disable smoothing.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ISTA_EPS, 0.05f);
