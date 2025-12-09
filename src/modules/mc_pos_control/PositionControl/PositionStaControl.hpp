#pragma once
#include <matrix/matrix/math.hpp>

#include <mathlib/mathlib.h>

#include <uORB/topics/position_sta_status.h>

class PositionStaControl
{
public:
	PositionStaControl() = default;
	~PositionStaControl() = default;

	float update(const float dt, matrix::Vector3f& _pos, matrix::Vector3f& _pos_sp,
		 matrix::Vector3f& _vel, matrix::Vector3f& _vel_sp, matrix::Vector3f& _acc_sp);

	void updateIntW(float thr_z_sp, float lim_thr_min, float lim_thr_max, const float dt);

	void setPosStaParams(float mc_mass, float pos_sta_c,float pos_sta_alpha, float pos_sta_lamada)
	{
		_mc_mass = mc_mass;
		_pos_sta_c = pos_sta_c;
		_pos_sta_alpha = pos_sta_alpha;
		_pos_sta_lamada = pos_sta_lamada;
	}

	void setPosStaWlimit(float pos_sta_w_limit)
	{
		_pos_sta_w_limit = pos_sta_w_limit;
	}

	void setPosStaNormLimit(float pos_sta_norm_max, float pos_sta_norm_min)
	{
		_pos_sta_norm_max = pos_sta_norm_max;
		_pos_sta_norm_min = pos_sta_norm_min;
	}

	void resetPosStaW()
	{
		_pos_sta_w = 0;
	}

	void lessPosStaW(float less_num)
	{
		_pos_sta_w -= less_num;
	}

	float getPosStaThrust(){return _pos_thrust;}

	float getVelErrorDivNorm(float vel_error, size_t axis);

	void setPosXyStaNormLimit(matrix::Vector2f pos_xy_sta_norm_min, matrix::Vector2f pos_xy_sta_norm_max)
	{
		_pos_xy_sta_norm_min = pos_xy_sta_norm_min;
		_pos_xy_sta_norm_max = pos_xy_sta_norm_max;
	}

	void getPosZStaStatus(position_sta_status_s& pos_sta_status){
		pos_sta_status.pos_z_ita = _pos_ita;
		pos_sta_status.pos_z_thrust = _pos_thrust;
		pos_sta_status.pos_z_w = _pos_sta_w;
		pos_sta_status.pos_z_w_dot = _pos_sta_w_dot;
	}

private:
	//pos sta params
	float _mc_mass = 1.0;
	float _pos_sta_c = 1.2;
	float _pos_sta_alpha;
	float _pos_sta_lamada;

	//key variable
	float _pos_ita;

	//pos sta control result
	float _pos_thrust;

	//pos sta intergrator
	float _pos_sta_w;
	float _pos_sta_w_dot;

	//pos sta control w limit
	float _pos_sta_w_limit;

	//norm limit
	float _pos_sta_norm_max;
	float _pos_sta_norm_min;

	matrix::Vector2f _pos_xy_sta_norm_max;
	matrix::Vector2f _pos_xy_sta_norm_min;

	float calculateIsta(float sigma, float dt, float alpha, float lambda, float &integral_state);




};
