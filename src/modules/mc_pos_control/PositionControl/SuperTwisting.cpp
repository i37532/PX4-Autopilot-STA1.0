

#include "SuperTwisting.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>
#include <iostream>  // 引入输入输出流库


SuperTwisting::SuperTwisting(/* args */)
{
}

SuperTwisting::~SuperTwisting()
{
}


void SuperTwisting::_staZPositionControl(const float dt, matrix::Vector3f& _pos, matrix::Vector3f& _pos_sp, matrix::Vector3f& _vel, matrix::Vector3f& _vel_sp, matrix::Vector3f& _acc_sp)
{
	float z_pos_error = _pos(2) - _pos_sp(2);
	_sta_z_pos_error = z_pos_error;

	if(!PX4_ISFINITE(z_pos_error)){
		z_pos_error = 0.0;
	}



	_sta_int_z_pos_error = z_pos_error;



	_sta_last_three_pos_sp = _sta_last_two_pos_sp;
	_sta_last_two_pos_sp = _sta_last_one_pos_sp;
	_sta_last_one_pos_sp = _pos_sp(2);

	_sta_int_z_pos_error = math::constrain(_sta_int_z_pos_error, -(float)_sta_z_pos_error_up, (float)_sta_z_pos_error_up);




	//  float deri_z_pos = (_pos(2) - _last_z_pos) / (dt);
	float deri_z_pos = _vel(2);

	if(!PX4_ISFINITE(deri_z_pos)){
	deri_z_pos = 0.0;
	}

	_sta_deri_z_pos_error = deri_z_pos;



	_last_z_pos = _pos(2);
	float ita = deri_z_pos + _sta_sliding_c * _sta_int_z_pos_error - _vel_sp(2);

	_sta_ita = ita;

	float ita_norm = 0;
	if(fabs(ita) < (float)_sta_ita_norm_up){
		ita_norm = (float)_sta_ita_norm_up;
	}else{
		ita_norm = fabs(ita);
	}



	// ====== 使用隐式 supertwisting（ISTA） ======

	// 1）ISTA 的输入是滑模面 ita，步长是 dt
	const float u_sta = _ista_z.update(ita_norm, dt);

	// 2）把 ISTA 的输出 u_sta 当成 supertwisting 非线性项
	//    原来控制律是：c * ev + λ * |s|^{1/2}sign(s) + w - acc_sp
	//    现在：c * ev + u_sta - acc_sp
	_sta_thrust = _sta_sliding_c * (_vel(2) - _vel_sp(2))
			+ u_sta
			- _acc_sp(2);

	if (!PX4_ISFINITE(_sta_thrust)) {
		_sta_thrust = 0.0f;
	}

	// 3）隐式形式内部已经有“积分状态” _nu，不再需要外部的 _sta_w / _sta_w_dot
	_sta_w     = 0.0f;
	_sta_w_dot = 0.0f;



	//publish
	_sta_status.timestamp = hrt_absolute_time();
	_sta_status.sta_sliding_c = _sta_sliding_c;
	_sta_status.sta_w_dot = _sta_w_dot;          // 补全字段
	_sta_status.sta_w = _sta_w;                  // 补全字段
	_sta_status.sta_thrust = _sta_thrust;        // 补全字段
	_sta_status.sta_z_pos_error = _sta_z_pos_error;  // 补全字段
	_sta_status.sta_deri_z_pos_error = _sta_deri_z_pos_error;  // 补全字段
	_sta_status.sta_ita = _sta_ita;              // 补全字段
	_sta_status.sta_z_pos_error_up = _sta_z_pos_error_up;      // 补全字段
	_sta_status.sta_ita_norm_up = _sta_ita_norm_up;            // 补全字段
	_sta_status.sta_int_z_pos_error = _sta_int_z_pos_error;    // 补全字段

	_publish_sta_status.publish(_sta_status);

}

float SuperTwisting::_getStaThrust()
{
	return this->_sta_thrust;
}

// void SuperTwisting::_set_sta_w(float less_num){
// 	_sta_w -= less_num;
// }

void SuperTwisting::_set_sta_w(float less_num)
{
    (void)less_num;   // 避免未使用警告，可以删掉形参也行

    _ista_z.reset();  // 把隐式 supertwisting 的内部积分 ν 清零
    _sta_w     = 0.0f;
    _sta_w_dot = 0.0f;
}


void SuperTwisting::_update_sta_sliding_c(float new_value)
{
	_sta_sliding_c = new_value;
}

void SuperTwisting::_update_sta_sliding_z_error_up(float new_value)
{
	_sta_z_pos_error_up = new_value;
}

void SuperTwisting::_update_sta_ita_norm_up(float new_value)
{
	_sta_ita_norm_up = new_value;
}
