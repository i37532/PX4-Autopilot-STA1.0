#include "PositionStaControl.hpp"
#include <math.h>

float PositionStaControl::update(const float dt, matrix::Vector3f& _pos, matrix::Vector3f& _pos_sp,
	matrix::Vector3f& _vel, matrix::Vector3f& _vel_sp, matrix::Vector3f& _acc_sp)
{
	// ==========================================
	// 1. 定义 ISTA 参数
	// ==========================================
	// 适当降低增益以防止过冲，如果响应太慢可调大
	const float _lambda1 = 4.0f;
	const float _lambda2 = 3.0f;
	const float tol = 1e-12f;

	// ==========================================
	// 2. 计算滑模面 x (修正方向)
	// ==========================================
	// 【关键修改】PX4 Z轴向下为正。
	// 改为 (实际值 - 期望值)。
	// 例如：掉高时，Vel(正) > Vel_Sp(0)，误差为正 -> ISTA输出负(向上加速) -> 修正成功。
	_pos_ita = (_vel(2) - _vel_sp(2)) + _pos_sta_c * (_pos(2) - _pos_sp(2));

	if(!PX4_ISFINITE(_pos_ita)) {
		_pos_ita = 0.f;
	}

	float x = _pos_ita;
	float h = dt;

	// ==========================================
	// 3. 执行 ISTA 算法 (保持不变)
	// ==========================================
	float b = -x - h * _pos_sta_w;
	float a = h * _lambda1;

	float u_ista = 0.0f;
	float sqrt_tilde_x = 0.0f;
	float xi = 0.0f;
	float disc = 0.0f;

	if (b < -h * h * _lambda2) {
		xi = 1.0f;
		disc = a * a - 4.0f * (b + _lambda2 * h * h);
		if (disc < -tol) disc = 0.0f;
		sqrt_tilde_x = (-a + sqrtf(fmaxf(disc, 0.0f))) / 2.0f;
		sqrt_tilde_x = fmaxf(sqrt_tilde_x, 0.0f);

		_pos_sta_w -= h * _lambda2 * xi;
		u_ista = -_lambda1 * sqrt_tilde_x * xi + _pos_sta_w;

	} else if (fabsf(b) <= h * h * _lambda2) {
		xi = b / (-h * h * _lambda2);
		sqrt_tilde_x = 0.0f;

		_pos_sta_w = -x / h;
		u_ista = _pos_sta_w;

	} else {
		xi = -1.0f;
		disc = a * a + 4.0f * (b - _lambda2 * h * h);
		if (disc < -tol) disc = 0.0f;
		sqrt_tilde_x = (-a + sqrtf(fmaxf(disc, 0.0f))) / 2.0f;
		sqrt_tilde_x = fmaxf(sqrt_tilde_x, 0.0f);

		_pos_sta_w += h * _lambda2;
		u_ista = _lambda1 * sqrt_tilde_x + _pos_sta_w;
	}

	// 限制积分项，防止离地前积分过大导致起飞跳变
	_pos_sta_w = math::constrain(_pos_sta_w, -5.0f, 5.0f);

	// ==========================================
	// 4. 计算最终输出 (修正单位和逻辑)
	// ==========================================
	// 【关键修改】
	// 1. 去掉 _mc_mass 乘法。因为 PositionControl 需要的是加速度(m/s^2)，不是力(N)。
	// 2. 去掉 -_acc_sp(2)。只需要返回“修正量”，PositionControl 会自动把它加到前馈上。

	_pos_thrust = u_ista;

	return _pos_thrust;
}

void PositionStaControl::updateIntW(float thr_z_sp, float lim_thr_min, float lim_thr_max, const float dt)
{
	// 仅做简单的抗饱和限制
	_pos_sta_w = math::constrain(_pos_sta_w, -5.0f, 5.0f);
}
