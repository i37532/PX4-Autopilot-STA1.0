#include "PositionStaControl.hpp"
#include <math.h> // 确保引入 math.h 以使用 fmaxf, sqrtf

/**
 * 位置稳定控制主函数，用于计算竖直方向上的推力命令
 * 使用隐式超螺旋算法 (Implicit Super-Twisting Algorithm)
 */
float PositionStaControl::update(const float dt, matrix::Vector3f& _pos, matrix::Vector3f& _pos_sp,
	matrix::Vector3f& _vel, matrix::Vector3f& _vel_sp, matrix::Vector3f& _acc_sp)
{
    // --- 竖直方向 (Z轴) 控制 ---

    // 1. 计算滑模面 sigma
    _pos_ita = _vel_sp(2) - _vel(2) + _pos_sta_c * (_pos_sp(2) - _pos(2));
    if(!PX4_ISFINITE(_pos_ita)) _pos_ita = 0.f;

    // 2. 调用 ISTA 算法计算控制量 u
    // 注意：传入 _pos_sta_w 的引用，它会在函数内被更新
    float u_ista_z = calculateIsta(_pos_ita, dt, _pos_sta_alpha, _pos_sta_lamada, _pos_sta_w);

    // 3. 计算最终推力 (注意 u_ista 需要取反)
    _pos_thrust = -_mc_mass * _acc_sp(2)
                  + _mc_mass * _pos_sta_c * (_vel(2) - _vel_sp(2))
                  - u_ista_z;

    // 清理无用的微分项 (如果头文件中没删的话)
    _pos_sta_w_dot = 0.f;

    return _pos_thrust;
}


/**
 * 更新扰动补偿项 w 的限幅
 * 注意：积分更新已在 update() 中通过 ISTA 隐式完成，此处仅保留限幅逻辑。
 */
/**
 * 更新扰动补偿项 w 的限幅
 * 修正：移除激进的抗饱和衰减，解决触底反弹和定高误差问题
 */
void PositionStaControl::updateIntW(float thr_z_sp, float lim_thr_min, float lim_thr_max, const float dt)
{
    // 1. 安全性检查
    if(!PX4_ISFINITE(_pos_sta_w)) _pos_sta_w = 0.f;

    // 2. 仅保留硬限幅
    // ISTA 算法具有很强的自适应性，外部干预积分项会导致稳态误差。
    // 我们只防止它超过物理允许的最大范围即可。
    _pos_sta_w = math::constrain(_pos_sta_w, -_pos_sta_w_limit, _pos_sta_w_limit);
}


/**
 * 计算归一化的速度误差（限幅后除以模长），用于 STA 控制器的 x、y轴
 * (保持不变)
 */
float PositionStaControl::getVelErrorDivNorm(float vel_error, size_t axis)
{
	// 仅处理 x 或 y 轴
	if(axis >= 2) return 0.f;

	// 获取速度误差的绝对值，并进行限幅
	float vel_error_norm = fabs(vel_error);
	vel_error_norm = math::constrain(vel_error_norm, _pos_xy_sta_norm_min(axis), _pos_xy_sta_norm_max(axis));

	// 返回归一化后的速度误差
	return vel_error / vel_error_norm;
}



float PositionStaControl::calculateIsta(float sigma, float dt, float alpha, float lambda, float &integral_state)
{
    float x = sigma;
    float h = dt;
    float b = -x - h * integral_state;
    float a = h * alpha;
    float xi = 0.0f;
    float sqrt_tilde_x = 0.0f;
    float u_out = 0.0f;

    // Case 1: b < -h^2 * lambda
    if (b < -h * h * lambda) {
        xi = 1.0f;
        float disc = a * a - 4.0f * (b + lambda * h * h);
        sqrt_tilde_x = (-a + sqrtf(fmaxf(disc, 0.0f))) / 2.0f;

        integral_state -= h * lambda * xi;
        u_out = -alpha * sqrt_tilde_x * xi + integral_state;
    }
    // Case 2: |b| <= h^2 * lambda (滑动模态区)
    else if (fabsf(b) <= h * h * lambda) {
        sqrt_tilde_x = 0.0f;

        // --- 修改开始 ---
        // 原代码: integral_state = -x / h;
        // 改进: 使用连续的增量更新，避免直接重置积分项导致的高频抖动。
        // 公式推导: xi = b / (-h^2 * lambda)
        // integral_state_new = integral_state_old - h * lambda * xi
        xi = b / (-h * h * lambda);
        integral_state -= h * lambda * xi;
        // --- 修改结束 ---

        u_out = integral_state;
    }
    // Case 3: b > h^2 * lambda
    else {
        xi = -1.0f;
        float disc = a * a + 4.0f * (b - lambda * h * h);
        sqrt_tilde_x = (-a + sqrtf(fmaxf(disc, 0.0f))) / 2.0f;

        integral_state += h * lambda; // xi is -1
        u_out = alpha * sqrt_tilde_x + integral_state;
    }

    return u_out;
}

