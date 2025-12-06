#include "ISTA.hpp"

float ISTA::update(float x, float h)
{
    float b = -x - h * _nu;
    float a = h * _lambda1;
    float tol = 1e-12f;

    float u = 0.0f;
    float sqrt_tilde_x = 0.0f;
    float xi = 0.0f;
    float disc = 0.0f;

    if (b < -h * h * _lambda2) {
        xi = 1.0f;
        disc = a * a - 4.0f * (b + _lambda2 * h * h);
        if (disc < -tol) disc = 0.0f;
        sqrt_tilde_x = (-a + sqrtf(fmaxf(disc, 0.0f))) / 2.0f;
        sqrt_tilde_x = fmaxf(sqrt_tilde_x, 0.0f);
        _nu -= h * _lambda2 * xi;
        u = -_lambda1 * sqrt_tilde_x * xi + _nu;

    } else if (fabsf(b) <= h * h * _lambda2) {
        xi = b / (-h * h * _lambda2);
        sqrt_tilde_x = 0.0f;
        _nu = -x / h;
        u = _nu;

    } else {
        xi = -1.0f;
        disc = a * a + 4.0f * (b - _lambda2 * h * h);
        if (disc < -tol) disc = 0.0f;
        sqrt_tilde_x = (-a + sqrtf(fmaxf(disc, 0.0f))) / 2.0f;
        sqrt_tilde_x = fmaxf(sqrt_tilde_x, 0.0f);
        _nu += h * _lambda2;
        u = _lambda1 * sqrt_tilde_x + _nu;
    }

    return u;
}
