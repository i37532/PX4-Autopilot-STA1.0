#pragma once
#include <math.h>
#include <float.h>

class ISTA
{
public:
    ISTA(float lambda1, float lambda2) :
        _lambda1(lambda1), _lambda2(lambda2) {}

    float update(float x, float dt);

    void reset() { _nu = 0.0f; }

private:
    float _nu{0.0f};
    float _lambda1;
    float _lambda2;
};
