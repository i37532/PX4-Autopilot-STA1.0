#pragma once
#include <math.h>
#include <float.h>

class ISTA {
public:
    ISTA() = default;

    void setLambda1(float l1) { _lambda1 = l1; }
    void setLambda2(float l2) { _lambda2 = l2; }
    void resetNu(float nu0 = 0.0f) { _nu = nu0; }

    float update(float x, float h);
private:
    float _lambda1{0.0f};
    float _lambda2{0.0f};
    float _nu{0.0f};
};

