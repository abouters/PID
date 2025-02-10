#ifndef PID_H
#define PID_H

#include <algorithm>

typedef struct{
	float Kp;
	float Ki;
	float Kd;
}PIDGain;

class PID{
    private:
        PIDGain gain = {0.0, 0.0, 0.0};
        double cycle;
        double minLimit, maxLimit;
        double prop = 0, deriv = 0;
        double pre_error = 0, pre_prop = 0;
        double low_pass_deriv_ = 0;     //ローパスフィルタ後の微分要素
        double output = 0;
        double lowPassFilterCoefficient = 8;  //ローパスフィルタの係数
    public:
        PID(): cycle(0), minLimit(0), maxLimit(0) {} // デフォルトコンストラクタ
        PID(double cycle, double min, double max);
        ~PID();
        void setGain(PIDGain gain);
        void setLimit(double min, double max);
        void setLowPassFilterCoefficient(double coeff);
        double calculate(double error);
        double clamp(double value, double min, double max);
};

#endif