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
        PIDGain gain = {};
        double cycle = 0;
        double minLimit = 0;     // 出力の最小値
        double maxLimit = 0;     // 出力の最大値
        double pre_error = 0;
        double pre_prop = 0;
        double low_pass_deriv_ = 0;     //ローパスフィルタ後の微分要素
        double output = 0;
        double lowPassFilterCoefficient = 8;  //ローパスフィルタの係数
    public:
        PID(double cycle, PIDGain gain, double min, double max);
        ~PID();
        void setGain(PIDGain gain);
        void setLimit(double min, double max);
        void setLowPassFilterCoefficient(double coeff);
        double calculate(double error);
};


#endif