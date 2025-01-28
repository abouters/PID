#include <PID.h>


PID::PID(double cycle, PIDGain gain, double min, double max)
    : cycle(cycle), gain(gain), minLimit(min), maxLimit(max){}

PID::~PID()
{
}

void PID::setGain(PIDGain gain){
    this->gain = gain;
}

void PID::setLimit(double min, double max){
    this->minLimit = min;
    this->maxLimit = max;
}

void PID::setLowPassFilterCoefficient(double coeff){
    this->lowPassFilterCoefficient = coeff;
}

double PID::calculate(double error){
	double prop = (error - pre_error) / cycle;	//偏差の1階微分値
	double deriv = (prop - pre_prop) / cycle;	//偏差の2階微分値
    low_pass_deriv_ += deriv / lowPassFilterCoefficient;

    output += gain.Kp*prop + gain.Ki*error + gain.Kd*low_pass_deriv_;
    output = std::clamp(output, minLimit, maxLimit);

    pre_error = error;
    pre_prop = prop;

    return output;
}