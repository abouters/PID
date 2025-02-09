#include <iostream>
#include <thread>
#include <chrono>
#include "PID.h"

int main() {
  double cycle = 0.01; // 制御周期[s]
  PIDGain gain = {1.0, 0.5, 0.1}; // Kp, Ki, Kd の設定
  double minLimit = -10.0; // 出力の最小値
  double maxLimit = 10.0;  // 出力の最大値
  PID pid(cycle, minLimit, maxLimit);
  pid.setGain(gain);

  // 制御対象の初期状態
  double target = 50.0;
  double current = 0.0;
  double output = 0.0;

  // 制御周期ごとにcalculate関数を呼び出し計算を行う
  while(1){
    current = 1; // 現在値を更新する
    double error = target - current; // 偏差の計算
    output = pid.calculate(error);  // PID制御出力の計算
    // 時間を進める
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10ms待機
  }

  return 0;
}
