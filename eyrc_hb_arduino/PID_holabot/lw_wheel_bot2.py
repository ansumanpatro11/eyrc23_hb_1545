# 0-90 pwm
const float a = -0.0011;
const float b = 0.0957;
const float c = -4.2499;
const float d = 124.4785;

float calculatePWMFromRPM(float rpm) {
  float pwm = a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d;
  return pwm;
}
# 90-180 pwm
10.9        
15
21.5
27
33.8
38.1
44.9
49.9
50.5
50.9

99
108
119
128
139
148
159
168
174
180
const float a = 0.0009;
const float b = -0.0755;
const float c = 3.6699;
const float d = 66.7297;

float calculatePWMFromRPM(float rpm) {
  float pwm = a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d;
  return pwm;
}




