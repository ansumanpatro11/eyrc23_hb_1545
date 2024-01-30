# 90-180
const float a = 0.0008;
const float b = -0.0635;
const float c = 3.327;
const float d = 70.7519;

float calculatePWMFromRPM(float rpm) {
  float pwm = a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d;
  return pwm;
}
99
108
119
128
136
148
159
168
174
180

10.1
14.6
20.8
26.2
30.6
37.1
43.5
48.5
49.6
50

# 0-90
0
10
19
26
35
45
54
67
80
89

0
10
19
26
35
45
54
67
80
89
const float a = -0.0017;
const float b = 0.1404;
const float c = -5.1504;
const float d = 128.7589;

float calculatePWMFromRPM(float rpm) {
  float pwm = a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d;
  return pwm;
}

void setup() {
  // Initialize serial communicati
}