# 90-180
10.3
14.7
21.2
25.4
30.6
37.2
44.4
49.6
50.5
50.9

99
108
119
126
136
148
159
168
174
180
const float a = 0.0007;
const float b = -0.0549;
const float c = 3.1529;
const float d = 71.3725;

float calculatePWMFromRPM(float rpm) {
  float pwm = a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d;
  return pwm;
} 

# 0-90
51.8
51.6
51.3
47.4
41.6
36.1
30.3
22.5
14.7
10.4

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



const float a = -0.0013;
const float b = 0.1086;
const float c = -4.4726;
const float d = 125.8242;

float calculatePWMFromRPM(float rpm) {
  float pwm = a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d;
  return pwm;
}