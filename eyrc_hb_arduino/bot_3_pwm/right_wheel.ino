0
10
19
26
35
45
54
67
80
87

47.1
47.2
46.5
43.5
38.5
32.5
26.9
19.7
12.7
10

float a = -0.0022;
float b = 0.1795;
float c = -6.1369;
float d = 133.3418;

// Function to calculate PWM based on the first equation
float calculatePWMFirst(int rpm) {
  float pwm = coef_a_first * pow(rpm, 3) + coef_b_first * pow(rpm, 2) + coef_c_first * rpm + coef_d_first;

  return pwm;
}

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

-9.4
-13.8
-20.5
-24.8
-30.2
-35.2
-42.3
-46.5
-47.7
-48.5


float coef_a_fifth = -0.0007;
float coef_b_fifth = -0.0504;
float coef_c_fifth = -2.8787;
float coef_d_fifth = 75.7313;

// Function to calculate PWM based on the first equation
float calculatePWMFirst(int rpm) {
  float pwm = coef_a_first * pow(rpm, 3) + coef_b_first * pow(rpm, 2) + coef_c_first * rpm + coef_d_first;

  return pwm;
}