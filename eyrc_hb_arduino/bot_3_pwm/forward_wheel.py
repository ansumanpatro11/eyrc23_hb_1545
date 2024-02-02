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


49
48.2
47.8
44.2
38.7
33
28.1
20.5
13.5
10.5





float coef_a_second = -0.0020;
float coef_b_second = 0.1719;
float coef_c_second = -6.2266;
float coef_d_second = 138.0841;

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

-10.2
-14
-18.8
-23.9
-28
-34.5
-39
-45.6
-47
-47.6


float coef_a_third = -0.0005;
float coef_b_third = -0.0450;
float coef_c_third = -3.1333;
float coef_d_third = 71.3381;

// Function to calculate PWM based on the first equation
float calculatePWMFirst(int rpm) {
  float pwm = coef_a_first * pow(rpm, 3) + coef_b_first * pow(rpm, 2) + coef_c_first * rpm + coef_d_first;

  return pwm;
}