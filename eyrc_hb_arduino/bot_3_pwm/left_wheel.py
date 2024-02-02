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

50.2
50
49.2
45.2
40
34.1
28
20.4
13.2
10

const float A = 0.0001;
const float B = -0.0160;
const float C = 0.1543;
const float D = 50.2928;

// Function to calculate PWM based on RPM using coefficients
float calculatePWM(float rpm) {
  float pwm = A * pow(rpm, 3) + B * pow(rpm, 2) + C * rpm + D;
  return pwm;
}


######
-10
-14.4
-20.6
-25.2
-30.2
-36.9
-44.1
-48.6
-49.7
-50


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

float a = -0.0008;
float b = -0.0693;
float c = -3.478;
float d = 69.9419;

#Function to calculate PWM based on RPM
float calculatePWM(int rpm) {
  float pwm = coef_a * pow(rpm, 3) + coef_b * pow(rpm, 2) + coef_c * rpm + coef_d;

  return pwm;
}



