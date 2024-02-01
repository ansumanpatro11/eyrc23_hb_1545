def calculate_pwm(rpm):
    a = -0.0007;
    b = -0.0549;
    c = -3.1529;
    d = 71.3725;

    pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);    
    
    return pwm

# Example usage:
rpm_input = -19  # Replace this with your desired RPM input
result_pwm = calculate_pwm(rpm_input)
print(f"For {rpm_input} RPM, PWM value is: {result_pwm}")
