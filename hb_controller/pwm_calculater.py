def calculate_pwm(rpm):
    a = -0.0007
    b = -0.0601
    c = -3.2047
    d = 71.6505
    pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);    
    
    return pwm

# Example usage:
rpm_input = -30  # Replace this with your desired RPM input
result_pwm = calculate_pwm(rpm_input)
print(f"For {rpm_input} RPM, PWM value is: {result_pwm}")
