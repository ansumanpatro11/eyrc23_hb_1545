def calculate_pwm(rpm):
    a = 0.0000
    b = -0.0001
    c = -1.6928
    d = 102.1493
    pwm = int(a * pow(rpm, 3) + b * pow(rpm, 2) + c * rpm + d);    
    
    return pwm

# Example usage:
rpm_input = 30  # Replace this with your desired RPM input
result_pwm = calculate_pwm(rpm_input)
print(f"For {rpm_input} RPM, PWM value is: {result_pwm}")
