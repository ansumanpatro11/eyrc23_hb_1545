def calculate_pwm(rpm):
    pwm = -0.0012 * rpm**3 + 0.1000 * rpm**2 - 4.1541 * rpm + 121.1095
    return pwm

# Example usage:
rpm_input = 17  # Replace this with your desired RPM input
result_pwm = calculate_pwm(rpm_input)
print(f"For {rpm_input} RPM, PWM value is: {result_pwm}")
