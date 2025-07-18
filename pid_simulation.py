 PID Controller Simulation
```python
import matplotlib.pyplot as plt

# PID parameters
Kp = 1.2
Ki = 1.0
Kd = 0.01
setpoint = 1.0

# Simulation setup
dt = 0.01
time = [0]
output = [0]
input_signal = [0]
error_sum = 0
last_error = 0

# Simple first-order system
def system_model(u, prev_output):
    tau = 1.0  # Time constant
    return prev_output + dt * (-prev_output + u) / tau

# PID Simulation
for i in range(1, 500):
    t = i * dt
    error = setpoint - output[-1]
    error_sum += error * dt
    d_error = (error - last_error) / dt

    # PID control law
    u = Kp * error + Ki * error_sum + Kd * d_error
    input_signal.append(u)

    # Update system output
    y = system_model(u, output[-1])
    output.append(y)
    time.append(t)
    last_error = error

# Plotting the response
plt.figure(figsize=(10,5))
plt.plot(time, output, label="System Output")
plt.axhline(setpoint, color='r', linestyle='--', label="Setpoint")
plt.title("PID Controller Response")
plt.xlabel("Time (s)")
plt.ylabel("Output")
plt.grid(True)
plt.legend()
plt.savefig("output.png")
plt.show()
