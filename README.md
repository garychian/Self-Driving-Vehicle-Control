# Self-Driving-Vehicle-Control
Longitudinal control, Lateral control 

This project, we will implement a controller (Longitudinal and Lateral) for CARLA simulator. Our goal is control the vehicle to follow a race track by navigating through preset waypoints. The vehicle needs to reach these waypoints at certain desired speeds. 

## File structure
Under the PythonClient folder, there is a folder called Course1FinalProject folder, this is the main folder we are going to working on. 
### Review
Before we implement the controller, let's review some key points for thoese controllers. 

|Description         | Variable Name | Units          |
|---------------     |:-------------:|:--------------:|
|Vehicle Location    |[x,y]          | [meters,meters]|
|Vehicle Orientation |yaw            | radians        |
|Vehicle Speed       |v              | meters per s   |
|In-Game Time        |t              | seconds        |
|Desired Speed       |v_desired      | meters per s   |
|Waypoints to track  |waypoints      | [m,m,m/s]      |
|   output                                            |
|Throttle            |throttle_output|0 to 1(in per)  |
|Steering            |steer_output   |-1.22 to 1.22   |
|Brake               |brake_output   | 0 to 1(in per) |

* Initial the parameters

```python
Kp = 2
Ki = 0.05
Kd = 0.01

k = 0.1
lad = 1.0  # look-ahead-distance
L = 2.9    # Wheelbase

```

* Longitudinal Control: PID controller

![PID](https://github.com/garychian/Self-Driving-Vehicle-Control/blob/master/PID.jpg)

PID controller takes Desired speed (vd) and Vehicle speed (v) as input. And output the Acceleration(u), if u >= 0, apply throttle (Tp), else, apply Brake (Bp).

```python
a = Kp * (V_desired - v)
throttle_output += a * t
brake_output = 0
```

* Lateral Control

![](https://github.com/garychian/Self-Driving-Vehicle-Control/blob/master/Lateral%20Control.jpg)

```python
# Use the pure pursuit controller
length = np.arange(0,100,1)
dx = [self._current_x - waypoints[icx][0] for icx in length]
dy = [self._current_y - waypoints[icy][1] for icy in length]
 d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx,idy) in zip(dx,dy)]
ind = d.index(min(d))
if ind < 2:
    tx = waypoints[ind][0]
    ty = waypoints[ind][1]
else:
	tx = waypoints[-1][0]
    ty = waypoints[-1][1]

alpha_hat = math.atan2(ty-y,tx-x)
alpha = alpha_hat - yaw
Lf = k * v + lad

steer_output    = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
print("steer_output = ", steer_output)

```


### controller2d.py
This is the file we need working on, we implement longitudinal and lateral controller here. When the controller is implemented, run `module_7.py` in terminal, we need keep the CARLA open as well. 

