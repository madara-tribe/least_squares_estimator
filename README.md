# Estimate real distance via least squares
This repository is to convert relative depth(midas prediction from [this repository](https://github.com/madara-tribe/Ros2-DepthCalculator-PX2)) to the actual physical distance (absolute distance) via the least-squares method.

after getting the absolute distance and x-coordinate, calculate angle as servo motor angle for real-world robotic applications such as object tracking or alignment.

as for the angle calcation(θ), refer the figure and code below.

![Image](https://github.com/user-attachments/assets/7d5d643a-2c41-4f97-8e4c-52964588ecde)

```python
def calculate_angle_APh(x, z, x_max=80):
    # Coordinates
    A = (0, 0)
    P = (int(x_max/2), z)
    h = (x, 0)

    # Vectors AP and HP (origin shifted to point P)
    vec_AP = (A[0] - P[0], A[1] - P[1])
    vec_HP = (H[0] - P[0], H[1] - P[1])

    # Dot product and vector magnitude
    dot = vec_AP[0]*vec_HP[0] + vec_AP[1]*vec_HP[1]
    mag_AP = math.hypot(*vec_AP)
    mag_HP = math.hypot(*vec_HP)

    # cosθ = dot / (|a||b|)
    cos_theta = dot / (mag_AP * mag_HP)
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)
    return angle_deg
```

# how to estimate
1. by running least-squares method to convert relative distance to absolute one by python
   
<code> $ python3 least_squares_calculate.py</code>

2. calcurate servo angle from absolute distance and x-coordinate

<code>ros2 run least_squares_fitter least_squares_fitter</code>



