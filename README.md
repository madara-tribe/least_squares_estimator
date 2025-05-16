# Estimate real distance via least squares
This repository is to convert relative depth(midas prediction) to the actual physical distance (absolute distance) via the least-squares method. 

after getting the absolute distance and x-coordinate, calcurate angle as servo motor angle for real-world robotic applications such as object tracking or alignment.


# how to estimate
1. by running least-squares method to convert relative distance to absolute one by python
<code> $ python3 least_squares_calculate.py</code>
2. calcurate servo angle from absolute distance and x-coordinate
<code>ros2 run least_squares_fitter least_squares_fitter</code>

as for the angle calcatation, refer the figure and code below.
![Image](https://github.com/user-attachments/assets/7d5d643a-2c41-4f97-8e4c-52964588ecde)

```python
def calculate_angle_APh(x, z, x_max=80):
    # coordinate
    A = (0, 0)
    P = (int(x_max/2), z)
    h = (x, 0)

    # ectors AP and HP (origin shifted to point P)
    vec_AP = (A[0] - P[0], A[1] - P[1])
    vec_hP = (h[0] - P[0], h[1] - P[1])

    # Dot product and vector magnitude
    dot = vec_AP[0]*vec_hP[0] + vec_AP[1]*vec_hP[1]
    mag_AP = math.hypot(*vec_AP)
    mag_hP = math.hypot(*vec_hP)

    # cosθ = dot / (|a||b|)
    cos_theta = dot / (mag_AP * mag_hP)
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)

    return angle_deg
```

