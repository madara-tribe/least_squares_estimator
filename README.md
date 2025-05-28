# Estimate real distance via least squares
This repository is to convert relative depth(midas prediction from [this repository](https://github.com/madara-tribe/Ros2-DepthCalculator-PX2)) to the actual physical distance (absolute distance) via the least-squares method.

after getting the absolute distance and x-coordinate, calculate angle as servo motor angle for real-world robotic applications such as object tracking or alignment.

as for the angle calcation(θ), refer the figure and code below.

![Image](https://github.com/user-attachments/assets/7d5d643a-2c41-4f97-8e4c-52964588ecde)


## formula to get angle

when x change from 0 to 100, you can get angle θ（degree）

$$
\theta(x) = \arccos \left( \frac{-AO \cdot (x - AO) + OP^2}{\sqrt{AO^2 + OP^2} \cdot \sqrt{(x - AO)^2 + OP^2}} \right) \times \frac{180}{\pi}
$$



```python
import numpy as np
AO = 50
OP = 90

def theta(x):
    dot = -AO * (x - AO) + OP**2
    mag_CA = np.sqrt(AO**2 + OP**2)
    mag_Cx = np.sqrt((x - AO)**2 + OP**2)
    cos_theta = dot / (mag_CA * mag_Cx)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return np.degrees(np.arccos(cos_theta))
for x in range(0, 101, 10):
    print(f"x = {x:3d} --> θ = {theta(x):.4f} [deg]")
```

# how to estimate
1. by running least-squares method to convert relative distance to absolute one by python
   
<code> $ python3 least_squares_calculate.py</code>

2. calcurate servo angle from absolute distance and x-coordinate

<code>ros2 run least_squares_fitter least_squares_fitter</code>



