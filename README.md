## Testing Strategy

We tuned our wall-following controller experimentally by testing it on different simulator maps and at different vehicle speeds. The main parameters we tested were the look-ahead distance, LiDAR beam angle, desired wall distance, PID gains, and speed control strategy.

### 1. Dynamic Look-Ahead Distance

Instead of using a fixed look-ahead distance, we make the look-ahead distance change with the vehicle speed:

```text
L = clip(L_min + preview_time × |velocity|, L_min, L_max)
```

We use:

```text
L_min = 1.0 m
L_max = 3.0 m
preview_time = 0.3 s
```

We used a preview time of 0.3 seconds since we felt that it was a good distance to predict for. Similar to our safety nodes TTC threshold of 0.3 seconds but with different meanings. 

After further testing we noticed that the higher you increase your look ahead distance, the car can become less stable. When L got higher than about 4m we noticed our car would begin to wiggle slightly left and right. 

This happens because a very large look-ahead distance makes the controller react too strongly to wall geometry that is far away. Also, it makes small errors in the wall-angle estimation more noticeable and makes over correction.

---

### 2. LiDAR Beam Angle

We estimate the left-wall orientation using two LiDAR measurements.

We choose:

```text
beam_angle = 50°
```

for the forward-left beam, and we use:

```text
90°
```

for the second beam.

Therefore:

```text
theta = 40°
```

We experimented with varying thetas and found that it can affect performance somewhat significantly. Especailly on the Levine map. 

If theta is too big at the first sharp turn to the left in Levine, the front left lidar beam could point towards another wall. Which fails the left-wall following strategy and moves outside of the map.

So our beams won't be reading off of the same wall and our wall angle could be inaccurate. Causing our car to turn late or incorrectly around the corner. 

But we also found that having theta be too small can cause problems as well. Because our beams will be reading too close to each other and may not have enough foresight when it comes to corners. 

After some more experimenting we found that using a beam angle of 50 degrees (theta = 40 degreess) was best. 

Spielburg wasn't very affected by changing theta that We were able to use a wider beam angle and still be able to read the walls correctly. 

---

### 3. Desired Wall Distance

We set:

```text
D_desired = 1.2 m
```

We choose this value because it keeps the car closer to the middle of the track instead of following the left wall too closely.

This is especially useful on Levine. During sharp left turns, if the car stays too close to the left wall, it has a higher chance of directly hitting the wall when the controller does not react fast enough.

By keeping around 1.2 m from the wall, we leave more space for the car to correct its path during these sharp turns.

We also found that this helps when the left wall has large gaps or sudden changes in geometry. On Levine, there are some areas where the wall shape changes significantly. If the car follows the wall too closely, these changes can cause a very strong steering response.

Using a larger desired distance does not completely remove this problem, but it gives the car more space to recover and makes the behaviour more stable.

---

### 4. PID Controller Tuning

We tuned the PID controller by testing the P, D, and I terms separately and observing how the car behaved on different parts of the track on different maps.

Our final values are:

```text
Kp = 0.70
Kd = 0.23
Ki = 0.00
```

#### Proportional Gain

We started our testing with:

```text
Kp = 1.0
```

and then tried different values over multiple runs.

We discovered that if you increase the proportional constant you can correct for error at a greater rate by moving the steering more. This could be useful to reduce wall distance error faster but if `Kp` is too big then the car will begin to swing back and forth. 

If `Kp` is too little then the car will move slowly and not turn as much as desired. 

After multiple tests, we found that:

```text
Kp = 0.7
```

gave us the most stable overall performance.

---

#### Derivative Gain

We did not consider the derivative part to be of very importance at first. 

However, after increasing our speed on Spielberg. Above about 8 m/s. We saw an issue occur when exiting corners.

Sometimes the car would leave the corner without being straight enough for the next straight away. Which would cause the P controller to act.

If our velocity was high enough the car may over correct to the other side. And our p controller would once again act.

This caused the car to continue oscillating to each side after exiting the corner.

By implementing the derivative part into our equation we can reduce this issue.

As we use derivative it will control the rate of change in our error for following the wall. This will reduce over corrections and add some damping to our car.

We started testing with approximately:

```text
Kd = 0.10
```

and then gradually increased the value.

After multiple tests, we found that:

```text
Kd = 0.23
```

gave us a good result. It reduced the oscillation after corners while still keeping the steering response fast enough.

---

#### Integral Gain

For the integral term, we decided to use:

```text
Ki = 0.0
```

The main reason is that there are many changes in the wall following scenario. The car will always be driving through different walls some may be straight while others may be curved. 

Throughout our simulation testing we never saw an error occur that would travel one direction for an extended amount of time. 

For this reason we did not find it beneficial to use the I portion of our controller. 

If the integral did accumulate error we may have seen overshooting of the wall. 

So for our application we used a PD controller. 

These are good numbers that we can work with for now. However we do not believe they are theoretically optimized. 

---

### 5. Steering-Based Speed Control

We also tested different ways of controlling the vehicle speed.

If we use one constant high speed everywhere, the car can travel quickly on straight sections, but it becomes much harder to complete sharp turns safely.

Because of this, we use a three-level speed controller based on the steering angle.

Our general idea is:

```text
small steering angle   → high speed
medium steering angle  → medium speed
large steering angle   → low speed
```

This means that if the car is moving somewhat straight, we can increase our speed.

If our controller tells us to steer more we assume our car is taking a tighter corner and decrease our speed.

This also allows us to decrease our speed when going into corners and through them. Without using a more advanced method of calculating our speeds.

We were able to drive the full course on Spielberg achieving max speeds of ~8-10m/s. And 4.5m/s on Levine.

We had to slow down for Levine due to its more challenging wall map.

---

### Future Testing Improvements

For this milestone, most of our testing was based on repeated simulator runs and observing whether the car could complete the track safely and smoothly.

We mainly looked at things such as:

- whether the car completed the lap;
- whether it collided with a wall;
- whether it oscillated after corners;
- whether it reacted early enough to sharp turns;
- whether the AEB system successfully stopped the vehicle when necessary.

In the future, we could make the testing process more systematic by recording quantitative data instead of mainly relying on visual observation.

For example, we could measure:

- lap completion rate;
- number of collisions;
- lap time;
- average wall-following error;
- maximum wall-following error;
- steering oscillation after corners;
- minimum TTC during each run;
- performance at different fixed speeds;
- performance on Spielberg, Levine, and Levine Blocked;
- robustness to invalid or noisy LiDAR measurements.

This would allow us to compare different parameter combinations more objectively and would make it easier to find better controller settings.
