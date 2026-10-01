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

The idea is that when the car moves faster, we want the controller to look further ahead so that it can react to the upcoming wall geometry earlier.

We started with a preview time of around 0.3 s because we thought it was a reasonable prediction horizon. It is also on a similar time scale to the 0.3 s TTC threshold we use in our safety node, although these two values have different purposes.

During testing, we found that making the look-ahead distance too large could actually make the car less stable. In particular, when `L` became larger than around 4 m, the car sometimes started making small repeated left-right steering corrections.

From our testing, we think this happens because a very large look-ahead distance makes the controller react too strongly to wall geometry that is far away. It also makes small errors in the wall-angle estimation more noticeable.

Because of this, we limit the maximum look-ahead distance to 3 m.

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

We tested different values for this angle and found that it has a noticeable effect, especially on the Levine map.

For example, at the first sharp left turn in Levine, if `theta` is too large, the forward LiDAR beam may point toward another wall surface instead of the left wall that we actually want to follow.

When this happens, the two LiDAR beams are no longer measuring the same wall, so the calculated wall angle can become inaccurate. This can make the car react too late or steer incorrectly through the corner.

However, we also found that making `theta` too small is not ideal. If the two beams are too close together, the controller has less forward-looking information and may detect the upcoming turn too late.

After testing different values, we found that using a beam angle of 50°, which gives `theta = 40°`, gives us a good balance between detecting turns early and still measuring the correct wall.

We also noticed that Spielberg is much less sensitive to this parameter. The wall geometry is easier for the two-ray wall-following method, so a wider range of beam angles can still work reasonably well.

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

We tuned the PID controller by testing the P, D, and I terms separately and observing how the car behaved on different parts of the track.

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

We found that a larger proportional gain gives stronger steering corrections, which can help the car respond quickly to wall-distance errors. However, if `Kp` is too large, the steering becomes too aggressive and the car can start oscillating.

On the other hand, if `Kp` is too small, the car reacts too slowly and may not turn enough when entering a corner.

After multiple tests, we found that:

```text
Kp = 0.7
```

gave us the most stable overall performance.

---

#### Derivative Gain

At the beginning, we did not pay much attention to the derivative term and originally thought that proportional control might already be enough.

However, when we increased the speed on Spielberg, especially above around 8 m/s, we noticed a problem after sharp turns.

After leaving a corner, the car was sometimes not pointing straight enough for the next straight section. The proportional controller would then try to correct the remaining error.

Because the car was already moving quickly, the correction could become too strong. The car would then move too far in the opposite direction, causing the P controller to correct again.

This created repeated left-right oscillation after the corner.

To reduce this behaviour, we added the derivative term.

The derivative term reacts to how quickly the wall-following error is changing, so it helps reduce sudden steering corrections and provides damping.

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

The main reason is that the wall-following environment changes very quickly. The car is constantly moving between straights, corners, and different wall geometries.

During our simulator testing, we did not notice a significant error that stayed in the same direction for a long period of time.

Since the integral term is mainly useful for correcting long-term steady-state error, we did not see much benefit from using it in our current controller.

We were also concerned that accumulated integral error could cause unnecessary overshoot when the track geometry suddenly changes.

Because of this, we decided to use a PD controller instead of a full PID controller.

The current values work well in our testing, but we do not consider them theoretically optimal. There is still room for further tuning and improvement.

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

This means that when the car is travelling almost straight, we allow it to move faster.

When the controller requests a larger steering angle, we treat this as an indication that the car is entering or travelling through a sharper turn, so we reduce the speed.

This gives us a simple way to slow down before and during corners without implementing a much more complicated speed-planning algorithm.

During our testing on Spielberg, we were able to complete the track while reaching peak speeds of around 8–10 m/s with suitable controller tuning.

For Levine, we had to use more conservative speeds because the map has more difficult wall geometry and sharper transitions.

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
