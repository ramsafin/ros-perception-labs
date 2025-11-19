# Time Synchronization Lab

This lab introduces two key concepts in ROS perception pipelines:

1. **Fake Camera Publisher** – generating synthetic image streams and inspecting ROS topics.
2. **Time Synchronization** – synchronizing messages from multiple sensors using `message_filters`.

The lab consists of **two parts**. Complete them in order.

---

# 📘 Lab 1 – Fake Camera Publisher

## 1. Getting Started

### Task 1.1 – Run the Fake Camera Publisher

Start ROS master:

```bash
roscore
```

In a new terminal:

```bash
rosrun ros_sync_buf_lab fake_camera_pub.py --frame basler --rate 1
```

You should see:

- A new topic appearing in ROS.
- Images containing a **number** (frame counter).
- One frame per second.

---

## 2. Inspecting ROS Topics

### Task 1.2 – Explore the Published Topic

List active topics:

```bash
rostopic list
```

> Which topic corresponds to your fake camera?

Inspect topic metadata:

```bash
rostopic info /basler/image_raw
```

> What message type does it use?

Inspect message structure:

```bash
rosmsg info sensor_msgs/Image
```

> Which fields in the message are important for synchronization?

Check publishing frequency:

```bash
rostopic hz /basler/image_raw
```

> Is the rate exactly 1 Hz? Why might it differ?

Inspect header timestamps:

```bash
rostopic echo /basler/image_raw/header
```

---

## 3. Visualizing Images

### Task 2.1 – View the Fake Camera Output

Run:

```bash
rqt_image_view
```

Select `/basler/image_raw`.

> What is shown in the image?
> How quickly does the digit change?

---

## 4. Experiment with Publishing Rate

### Task 3.1 – Try Higher Rates

Restart the publisher with different rates:

```bash
rosrun ros_sync_buf_lab fake_camera_pub.py --frame basler --rate <N>
```

Try: **10, 50, 100, 500 Hz**

Observe:

1. Output of `rostopic hz`
2. CPU load (`htop`)
3. Whether `rqt_image_view` can keep up

> At which point do frames begin to drop?

---

## 5. Running Two Cameras

### Task 4.1 – Start Two Camera Streams

Terminal 1:

```bash
rosrun ros_sync_buf_lab fake_camera_pub.py --frame left --rate 1
```

Terminal 2:

```bash
rosrun ros_sync_buf_lab fake_camera_pub.py --frame right --rate 2
```

Check:

```bash
rostopic hz /left/image_raw
rostopic hz /right/image_raw
```

Visualize both in `rqt_image_view`.

> Which stream publishes faster?
> How do the timestamps differ?

---

# 📘 Lab 2 – Time Synchronization

## 1. Background Concepts

### Why Synchronize?

Real robots often require combining information from multiple sensors:

- Stereo cameras
- Camera + IMU
- LiDAR + camera

Each publishes independently → timestamps drift.

### Exact Synchronization (`TimeSynchronizer`)

- Requires **identical timestamps**
- Rarely works with real camera hardware

### Approximate Synchronization (`ApproximateTimeSynchronizer`)

- Allows timestamp difference up to **`slop`**
- Much more robust

### Queue Size

Number of buffered messages per topic:

- Small queue → many missed matches
- Large queue → higher latency

---

## 2. Running Stereo Cameras

Use the provided launch file:

```bash
roslaunch ros_sync_buf_lab stereo_cameras.launch
```

Override camera rates:

```bash
roslaunch ros_sync_buf_lab stereo_cameras.launch left_cam_rate:=1 right_cam_rate:=10
```

Stop with `Ctrl+C`.

---

## 3. Create the Time Sync Launch File

Create `launch/stereo_sync.launch`:

```xml
<launch>
  <arg name="left_cam_rate"  default="1"/>
  <arg name="right_cam_rate" default="1"/>

  <include file="$(find ros_sync_buf_lab)/launch/stereo_cameras.launch">
    <arg name="left_cam_rate"  value="$(arg left_cam_rate)"  />
    <arg name="right_cam_rate" value="$(arg right_cam_rate)" />
  </include>

  <node pkg="ros_sync_buf_lab" type="stereo_sync_sub.py" name="stereo_sync_sub" output="screen">
    <param name="left_topic"  value="/left/image_raw"/>
    <param name="right_topic" value="/right/image_raw"/>
    <param name="queue_size"  value="10"/>
    <param name="slop"        value="0.05"/>
    <param name="use_approx"  value="true"/>
  </node>
</launch>
```

---

## 4. Time Sync Tasks

### Task 4.1 – Run Stereo Sync

```bash
roslaunch ros_sync_buf_lab stereo_sync.launch
```

You should see entries like:

```text
[SYNC] Left seq=12, Right seq=8, dt=0.0349 s
```

> If no messages appear, synchronization failed.

Try increasing the right camera rate:

```bash
roslaunch ros_sync_buf_lab stereo_sync.launch left_cam_rate:=1 right_cam_rate:=5
```

> Does synchronization improve?

---

### Task 4.2 – Exact vs Approximate Sync

Disable approximate sync:

```xml
<param name="use_approx" value="false"/>
```

Run again:

```bash
roslaunch ros_sync_buf_lab stereo_sync.launch
```

> Are there any matches? Why or why not?

Re-enable approximate sync afterwards.

---

### Task 4.3 – Experiment with `slop`

Change:

```xml
<param name="slop" value="..."/>
```

Try: **0.005, 0.02, 0.1, 0.25, 0.5**

Observe:

- Very small → almost no matches
- Moderate → stable matches
- Large → mismatched pairs

> What slop value seems appropriate for your camera rates?

---

### Task 4.4 – Experiment with Queue Size

Modify:

```xml
<param name="queue_size" value="..."/>
```

Try: **1, 5, 25, 100**

Questions:

- Why does `queue_size=1` fail often?
- With very large queues, do the matched seq numbers lag?
- What happens right after startup?

---

### Task 4.5 – Sync With Different Rates

Try:

- (10, 10)
- (10, 5)
- (50, 10)

```bash
roslaunch ros_sync_buf_lab stereo_sync.launch left_cam_rate:=<M> right_cam_rate:=<N>
```

> How often does synchronization occur?

---

### Task 4.6 – Side-by-Side Visualization

Modify `stereo_sync_sub.py`:

1. Convert left/right images to OpenCV
2. Combine them:

   ```python
   combined = np.hstack([left_img, right_img])
   ```

3. Publish to `/stereo/image_raw`
4. Visualize in `rqt_image_view`

> Does the visualization reflect correct synchronization order?

---
