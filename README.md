# Time Synchronization Lab

This lab introduces two key concepts in ROS perception pipelines:

1. **Fake Camera Publisher** — generating synthetic image streams and inspecting ROS topics.
2. **Time Synchronization** — synchronizing messages from multiple sensors using `message_filters`.

The lab consists of **two parts**. Complete them in order.

---

# 📘 Lab 1 — Fake Camera Publisher

## 1. Getting Started

### Task 1.1 — Run the Fake Camera Publisher

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
- Images containing a **large digit** (frame counter).
- One frame per second.

---

## 2. Inspecting ROS Topics

### Task 1.2 — Explore the Published Topic

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

### Task 2.1 — View the Fake Camera Output

Run:

```bash
rqt_image_view
```

Select `/basler/image_raw`.

> What is shown in the image?
> How quickly does the digit change?

---

## 4. Experiment with Publishing Rate

### Task 3.1 — Try Higher Rates

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

### Task 4.1 — Start Two Camera Streams

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

# 📘 Lab 2 — Time Synchronization

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

### Task 4.1 — Run Stereo Sync

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

### Task 4.2 — Exact vs Approximate Sync

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

### Task 4.3 — Experiment with `slop`

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

### Task 4.4 — Experiment with Queue Size

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

### Task 4.5 — Sync With Different Rates

Try:

- (10, 10)
- (10, 5)
- (50, 10)

```bash
roslaunch ros_sync_buf_lab stereo_sync.launch left_cam_rate:=<M> right_cam_rate:=<N>
```

> How often does synchronization occur?

---

### Task 4.6 — Side-by-Side Visualization

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

# ✔ End of Lab

```shell
rosrun ros_sync_buf_lab fake_camera_pub.py --frame basler --rate 1
```
You should see:
- A new topic appearing in ROS.
- Images containing a number (frame counter).
- One frame per second.

### Task 1.2. Inspect topics

Topics discovery:
```shell
rostopic list
```

> What is the output? Can you see the fake camera's topic?


Topic info (publisher, subscribers, type):
```shell
rostopic info <specify the topic ...>
```

> What type of messages it publishes?

Message definition:
```shell
rosmsg info <specify the message type>
```

> What are the fields in the message?

Check publishing rate:
```shell
rostopic hz <specify the topic ...>
```

> What is the publishing rate? Is it exactly the same as we set earlier? Why?

Inspect header field:
```shell
rostopic echo <speicify the topic ...>/header
```

### 2. Visualization

#### Task 2.1. Visualize Images

Open image viewer:
```shell
rqt_image_view
```
and select the topics with fake camera's images.

> What is depicted in the images? How fast the frame changes (approximately)?

### 3. Experimenting with Rate

#### Task 3.1. Increase Publish Rate

Terminate the fake camera publisher (`Ctrl + C`) and run the fake camera publisher with different rates `N` (try 10, 50, 100, 500 Hz):
```shell
rosrun ros_sync_buf_lab fake_camera_pub.py --frame basler --rate <N>
```

Observe:
1. `rostopic hz` output
2. Check CPU load (in System Overview or `htop`)

### 4. Running two Cameras

#### Task 4.1. Start two Publishers

Left camera:
```shell
rosrun ros_sync_buf_lab fake_camera_pub.py --frame basler --rate 1
```

Right camera:
```shell
rosrun ros_sync_buf_lab fake_camera_pub.py --frame elp --rate 2
```

- Check the output with `rostopic list` and the publish rate with `rostopic hz`.
- Visualize both topics in `rqt_image_view`.
- Compare timestamps (depicted numbers).

> Which publisher is faster? Why?

## Lab 2: Time Synchronization

### 1. Overview

> **Why synchronize data?**

Robots fuse data from multiple sensors:
- stereo cameras (two or more cameras)
- camera + IMU
- LiDAR + camera

Each sensor publishes independently → timestamps drift.

**Solution:** ROS provide time synchronization API (message filters).

#### Exact Sync (TimeSynchronizer)

- Requires identical header timestamps
- Almost never succeeds for real sensors
- Only useful when sensors are hardware-triggered or simulated

#### Approx Sync (ApproximateTimeSynchronizer)

- Allows timestamp difference up to a threshold (`slop` parameter)
- Useful for real cameras with jitter (turbulence) or different rates

#### Queue Size

Queue size directly influences how many messages are kept from each synchronized topic while trying to find a match.
- Too small → matches missed
- Too large → higher latency

### 2. Run a Launch File

Inspect and run an existing launch file:
```shell
roslaunch ros_sync_buf_lab stereo_cameras.launch
```

Optionally, provide publishing rate arguments: `left_cam_rate` and `right_cam_rate`:
```shell
roslaunch ros_sync_buf_lab stereo_cameras.launch left_cam_rate:=1 right_cam_rate:=10
```

Terminate the process.

### 3. Prepare Time Sync Launch File

Now create a launch file: `launch/stereo_sync.launch`:

```xml
<launch>
  <arg name="left_cam_rate" default="1">
  <arg name="right_cam_rate" default="1">

  <include file="$(find ros_sync_buf_lab)/launch/stereo_cameras.launch">
    <arg name="left_cam_rate" value="$(arg left_cam_rate)" />
    <arg name="right_cam_rate" value="$(arg right_cam_rate)" />
  </include>

  <node pkg="ros_sync_buf_lab" type="stereo_sync_sub.py" name="stereo_sync_sub" output="screen">
    <param name="left_topic"  value="/left_camera/image_raw"/>
    <param name="right_topic" value="/right_camera/image_raw"/>
    <param name="queue_size"  value="10"/>
    <param name="slop"        value="0.05"/>
    <param name="use_approx"  value="true"/>
  </node>
</launch>
```

### 4. Tasks

#### Tasks 4.1. Run Stereo Sync

Run the launch file:
```shell
roslaunch ros_sync_buf_lab stereo_sync.launch
```

Expected output (numbers might be different):
```shell
[SYNC] Left seq=12, Right seq=8, dt=0.0349 s
```

If it does not output anything, then it time synchronizer cannot find matches.

Try increasing right camera's rate (`right_cam_rate`): does matching improve?

#### Task 4.2. Exact Sync

Change the `use_approx` to `false` in the launch file:
```xml
<param name="use_approx" value="false"/>
```

Run the modified launch file:
```shell
roslaunch ros_sync_buf_lab stereo_sync.launch
```

Expected output:
- Very few matches, because timestamp rarely match exactly

Change back `use_approx` to `true`.

#### Task 4.3. Effect of Slop

Slop works only for approximate timestamp synchronizer.

Try different slop values `{0.005, 0.02, 0.1, 0.25, 0.5}` (in asceding order):
```xml
<param name="slop"        value="..."/>
```

Observe:
- For very low slop → no matches
- For moderate slop → good matches
- For large slop → mismatched pairs (timestamps too far apart!)

#### Task 4.4. Effect of Queue Size

Try different values of `queue_size` (1, 5, 25, 100):
```xml
<param name="queue_size"  value="..."/>
```

Questions:
- With `queue_size = 1` why there are only few matches?
- With large queues, do you observe latency? Compare the sequence numbers (`header.seq`) - what is published to the topics vs what is matched.
- What happens during startup? Waiting for long for the first match?


#### Task 4.5. Different Rates

Try the following combinations of publishing rates (`(M, N) = {(10, 10), (10, 5), (50, 10)}`):
```shell
roslaunch ros_sync_buf_lab stereo_sync.launch left_cam_rate:=<N> right_cam_rate:=<M>
```

> How often sync happens (matching occurs)?


#### Task 4.6. Side-by-Side visualization

Edit the `scripts/stereo_sync_sub.py`:
1. Conver the left and right images to OpenCV format.
2. Merge the images into one (left and right side-by-side).
3. Publish the resulting image to the `/stereo/image_raw` topic.
4. Visualize the image in `rqt_image_view`.
