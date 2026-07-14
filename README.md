# ROS2 Autonomous Driving Simulation

> Robot Software course project, Dept. of Computer Software Engineering, Dong-eui University (Section 2, Team 5)
> A ROS 2 based autonomous driving system running in Gazebo: lane following, stop/end line recognition, dynamic obstacle & pedestrian avoidance, and hill driving.

---

## 1. Overview

An autonomous vehicle completes the following missions on a Gazebo simulation track that mimics a real road environment, using a **single camera sensor**:

- Lane keeping via PID control
- Stop line detection → stop within 1 m → resume after a 3-second wait
- Avoiding a **dynamic cube obstacle** (2 m/s) crossing the lane back and forth
- Driving a **hill section**: 10 m uphill + 5 m flat + 10 m downhill, with a stop line mid-slope
- Detecting and avoiding a Gazebo actor-based **pedestrian** (1.1 m/s)
- Detecting the end line (yellow), parking within 1 m, and shutting the system down automatically

Vehicle speed is capped at 6 m/s during driving, with real-time monitoring of speed and lane-departure count.

## 2. Environment & Tech Stack

| Category | Details |
|---|---|
| OS | Ubuntu 20.04.5 LTS (Docker container) |
| Middleware | ROS 2 Foxy |
| Simulator | Gazebo (custom `car_track.world`) |
| Language / Libraries | Python (rclpy), OpenCV (cv_bridge) |
| 3D Modeling | Blender (hill section modeled from scratch → exported as `.dae`, UV texture mapping) |
| IDE | PyCharm 2022.2.1 Community |
| Collaboration | Git / GitHub (private repo), project monitoring tool |

## 3. System Architecture

### 3.1 ROS 2 Node Layout

```
[starter.py] --(Target msg, 'start_car' topic)--> [controller.py]
                                                      │
[camera image] --> [line_follower.py] --(Twist)------┤
        │               └ line_tracker.py (lane center / error calculation)
        │
        └-----> [line_detector.py]
                    ├ stop_line_tracker.py  (white stop line detection)
                    ├ end_line_tracker.py   (yellow end line detection)
                    └ detect_obstacle()     (red obstacle detection)
                              │
                              └--(stop/end/obstacle issue)--> [controller.py] --(Twist)--> Gazebo vehicle

[state_check.py]  : monitors vehicle speed (Twist) and lane-departure count
[obstacle_cube.py]: controls the cube obstacle's reciprocating motion (subscribes Odometry → publishes Twist)
```

| Node / Module | Responsibility |
|---|---|
| `starter.py` | Selects the vehicle to start (PR001/PR002) from a command-line argument and publishes a `Target` message on a 0.5 s timer |
| `state_check.py` | Receives vehicle info → dynamically subscribes to that vehicle's velocity topic (`/demo/<car>_cmd_demo`), logs speed and lane departures |
| `line_tracker.py` | HSV conversion → ROI (top 1/3, bottom 5/6) → Hough transform to detect left/right lanes; computes lane-center error (`_delta`) and departure count (`_invasion`) |
| `line_follower.py` | Derives angular velocity (angular.z) and linear velocity (linear.x) from the tracker output; slows down in turns, speeds up on straights; publishes drive state, velocity, and departure count |
| `line_detector.py` | Subscribes to camera images → publishes stop-line, end-line, and obstacle detection results |
| `stop_line_tracker.py` | White HSV mask + restricted search region + `cv2.moments()` centroid to compute stop-line center and error |
| `end_line_tracker.py` | Yellow HSV mask for end-line detection (same centroid/error scheme) |
| `controller.py` | Integrated control: gentle start at low speed; on stop line, stop 3 s then resume (hold-speed control on the hill stop line to prevent rollback); on end line, stop within 1 m and shut down the node |
| `obstacle_cube.py` | Tracks the cube's Y position via Odometry; reverses direction at `min_y`/`max_y` bounds (−78 to −62); moves with `linear.y = 2.0 × direction` |

### 3.2 Gazebo World (`car_track.world`)

- Two vehicles (PR001, PR002) spawned at the start line
- Cube obstacle: 2 m × 2 m × 2 m, red (ambient/diffuse `1 0 0 1`), initial pose (35, −64, 0.6), ROS plugin (namespace `robot1`, `cmd_vel`/`odom` topics, 1000 Hz update)
- Hill section: Blender-made `pavement_hill.dae` included via `model.sdf` (meshes/textures/model.config structure; Gazebo model path registered in `~/.bashrc`)
- Pedestrian: Gazebo `actor` with `<trajectory>`/`<waypoint>` crossing the lane back and forth (1.1 m/s)
- A white stop line placed 8 m after the uphill starts

## 4. Core Algorithms

### Lane Detection & Driving
1. **HSV conversion** to extract white lane markings
2. **ROI restriction** (top 1/3, bottom 5/6) to reduce computation
3. **Hough Transform** for line detection → split into left/right lanes → compute lane center (falls back to previous-frame values when a lane is not detected)
4. Lane-center vs. vehicle-center error (delta) fed to a **PID controller** → adjusts steering (angular.z) and speed (linear.x)
5. When the error exceeds a threshold, the lane-departure counter increments and a warning is issued

### Stop / End Line Detection
- Stop line (white) and end line (yellow) each extracted with an HSV `cv2.inRange()` mask
- Search-region masking reduces false positives
- `cv2.moments()` computes the centroid (cx, cy) → offset from the image center drives control
- Stop line: 3-second stop, then resume / End line: decelerate → stop within 1 m → `destroy_node()` + `rclpy.shutdown()`

### Obstacle (Red) Detection
- Red spans both ends of the Hue axis in HSV (0–10 and 170–180), so **two masks are combined with `bitwise_or`**
- `cv2.countNonZero()` counts mask pixels → an obstacle is declared at **5,000+ pixels**
- On detection the vehicle decelerates and stops; driving resumes once the obstacle leaves the field of view

### Hill (3D Modeling)
- Modeled in Blender by joining a cuboid with subdivided faces: uphill (10 m) – flat (5 m) – downhill (10 m); total length 24.6 m, height 2 m, width 8 m (a triangle with 10 m hypotenuse and 2 m height requires a 9.8 m base)
- Road texture applied via UV mapping, exported as Collada (`.dae`), and loaded into Gazebo

## 5. Requirements & Implementation Status (SFR)

| ID | Requirement | Status |
|---|---|---|
| SFR-101–105 | Simulator environment, two vehicles spawned, selected-vehicle start, 6 m/s speed cap with monitoring, lane-departure logging | ✅ |
| SFR-201–202 | Stop within 1 m of the stop line, resume after 3 s | ✅ |
| SFR-301–304 | Cube (2×2×2 m) creation, 2 m/s reciprocation, collision-free passage, camera-based detection | ✅ |
| SFR-305 | LiDAR-based obstacle detection | ❌ Removed (see note) |
| SFR-401–404 | Hill section layout, hill stop line, stop & resume | ✅ |
| SFR-501–504 | Gazebo actor pedestrian, reciprocating motion (1.1 m/s), collision avoidance | ✅ |
| SFR-601 | Park within 1 m of the final stop line after completing the full course | ✅ |

> **Why LiDAR was removed**: enabling the LiDAR sensor drastically increased Gazebo loading time, making testing inefficient. Since the camera alone provided sufficient obstacle detection, LiDAR was dropped in favor of the camera.

## 6. Design Artifacts (in the full report)

For each feature (system environment / lane detection & driving / cube obstacle / hill / pedestrian / obstacle recognition / stop-resume-terminate), the report includes:

- Use-case models (main, alternative, and exception flows)
- Domain models and design class diagrams (Vehicle, Controller, LineFollower, LineTracker, LineDetector, EndLineTracker, StopLineTracker, Starter, StateCheck, ObstacleCube, etc.)
- Activity diagrams and communication diagrams
- Data Flow Diagrams (DFD) & Data Dictionaries (DD)
- System operation definitions (input / output / postconditions)

Object-oriented design principles applied:
- **GRASP** — Information Expert (ObstacleCube owns its state and motion data and controls itself)
- **GoF** — Command pattern (motion commands delivered via Twist messages), Strategy pattern (color ranges swappable at runtime via `cv2.inRange`)
- **SOLID** — SRP (obstacle-detection logic isolated), OCP (new colors/algorithms added without modifying existing code), DIP (Gazebo↔ROS coupling isolated behind plugins), and others

## 7. Testing

Specification-based techniques from the ISO/IEC/IEEE 29119 software testing standard (equivalence partitioning, boundary value analysis) were applied; unit-level feature tests were followed by integration testing.

Key test cases (all passed):

| Test | Input | Result |
|---|---|---|
| Vehicle start | `ros2 run ros2_term_project starter PR001` | Vehicle info published and initialized correctly |
| Invalid vehicle input | PR003 (nonexistent) | Error message printed as expected |
| Lane tracking | Camera images | Lane center held; deceleration in turns works |
| Stop line | stop_issue message | Stop → 3 s → resume, as expected |
| Obstacle reciprocation | min_y=−78, max_y=−62 | 2.0 m/s reciprocation within bounds; direction reversal at limits |
| Obstacle avoidance | Observed while driving | Detects and stops (slightly closer to the obstacle than expected — room for improvement); resumes after passage |
| Termination | end_issue message | Stops within 1 m; node and system shut down |

## 8. Team & Roles

| Name | Responsibilities |
|---|---|
| Taeyoung Nam | System environment, lane detection, stop-section implementation / requirements management, meeting minutes, final report, driving video |
| Donghyun Kwak | Obstacle detection implementation / final report |
| Juri Seok | Cube obstacle implementation / meeting minutes, project plan, driving video |
| Chaeyeon Won | Hill implementation (Blender 3D modeling) / report, slides, presentation video |
| Jinhyuk Cho | Pedestrian implementation / report |

## 9. Retrospective (Git Usage)

**What went well**
- Docker unified the development environment across teammates, eliminating environment-mismatch issues
- Git/GitHub (private repo) enabled remote collaboration, change tracking, and code-URL sharing for efficient communication
- `.gitignore` kept build artifacts and logs out of the repo, keeping it lightweight

**What could improve**
- Git LFS was not used for large modeling assets
- Git was adopted mid-project, so some work happened by overwriting the Git folder rather than through it, diluting the benefits of synchronization

## testing video link
https://www.youtube.com/watch?v=TpHQC-PzqVg


## 10. How to Run

```bash
cd ~/Ros2Projects/oom_ws
source install/setup.bash
ros2 run ros2_term_project starter PR001   # or PR002
```

---

*Documentation follows the ISO/IEC/IEEE 29148:2011 requirements-engineering standard and IEEE Std 830-1998 SRS recommended practice. (Submitted: 2024-12-07)*
