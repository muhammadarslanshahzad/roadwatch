# RoadWatch 🚁🛣️

![Drone view](docs/images/frame_00012.jpg)

**A multi-drone highway patrol system that finds road damage (cracks and potholes), pins it with GPS, and reports it to a maintenance HQ, simulated in Webots.**

> Status: 🚧 work in progress. Phase 1 (single-drone flight + road following + image capture) is working.

---

## Why

Long highways like Pakistan's M-2 and N-5 are mostly inspected by hand or fixed only after complaints. Damage gets found late, and a small crack becomes a dangerous pothole.

RoadWatch explores a cheaper approach:

1. A **fleet of drones** splits a highway into segments and patrols them.
2. A **downward camera + YOLO** detects cracks and potholes.
3. Each detection is **GPS-pinned** and sent to **HQ**.
4. HQ gets a map of damage, **sorted by severity**, so crews know what to fix first.

---

## Current status

| Phase | Description | Status |
|---|---|---|
| 0 | Highway world in Webots (1.5 km road, scenery, traffic, work zone) | ✅ done |
| 1 | Single drone: stable altitude hold, follows the road, saves frames + poses | ✅ done |
| 2 | Real road damage in the world (textures from UAV-PDD2023) + ground truth | ⏳ next |
| 3 | YOLO26 trained on drone road-damage data, tested on sim frames | 🔜 |
| 4 | Detection → GPS pin → message to HQ | 🔜 |
| 5 | 3-drone fleet with segment assignment | 🔜 |
| 6 | Static vs dynamic segment splitting experiments | 🔜 |
| 7 | HQ dashboard, demo video | 🔜 |

**Measured so far (phase 1):** the drone holds **25.0 m** altitude and cruises at **~5 m/s**, staying within **~0.5–2.2 m** of the road centerline (road width is 8 m).

---

## The world

`worlds/highway.wbt`, built for Webots R2025a:

- **Road:** 1.5 km, 2 lanes, 8 m wide, S-shaped (two gentle bends), crash barriers on both sides
- **Scenery:** tree clumps near the road, background forest patches, rocks, a roadside rest stop
- **Traffic:** parked cars and a truck on the lanes, used to test occlusion (a vehicle hiding damage)
- **Work zone:** traffic cones + oil barrels
- **Drone:** DJI Mavic 2 Pro (`drone_1`) with a gimbal camera pointed straight down

---

## How the drone controller works

`controllers/drone/drone.py`, an extern Python controller:

### Flight
- **Attitude (roll/pitch):** P control on the angle + gyro rate as damping, with values taken from the official Webots Mavic controller.
- **Altitude:** a cascaded loop. The altitude error sets a target climb speed (capped at ±2 m/s), and the climb-speed error sets thrust. This replaced the official single-loop controller, which overshot by ~18 m at a 25 m target.
- **Motor mixing:** base thrust plus roll, pitch and yaw corrections, with diagonal props spinning in opposite directions so the torques cancel.

### Road following
- **Exact road centerline:** a Python port of Webots' own B-spline (`wbgeometry.bSpline3`), so the drone follows the same curve the `Road` node draws.
- **Pure pursuit:** the drone steers (yaw) toward a point 15 m ahead on the road.
- **Speed control:** it tilts forward to cruise at 5 m/s, turns before it accelerates, and slows down near the end.
- **States:** `TAKEOFF → FOLLOW_ROAD → ARRIVED`

### Image capture
- Saves a JPG **every 10 m** of travel (about 35% overlap between frames at 25 m altitude).
- Logs the drone's **pose** for each frame (`x, y, alt, roll, pitch, yaw`) to a CSV. Later this is used to turn a detection's pixel location into a position on the road.

---

## Datasets (planned)

| Role | Dataset | Why |
|---|---|---|
| Main training | **UAV-PDD2023** | Drone images taken at 30 m, top-down, same view as our drone |
| Extra training | **RDD2022 China_Drone subset** | Also top-down drone images |
| Hard test set | **HighRPD** | High-altitude drone images, to test robustness at other altitudes |
| Pakistan sanity check | **PathCare** (Mehran UET) | Pakistani roads, used to measure the domain gap |

**Classes:** `crack`, `alligator_crack`, `pothole`

**Model:** Ultralytics **YOLO26** (`s` for training, `n` for running inside the sim)

---

## Research angle 🎓

- **Fleet coordination:** static vs dynamic road splitting (e.g. battery handoff, extra drones for damage-dense areas). Measured by time to full coverage, % of damage found, and false pins.
- **Sim-to-real:** does adding auto-labelled sim images help or hurt detection on real roads?
- **Domain gap:** training data is mostly Chinese roads. How well does it transfer to Pakistani roads?

---

## Project structure

```
roadwatch/
├── worlds/
│   └── highway.wbt          # the Webots world
├── controllers/
│   └── drone/
│       └── drone.py         # drone flight + road following + image capture
├── protos/                  # custom PROTOs (RoadDamage.proto coming in phase 2)
├── textures/                # damage textures (phase 2)
├── tools/                   # helper scripts (dataset cropping, etc.)
└── data/                    # datasets + captured frames (not in git, too big)
    └── sim_frames/run_<timestamp>/
        ├── frame_00000.jpg
        └── log.csv
```

---

## Setup

**Requirements**
- Windows 10/11
- [Webots R2025a](https://cyberbotics.com/)
- Python 3.10–3.12 (Windows install, **not** WSL)
- VSCode (optional, recommended)

**VSCode autocomplete for the Webots API:** add this to `.vscode/settings.json`:
```json
{
  "python.analysis.extraPaths": ["C:/Program Files/Webots/lib/controller/python"]
}
```

---

## Run it

1. Open `worlds/highway.wbt` in Webots and press play ▶️.
2. In **PowerShell** (not WSL), from the project root:
   ```powershell
   & "C:\Program Files\Webots\msys64\mingw64\bin\webots-controller.exe" --robot-name=drone_1 controllers\drone\drone.py
   ```
3. Watch the camera live in Webots: **Overlays → Camera Devices → drone_1: camera**
4. Frames and poses are saved to `data/sim_frames/run_<timestamp>/`.

> Note: the world must use `gpsCoordinateSystem "local"` so the GPS returns meters. Lat/lon conversion happens later, at HQ.

---

## Limitations / honest notes

- Simulation only. A real highway deployment would need regulatory approval for drone flights (BVLOS).
- The sim asphalt is much cleaner than real roads, so domain randomization is planned.
- Camera tilt during forward flight is logged, but not yet corrected for when pinning damage.

---

## Author

**Muhammad Arslan Shahzad**, AI Engineer

Built with Webots, Python and a lot of crashed drones. 🪦🚁