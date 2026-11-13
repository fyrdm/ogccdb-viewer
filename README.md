# ogccdb-viewer

A 3D desktop viewer for OGC CDB 1.3 Elevation and Imagery: a globe you can fly over, with
terrain streamed by LOD from a CDB store. C++17, OpenGL 3.3 core (runs on any 4.x driver),
GLFW and Dear ImGui. All CDB logic (LOD selection, tile loading, meshes) lives in
[ogccdb-runtimepublisher](https://github.com/fyrdm/ogccdb-runtimepublisher), on top of
[ogccdb-core](https://github.com/fyrdm/ogccdb-core); this repo only renders. MIT licence.

| Folder | What it is |
| --- | --- |
| `src/` | the viewer application (window, panels, cameras) |
| `render/` | library `ogccdb::render`: draws the terrain with OpenGL, into a window or off screen |
| `flight/` | library `ogccdb::flight`: reads and writes ROS 2 bags without ROS, builds the flight track |
| `tools/bag-render/` | `ogccdb-bag-render`: adds a simulated camera to a recorded flight (below) |

## Build (Ubuntu/Debian)

```bash
scripts/setup-ubuntu.sh     # once: GLFW, OpenGL and SQLite headers (sudo)
scripts/build-all.sh        # builds core, runtime and viewer from sibling clones
```

`build-all.sh` expects (or clones) `ogccdb-core` and `ogccdb-runtimepublisher` next to this
repo, installs the libraries to `~/ogccdb-prefix` (`OGCCDB_PREFIX` overrides it) and builds
the viewer in `build-release/`. By hand:

```bash
cmake -B build -S . -G Ninja -DCMAKE_PREFIX_PATH=$HOME/ogccdb-prefix
cmake --build build
```

Dear ImGui (v1.91.9) and stb (pinned commit) are fetched at configure time. SQLite is needed
for `.db3` bags; zstd and lz4 are optional and only needed for compressed MCAP bags.

## Run

```bash
build-release/ogccdb-viewer ~/ogccdb-core/out/sample/CDB
build-release/ogccdb-viewer <cdb> --lat 47.12 --lon 8.75 --distance 25000 --pitch -18 --heading 160
build-release/ogccdb-viewer <cdb> --fly --lat 47.1 --lon 8.62 --height 300 --pitch 25   # underground
build-release/ogccdb-viewer <cdb> --screenshot view.png --no-ui   # render, save, exit
```

Two camera modes; **V** (or the panel) switches between them.

| Orbit (default) | |
| --- | --- |
| Left drag / WASD | pan |
| Right or middle drag / Q E R F | rotate and tilt |
| Wheel / + - | zoom |

| Free fly (like a first-person game) | |
| --- | --- |
| Mouse | look around; the cursor is captured (click the view or Tab to capture, Esc or Tab to release) |
| W S A D | forward, back, left, right |
| E or Space / Q or C | up / down |
| Shift / Ctrl | 5x faster / 5x slower |
| Wheel | speed (scales with height above ground) |
| G | walk on the ground at a set eye height (1.8 m), or fly again |
| Arrow keys | look around without the mouse |
| Hold right button | zoom 4x (binoculars), with slower look |

The field of view (5–120°, vertical; default 45°) is in the panel, on `[` `]`, and on
`--fov`. LOD selection follows it, so a narrow view loads finer tiles in the distance.

### Sea

Where the store has no tiles, a sea is drawn: an imaginary reference surface that, like
mean sea level in geodesy, runs on under the terrain and shows wherever the terrain is
lower or missing. Its height above the ellipsoid is the geoid (EGM2008 if its PROJ grid is
installed, else EGM96) or the ellipsoid itself, plus an offset you can raise to flood the
terrain. Pick the reference that matches the store's heights: the geoid for ellipsoid
heights, the ellipsoid for heights above sea level written as they are. A 1° graticule is
drawn on it.

```bash
build-release/ogccdb-viewer <cdb> --sea ellipsoid --sea-offset 430   # flood to 430 m
build-release/ogccdb-viewer <cdb> --sea off                          # dark background globe
```

The sea moves: a wind sea of 24 travelling waves, from a dominant wave whose length grows
with the wave height down to one-metre ripples, each moving at its deep-water speed
(c = sqrt(g / k)). The longer waves lift the surface near the eye; all of them shade it,
each fading out before it gets smaller than a pixel, with sun glitter and some foam on the
crests of a rough sea. **Waves**, **Wave height** (crest to trough) and **Wind from** are in
the panel; `--waves <m>` (0 = calm, default 1.5) and `--wind <deg>` set them at start.
`--screenshot f.png --frames 150` saves 5 s of frames at 30 fps (`f_0000.png` ...) for a clip.

`--sea` takes `auto` (default), `egm2008`, `egm96`, `ellipsoid` or `off`. The geoid comes
from PROJ grids (`proj-data`, or `PROJ_NETWORK=ON`); without one the sea falls back to the
ellipsoid and the panel says so.

Free fly goes anywhere, also below the terrain: seen from below, tiles are drawn darker
and the skirts show as thin walls. Mouse sensitivity and invert Y are in the panel.
**H** shows or hides the panel.

The panel shows the store, camera, render options (imagery, lighting, haze, wireframe, LOD
colours, sea and graticule, vertical exaggeration, screen-space error, max LOD) and streaming statistics.

`--help` lists all options.

## Flight playback (ROS 2 bags)

The viewer plays back a drone flight recorded as a ROS 2 bag, and can look through a camera
mounted under the drone.

```bash
build-release/ogccdb-viewer <cdb> --bag ~/bags/nav_data               # a bag folder, .mcap or .db3
build-release/ogccdb-viewer <cdb> --bag ~/Downloads/nav_data.tar      # a bag in a .tar, as shipped
build-release/ogccdb-viewer <cdb> --bag ~/bags                        # a folder of bags: pick one
build-release/ogccdb-viewer <cdb> --bag ~/bags/nav_data --bag-time 90 --play --speed 4
```

**Open bag...** in the panel browses for a bag. A bag is a folder with `metadata.yaml` and
`.mcap` or `.db3` files (rosbag2's MCAP and SQLite storage), one such file, or an
uncompressed `.tar` holding a bag. MCAP is read in place inside the tar; `.db3` files are
copied out once to a temporary folder, since SQLite needs a real file. Compressed archives
(`.tar.gz`, `.zip`, ...) have to be extracted first. ROS is not needed: the bag is read
directly and its CDR messages decoded.

Messages used (one topic of each is picked automatically; `ogccdb-bag-info` shows the choice):

| Message | Used for |
| --- | --- |
| `sensor_msgs/NavSatFix` | geodetic position (ellipsoid height) |
| `geometry_msgs/PoseStamped`, `PoseWithCovarianceStamped`, `nav_msgs/Odometry` | local position (ENU) and attitude |
| `sensor_msgs/Imu`, `geometry_msgs/QuaternionStamped` | attitude, if there is no pose |
| `geographic_msgs/GeoPoseStamped` | position and attitude |

Attitude is a quaternion from the body frame (x forward, y left, z up) to the local ENU
frame. With a local pose the track uses it (smooth, high rate) and anchors it to the fixes
(the median offset between the two); otherwise it uses the fixes alone. Between messages,
position is interpolated linearly and attitude by slerp.

| Flight panel / keys | |
| --- | --- |
| P | play / pause (speed 0.1–32x, loop, time slider) |
| B | drone camera on / off |
| Left / Right | seek 5 s back / forward (drone camera) |
| Camera tilt | -90° straight down (default), 0° forward; `--camera-tilt` |
| Path / Follow | draw the track (flown part bright) / keep the orbit centred on the drone |

The drone camera sits under the drone, aligned with its body: image up is the drone's nose,
and the picture rolls and pitches with it. It uses the panel's field of view. Outside the
drone camera the drone is drawn as a small cross with a red nose, with the camera's footprint
on the ground. The panel shows position, height above ground, heading/pitch/roll, speeds and,
per topic, the messages played so far and their rate.

**Altitude** decides how flight heights meet the terrain: *As recorded*; *Start on ground*
(shift all heights so the first pose sits on the terrain, chosen automatically when the flight
starts at rest, because a GNSS height and the terrain's height model often disagree by tens
or hundreds of metres); or a manual *Offset*.

`ogccdb-bag-info <bag> [seconds...]` prints the bag's topics, the track it builds and the pose
at the given times.

## Camera feed for a recorded flight (`ogccdb-bag-render`)

`ogccdb-bag-render` takes a bag with a flight's positions and attitudes but no camera, flies
it over a CDB store, and writes a copy of the bag with what a camera on the vehicle would
have seen. It is meant for developing vision-based localization against CDB data before a
recording with a real camera exists: the new bag plays like any other, and side files hold
the truth to compare with. It opens no window.

```bash
build-release/ogccdb-bag-render <cdb> ~/bags/nav_data ~/bags/nav_data_camera
build-release/ogccdb-bag-render <cdb> <bag> <out> --tilt -45 --hfov 70 --rate 5
build-release/ogccdb-bag-render <cdb> <bag> <out> --realistic --distortion=-0.2,0.05,0,0,0
build-release/ogccdb-bag-render <cdb> <bag> <out> --from 60 --to 120 --truth-every 1
build-release/ogccdb-bag-render <cdb> <bag> <out> --drop /mavros/global_position/global
```

The output folder is the new bag (`metadata.yaml` and one `.mcap`), with a `truth/` folder
inside it:

| In the new bag | |
| --- | --- |
| every topic of the input | copied as it is (`--drop <topic>` leaves one out) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage`, JPEG, 10 Hz by default |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` for each image: K, plumb_bob D, R = identity, P = [K \| 0] |
| `/tf_static` | the mount: `base_link` to `camera_optical_frame`, once at the start |
| `/prior/geo_pose` | `geographic_msgs/GeoPoseStamped` for each frame: the drifting prior (below) |

Images, camera_info and prior of one frame carry the same header stamp. Stamps are on the
clock of the input's own header stamps, and the messages are filed at the time a recorder
would have received them. Topic names and frame ids can be changed (`--image-topic`,
`--info-topic`, `--prior-topic`, `--camera-frame`, `--body-frame`).

**Camera.** A pinhole camera: `--size` and `--hfov`, or `--intrinsics fx,fy,cx,cy` in the
camera_info convention (pixel centres at whole numbers, (0, 0) the centre of the top-left
pixel). `--distortion k1,k2,p1,p2,k3` adds lens distortion: the scene is rendered as a wider
ideal image and resampled through the lens model, so the image, the camera_info and the
truth agree. The mount is given on the body (x forward, y left, z up): `--tilt` (-90 straight
down with the image top toward the nose, 0 forward), `--pan` (positive turns the camera
left), `--roll` (about the viewing direction) and `--mount-offset x,y,z`. The image's frame
is the optical one: x right, y down, z into the scene.

**Image realism.** A render of the map's own imagery is an easy target for a matcher; these
make it less so. They are applied in this order, and all are off by default:

| Option | |
| --- | --- |
| `--blur <px>` | lens blur (Gaussian sigma) |
| `--vignette <0..1>` | how much darker the corners are |
| `--gains r,g,b`, `--brightness`, `--contrast`, `--saturation`, `--gamma` | colour |
| `--noise <levels>` | sensor noise, sigma in 8-bit levels |
| `--jpeg-quality <1..100>` | compression (default 90) |
| `--realistic` | a moderate amount of each; options after it adjust it |

**Prior pose.** A localizer needs a rough idea of where it is, and must not be handed the
answer. `/prior/geo_pose` is the true pose plus an error that grows as dead reckoning
without GNSS does: on each horizontal axis the velocity error is a random walk
(`--drift-velocity`, m/s per sqrt(s); the position error's standard deviation is that times
t^1.5 / sqrt(3): about 30 m after 5 minutes at the default 0.01), height and heading errors
are random walks (`--drift-height`, `--drift-yaw`), the heading starts off by `--yaw-bias`
degrees, roll and pitch get `--tilt-noise`, and `--start-error` puts the first position off
by so many metres. `--seed` repeats a run exactly. The error is not reset by anything: a
localizer should take the prior's changes from frame to frame, as it would from odometry.
The input's own position topics are still in the bag unless dropped.

**Truth.** In `truth/`:

| File | |
| --- | --- |
| `frames.csv` | one row per frame time: the vehicle's true position (`lat`, `lon`, `height` above the ellipsoid) and attitude (`qx..qw`, body to local ENU), the camera in the local frame (`cam_e`, `cam_n`, `cam_u`, and `cam_qx..cam_qw`, optical frame to local frame), the prior and its error, the height above the terrain, whether an image was written |
| `camera.json` | intrinsics, mount, the local frame's origin, and every setting of the run |
| `000120.npy`, `000120.jpg` | for every n-th image (`--truth-every`, default 10): the ground point seen by each pixel, float32 of shape (height, width, 3), east, north, up in metres in the local frame, NaN where there is no terrain; and the image itself. The number is the `frame` column. |

The local frame is east, north, up at the start of the flight (a plane tangent to the
ellipsoid there). A ground-point file is 12 bytes a pixel (11 MB at 1280x720), so write
them for every frame only over a short `--from`/`--to` range. With them a match between an
image pixel and the map can be checked directly, and solving the camera pose from the
pixels and their ground points gives back `cam_*` of that row:

```python
import csv, json, numpy as np, cv2
cam = json.load(open("truth/camera.json"))["camera"]
K = np.array([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1]])
row = next(r for r in csv.DictReader(open("truth/frames.csv")) if r["xyz"])
xyz = np.load("truth/" + row["xyz"])                  # (height, width, 3)
v, u = np.nonzero(np.isfinite(xyz[..., 0]))
pick = np.random.default_rng(0).choice(len(u), 2000, replace=False)
ok, rvec, tvec = cv2.solvePnP(xyz[v[pick], u[pick]].astype(float),
                              np.stack([u[pick], v[pick]], 1).astype(float), K, np.array(cam["d"]))
print((-cv2.Rodrigues(rvec)[0].T @ tvec).ravel(), row["cam_e"], row["cam_n"], row["cam_u"])
```

**Heights.** `--altitude` works as in the viewer: `auto` (default) shifts a flight that
starts at rest so that it starts on the terrain, `ground` always does, `recorded` never,
and a number adds that many metres. Everything written (images, truth, prior) uses the
shifted heights; the copied input topics keep theirs. Frames closer than `--min-agl`
(default 1 m) to the terrain are left out, so a vehicle standing on the ground sends none.

It needs OpenGL 3.3 and a display for its (hidden) window; on a machine without one, run it
under a virtual display (`xvfb-run -a build-release/ogccdb-bag-render ...`). What it cannot
give: the light, weather and season of another day, moving things, or anything the store's
imagery and elevation do not hold. `--help` lists all options.
