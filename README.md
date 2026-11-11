# ogccdb-viewer

A 3D desktop viewer for OGC CDB 1.3 Elevation and Imagery: a globe you can fly over, with
terrain streamed by LOD from a CDB store. C++17, OpenGL 3.3 core (runs on any 4.x driver),
GLFW and Dear ImGui. All CDB logic (LOD selection, tile loading, meshes) lives in
[ogccdb-runtimepublisher](https://github.com/fyrdm/ogccdb-runtimepublisher), on top of
[ogccdb-core](https://github.com/fyrdm/ogccdb-core); this repo only renders. MIT licence.

## Build (Ubuntu/Debian)

```bash
scripts/setup-ubuntu.sh     # once: GLFW and OpenGL headers (sudo)
scripts/build-all.sh        # builds core, runtime and viewer from sibling clones
```

`build-all.sh` expects (or clones) `ogccdb-core` and `ogccdb-runtimepublisher` next to this
repo, installs the libraries to `~/ogccdb-prefix` (`OGCCDB_PREFIX` overrides it) and builds
the viewer in `build-release/`. By hand:

```bash
cmake -B build -S . -G Ninja -DCMAKE_PREFIX_PATH=$HOME/ogccdb-prefix
cmake --build build
```

Dear ImGui (v1.91.9) and stb (pinned commit) are fetched at configure time.

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

Free fly goes anywhere, also below the terrain: seen from below, tiles are drawn darker
and the skirts show as thin walls. Mouse sensitivity and invert Y are in the panel.
**H** shows or hides the panel.

The panel shows the store, camera, render options (imagery, lighting, haze, wireframe, LOD
colours, vertical exaggeration, screen-space error, max LOD) and streaming statistics.

`--help` lists all options.
