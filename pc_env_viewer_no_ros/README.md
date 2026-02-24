# pc_env_viewer

Standalone desktop viewer for `.pcd` and `.ply` point-cloud files.

## Features

- Offline file-based point cloud loading (`.pcd`, `.ply`)
- Embedded 3D viewer with orbit/pan/zoom (PCL + VTK)
- Adaptive preview downsampling for large clouds
- Color modes: RGB, Intensity, Height
- Rendering controls: point size and background color
- Cloud stats: raw points, preview points, bounds, fields
- Drag-and-drop file loading
- Screenshot export (`.png`)
- Preview export (`.pcd`)
- Headless load-check mode for CI (`--headless-check`)

## Build

```bash
cd ~/ros2_ws/src/pc_env_viewer
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
./build/pc_env_viewer
```

Auto-load a file:

```bash
./build/pc_env_viewer --file /path/to/cloud.pcd
```

With custom preview cap:

```bash
./build/pc_env_viewer --max-preview-points 1500000
```

Headless check:

```bash
./build/pc_env_viewer --headless-check --file /path/to/cloud.pcd
```

## CLI

```text
pc_env_viewer \
  [--file <path>] \
  [--max-preview-points <int>] \
  [--bg <r,g,b>] \
  [--point-size <float>] \
  [--headless-check]
```

## Notes

- Default file picker path: `/tmp/vertical_mapper_exports` if it exists, otherwise `$HOME`.
- Default preview limit: `1,500,000` points.
- Recommended for Linux workstations with Qt + PCL + VTK installed.
