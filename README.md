# Fisheye Camera Calibration

A robust fisheye camera calibration tool using OpenCV with support for multiple camera models including **Double Sphere** for ultra-wide FOV lenses (180°+).

## Features

- **Multiple camera models**: KB4, KB8, OCam, Double Sphere (DS), EUCM
- **Two-stage outlier rejection**: Automatically removes bad frames for better accuracy
- **Video-based calibration**: Extract frames from video automatically
- **Per-view diagnostics**: Shows reprojection error distribution
- **Ultra-wide FOV support**: Double Sphere model for 180°-220° FOV lenses

## Supported Camera Models

| Model | Best For | Parameters |
|-------|----------|------------|
| `KB4` | FOV < 180° | fx, fy, cx, cy, k1-k4 |
| `KB8` | FOV < 180° | fx, fy, cx, cy, k1-k8 |
| `DS` (Double Sphere) | **FOV 180°-220°** | fx, fy, cx, cy, xi, alpha |
| `EUCM` | FOV 150°-200° | fx, fy, cx, cy, alpha, beta |
| `OCam` | General fisheye | Polynomial coefficients |

## Prerequisites

- C++17 compatible compiler
- CMake >= 3.14
- OpenCV 4.x (with `fisheye` module)

## Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

1. Prepare your configuration file `config.json`:

```json
{
    "video_path": "calibration_video.mp4",
    "board_width": 9,
    "board_height": 6,
    "square_size": 0.025,
    "output_file": "calibration_results.yaml",
    "undistorted_image_output": "undistorted_sample.jpg",
    "output_model": "DS",
    "show_detection": true,
    "frame_step": 10,
    "max_frames": 80,
    "outlier_reject_threshold_px": 10.0,
    "outlier_reject_top_fraction": 0.2
}
```

2. Run the calibration:

```bash
./fisheye_calib config.json
```

## Configuration Parameters

| Parameter | Description |
|-----------|-------------|
| `video_path` | Path to input video with checkerboard views |
| `board_width` | Number of inner corners per row |
| `board_height` | Number of inner corners per column |
| `square_size` | Physical size of a square in meters |
| `output_file` | Output YAML file for calibration results |
| `output_model` | `KB4`, `KB8`, `OCam`, `DS`, or `EUCM` |
| `show_detection` | Show corner detection visualization |
| `frame_step` | Process every Nth frame |
| `max_frames` | Maximum number of frames to use (30-100 recommended) |
| `outlier_reject_threshold_px` | Keep views with RMS <= threshold (-1 to disable) |
| `outlier_reject_top_fraction` | Drop worst fraction of views (0.0-1.0) |

## Output

The calibration produces:

1. **YAML calibration file** containing:
   - Camera matrix K (fx, fy, cx, cy)
   - Distortion coefficients
   - RMS and mean reprojection error
   - Model-specific parameters

2. **Undistorted sample image** for visual verification

## Example Output

```
Stage-1 RMS (OpenCV): 392.1 pixels
--- Per-View Reprojection RMS (Stage-1) ---
Min: 1.27 px, Max: 991.1 px, Avg: 262.0 px
Distribution: <1px: 0, 1-5px: 41, 5-10px: 0, >10px: 39

Stage-2 recalibration using 41 inlier views...
Stage-2 RMS (OpenCV): 0.586 pixels

Double Sphere Parameters:
  fx=581.2, fy=581.7, cx=942.3, cy=568.7
  xi=0, alpha=0.65
```

## Tips for Good Calibration

1. **Checkerboard visibility**: Ensure the entire checkerboard is visible in each frame
2. **Variety of poses**: Move the board to different positions and angles
3. **Cover the FOV**: Include views from center and edges of the image
4. **Good lighting**: Avoid motion blur and ensure clear corner detection
5. **Enough frames**: Use 30-100 frames for reliable results

## License

MIT License
