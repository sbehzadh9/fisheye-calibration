# Fisheye Camera Calibration

A robust fisheye camera calibration tool using OpenCV with support for multiple camera models including **Double Sphere** for ultra-wide FOV lenses (180°+).

## Features

- **Multiple camera models**: KB4, KB8, OCam (Scaramuzza), Double Sphere (DS), EUCM
- **Intelligent frame selection**: Automatically selects diverse frames for optimal FOV coverage
- **Two-stage outlier rejection**: Removes bad frames for better accuracy
- **FOV coverage analysis**: Visual grid showing calibration coverage across the image
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
| `OCam` | General fisheye | pol_data, invpol_data, affine params (Scaramuzza format) |

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
    "output_model": "OCam",
    "show_detection": true,
    "frame_step": 5,
    "max_frames": 0,
    "outlier_reject_threshold_px": 10.0,
    "outlier_reject_top_fraction": 0.2,
    "smart_frame_selection": true,
    "grid_divisions": 5,
    "min_frames_per_cell": 2,
    "min_orientation_diff": 15.0
}
```

2. Run the calibration:

```bash
./fisheye_calib config.json
```

## Configuration Parameters

### Basic Parameters

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
| `max_frames` | Maximum frames to use (0 = unlimited) |

### Outlier Rejection

| Parameter | Description |
|-----------|-------------|
| `outlier_reject_threshold_px` | Keep views with RMS <= threshold (-1 to disable) |
| `outlier_reject_top_fraction` | Drop worst fraction of views (0.0-1.0) |

### Intelligent Frame Selection

| Parameter | Description |
|-----------|-------------|
| `smart_frame_selection` | Enable/disable intelligent frame selection |
| `grid_divisions` | Divide image into NxN grid for FOV coverage (default: 5) |
| `min_frames_per_cell` | Target minimum frames per grid cell (default: 2) |
| `min_orientation_diff` | Minimum board orientation difference in degrees (default: 15) |

## Intelligent Frame Selection

When `smart_frame_selection` is enabled, the calibration tool:

1. **Divides the image into a grid** (e.g., 5x5 = 25 cells)
2. **Analyzes each detected checkerboard** for:
   - Position (which grid cell the centroid falls in)
   - Orientation (rotation angle of the board)
   - Tilt (perspective distortion)
   - Size (proxy for distance from camera)
3. **Rejects similar frames** that don't add new information
4. **Reports FOV coverage** showing how well each region is covered

### Example Output

```
=== FOV Coverage Analysis ===
Grid coverage (frames per cell):
    0   0   0   0   0 
    1   7   4   2   1 
    3  10  25   8   4 
    2  10   7   1   1 
    0   0   0   0   0 
Cells with data: 15/25 (60%)
Well-covered cells (>=2 frames): 11/25 (44%)
Total frames skipped (too similar): 1915
```

This helps identify if your calibration video needs more coverage in certain areas.

## Output

The calibration produces:

1. **YAML calibration file** containing:
   - Camera matrix K (fx, fy, cx, cy)
   - Distortion coefficients (KB4 format)
   - Model-specific parameters (OCam pol/invpol, DS xi/alpha, etc.)
   - RMS and mean reprojection error
   - Calibration metadata

2. **Undistorted sample image** for visual verification

### OCam Output Format

When using `output_model: "OCam"`, the output matches industry-standard Scaramuzza format:

```yaml
model_type: Ocam
image_width: 1920
image_height: 1080
length_pol: 5
pol_data: [a0, a1, a2, a3, a4]      # Forward polynomial (unprojection)
length_invpol: 13
invpol_data: [b0, b1, ..., b12]     # Inverse polynomial (projection)
ac: 1.0                              # Affine parameter c
ad: 0.0                              # Affine parameter d
ae: 0.0                              # Affine parameter e
center_x: 960.0                      # Principal point x
center_y: 540.0                      # Principal point y
```

## Tips for Good Calibration

1. **Checkerboard visibility**: Ensure the entire checkerboard is visible in each frame
2. **Cover the full FOV**: Move the board to center, edges, and corners of the image
3. **Vary orientations**: Tilt and rotate the board at different angles
4. **Vary distances**: Include close-up and distant views
5. **Good lighting**: Avoid motion blur and ensure clear corner detection
6. **Use smart selection**: Enable `smart_frame_selection` to automatically filter redundant frames

## License

MIT License
