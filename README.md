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
    "report_file": "calibration_report.txt",
    "output_model": "OCam",
    "show_detection": false,
    "frame_step": 5,
    
    "max_frames": 0,
    "outlier_reject_threshold_px": 5.0,
    "outlier_reject_top_fraction": 0.3,
    "undistort_fov_scale": 0.5,
    
    "smart_frame_selection": true,
    "grid_divisions": 5,
    "min_frames_per_cell": 1,
    "min_orientation_diff": 15.0,
    "max_tilt_threshold": 0.6,
    "max_rotation_angle": 45.0,
    "save_selected_frames": true
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

### Undistortion Control

| Parameter | Description |
|-----------|-------------|
| `undistort_fov_scale` | FOV scale for undistortion (0.3-0.8). Lower = wider FOV but more edge stretching. Higher = less stretching but crops more. Default: 0.5 |

### Intelligent Frame Selection

| Parameter | Description |
|-----------|-------------|
| `smart_frame_selection` | Enable/disable intelligent frame selection |
| `grid_divisions` | Divide image into NxN grid for FOV coverage (default: 5) |
| `min_frames_per_cell` | Target minimum frames per grid cell (default: 1 for ultra-wide FOV) |
| `min_orientation_diff` | Minimum board orientation difference in degrees (default: 15) |
| `max_tilt_threshold` | Maximum tilt ratio to accept (0.0-1.0). Rejects frames with excessive perspective distortion. Set to -1 to disable. Default: 0.6 |
| `max_rotation_angle` | Maximum rotation angle from horizontal in degrees. Rejects highly rotated boards. Set to -1 to disable. Default: 45 |
| `save_selected_frames` | Save selected frames to a folder (named after video) for review. Default: false |
| `report_file` | Path to generated calibration report text file. Default: "calibration_report.txt" |

## Intelligent Frame Selection

When `smart_frame_selection` is enabled, the calibration tool:

1. **Divides the image into a grid** (e.g., 5x5 = 25 cells)
2. **Analyzes each detected checkerboard** for:
   - Position (which grid cell the centroid falls in)
   - Orientation (rotation angle of the board)
   - Tilt (perspective distortion)
   - Size (proxy for distance from camera)
3. **Applies quality filters**:
   - **Tilt filter**: Rejects frames with excessive perspective distortion (controlled by `max_tilt_threshold`)
   - **Rotation filter**: Rejects frames where board is rotated too much from horizontal (controlled by `max_rotation_angle`)
   - These filters prevent poor calibration from extreme viewing angles
4. **Rejects similar frames** that don't add new information
5. **Reports FOV coverage** showing how well each region is covered
6. **Optionally saves selected frames** to a folder for manual review

### Quality Filters

**Tilt Filtering** (`max_tilt_threshold`):
- Measures perspective distortion (how much the board is tilted away from the camera)
- Value range: 0.0-1.0 (0.6 = reject if tilt > 60%)
- **Why it matters**: Extreme tilt at edges/corners causes poor corner detection and degrades calibration
- Set to `-1` to disable
- Recommended: `0.5-0.7` for ultra-wide FOV, `0.6-0.8` for standard fisheye

**Rotation Filtering** (`max_rotation_angle`):
- Rejects frames where checkerboard is rotated too far from horizontal
- Value in degrees (e.g., 45 = accept up to 45° rotation)
- **Why it matters**: Diagonal/vertical boards have worse corner detection accuracy
- Set to `-1` or `180` to disable
- Recommended: `30-45°` for best quality, `60-90°` for more coverage

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
   - Uses `undistort_fov_scale` to control stretching vs FOV trade-off
   - Lower scale = wider FOV but more edge stretching
   - Higher scale = less stretching but crops more

3. **Calibration report** (`calibration_report.txt`) containing:
   - Frame selection summary (processed/detected/kept/skipped)
   - FOV coverage analysis with visual grid
   - Calibration quality metrics (Stage-1/Stage-2 RMS)
   - Per-view error distribution
   - Quality assessment and recommendations

4. **Selected frames folder** (if `save_selected_frames: true`)
   - Folder named after video (e.g., `camera3_selected_frames/`)
   - Contains all frames selected by smart selection
   - Useful for manual review of calibration data

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
3. **Vary orientations**: Tilt and rotate the board at different angles (but avoid extreme tilts)
4. **Vary distances**: Include close-up and distant views
5. **Good lighting**: Avoid motion blur and ensure clear corner detection
6. **Use smart selection**: Enable `smart_frame_selection` to automatically filter redundant frames
7. **Enable quality filters**: Use `max_tilt_threshold` and `max_rotation_angle` to reject problematic frames
8. **Monitor the report**: Check `calibration_report.txt` for coverage analysis and quality metrics

## Troubleshooting

### Poor Calibration Quality (High RMS Error)

**Symptoms**: RMS > 5 px, asymmetric stretching in undistorted images

**Causes**:
- Frames with extreme tilt/rotation at edges/corners
- Poor FOV coverage (checkerboard only in center)
- Motion blur or poor corner detection

**Solutions**:
1. Enable tilt/rotation filters:
   ```json
   "max_tilt_threshold": 0.6,
   "max_rotation_angle": 45.0
   ```
2. Stricter outlier rejection:
   ```json
   "outlier_reject_threshold_px": 5.0,
   "outlier_reject_top_fraction": 0.3
   ```
3. Record new video with better lighting and slower movement

### Asymmetric Stretching in Undistorted Image

**Cause**: Bad calibration (incorrect principal point or distortion coefficients)

**Solution**: 
1. Recalibrate with quality filters enabled
2. Target RMS < 2.0 px (check report)
3. Once calibration is good, adjust `undistort_fov_scale` (0.4-0.7) to control symmetric stretching

### Low FOV Coverage

**Symptoms**: Coverage < 30%, many grid cells empty

**Causes**:
- Quality filters too strict
- Checkerboard too large for FOV
- Video doesn't cover full image area

**Solutions**:
1. Relax filters:
   ```json
   "max_tilt_threshold": 0.7,
   "max_rotation_angle": 60.0
   ```
2. Record longer video with systematic coverage (see DATA_COLLECTION_GUIDE.md)
3. For ultra-wide FOV (>180°), 50-70% coverage is normal and acceptable

## License

MIT License
