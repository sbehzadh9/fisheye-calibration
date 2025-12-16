# Fisheye Camera Calibration - Data Collection Guide

## Overview

This guide provides instructions for collecting calibration videos for fisheye cameras based on lessons learned from the TV_CAM calibration (200° FOV lens, 1920x1080 resolution).

## Reference Calibration Results

**Camera**: TV_CAM (200° FOV fisheye)
- **Video**: 10,693 frames processed
- **Detections**: 2,001 frames with checkerboard detected
- **Selected**: 86 diverse frames (1,915 similar frames skipped)
- **Final RMS**: 0.974 px (VERY GOOD quality)
- **FOV Coverage**: 60% (15/25 grid cells)
- **Checkerboard**: 11x8 inner corners, 30mm squares

### Coverage Analysis from TV_CAM

```
FOV Coverage Grid (5x5):
    +-----+-----+-----+-----+-----+
    |  L  |     |  C  |     |  R  |
    +-----+-----+-----+-----+-----+
 T  | 0   | 0   | 0   | 0   | 0   |  ← No coverage (board goes out of frame)
    +-----+-----+-----+-----+-----+
    | 1   | 7   | 4   | 2   | 1   |
    +-----+-----+-----+-----+-----+
 M  | 3   | 10  | 25  | 8   | 4   |  ← Best coverage (center region)
    +-----+-----+-----+-----+-----+
    | 2   | 10  | 7   | 1   | 1   |
    +-----+-----+-----+-----+-----+
 B  | 0   | 0   | 0   | 0   | 0   |  ← No coverage (board goes out of frame)
    +-----+-----+-----+-----+-----+

Coverage: 60% (15/25 cells)
Well-covered cells (≥2 frames): 44% (11/25 cells)
```

**Key Insight**: Top and bottom rows have zero coverage because when the checkerboard moves to these positions, parts of it go outside the camera's FOV. OpenCV requires ALL corners to be visible for detection.

---

## Equipment Required

### 1. Checkerboard Pattern

**Recommended Specifications**:
- **Size**: 11x8 or 9x6 inner corners
- **Square size**: 25-30mm (larger is better for detection accuracy)
- **Material**: Rigid flat board (foam board, acrylic, or aluminum)
- **Print quality**: High contrast, sharp edges
- **Mounting**: Ensure board stays perfectly flat

**Important**: For ultra-wide FOV cameras (>180°):
- **Smaller boards** allow better edge coverage (can get closer to edges without being cut off)
- **Larger boards** provide better detection accuracy but limit edge coverage
- **Trade-off**: TV_CAM used 11x8 (30mm) → 60% coverage but excellent 0.974px RMS

### 2. Video Recording Setup

- **Camera**: The fisheye camera to be calibrated
- **Lighting**: Uniform, bright lighting (avoid shadows and glare)
- **Framerate**: 30 fps or higher
- **Duration**: 2-5 minutes of continuous recording
- **Format**: Any common video format (mp4, mkv, avi)

---

## Data Collection Procedure

### Step 1: Environment Setup

1. **Lighting**:
   - Use diffuse, even lighting
   - Avoid direct sunlight causing specular reflections
   - Ensure checkerboard has high contrast (no shadows on pattern)

2. **Background**:
   - Plain, non-reflective background
   - Different color from checkerboard (avoid white board on white wall)

3. **Camera Position**:
   - Mount camera on tripod (camera stays stationary)
   - Ensure camera is stable (no vibration)
   - Frame rate: 30 fps minimum

### Step 2: Recording the Video

**Goal**: Collect ~2000 frames with checkerboard visible, targeting diverse poses across the FOV

**Recording Strategy**:

1. **Start Recording** and maintain continuous recording for 3-5 minutes

2. **Checkerboard Movement Pattern**:

   **A. Center Region Coverage** (1 minute)
   - Hold board in center at various orientations (0°, 45°, 90°, 135°)
   - Move board closer and farther (3 different distances)
   - Tilt board in different directions (front-back, left-right)
   
   **B. Left-Right Coverage** (1 minute)
   - Move board slowly from center to left edge
   - Keep board fully visible (watch edges don't get cut off)
   - Vary orientation while moving (0°, 45°, 90°)
   - Repeat for right side
   
   **C. Vertical Coverage** (1-2 minutes)
   - Move board slowly from center upward
   - **CRITICAL**: Stop before ANY corners go out of frame
   - For 200° FOV lenses, expect limited top/bottom coverage
   - Vary orientation while moving
   - Repeat for downward movement
   
   **D. Diagonal Coverage** (1 minute)
   - Move to top-left, top-right, bottom-left, bottom-right
   - Keep entire board visible
   - Vary distance and tilt
   
   **E. Random Poses** (1 minute)
   - Free-form movement covering missed areas
   - Include steep tilts and rotations
   - Mix close and far positions

3. **Movement Tips**:
   - Move slowly and smoothly (avoid motion blur)
   - Hold each pose for 2-3 seconds
   - Keep the board flat (avoid bending)
   - Ensure all corners remain visible at all times

### Step 3: Expected Coverage Limitations

**For Ultra-Wide FOV Cameras (180°-220°)**:

Based on TV_CAM results with 11x8 checkerboard:
- **Achievable Coverage**: 50-70% of FOV grid
- **Top/Bottom Edges**: Likely zero coverage (board physically too large)
- **Left/Right Edges**: Moderate coverage possible
- **Center Region**: Full coverage expected

**This is NORMAL and ACCEPTABLE**. The calibration quality (RMS error) is more important than 100% grid coverage.

### Step 4: Quality Indicators During Recording

✅ **Good Signs**:
- Checkerboard clearly visible and in focus
- All corners remain within frame boundaries
- Smooth, slow movements
- Board appears in different parts of the image
- Various orientations and distances

❌ **Bad Signs**:
- Motion blur (moving too fast)
- Checkerboard corners cut off at frame edges
- Poor lighting (shadows on pattern)
- Out of focus
- Board not flat (bent or curved)

---

## Configuration Parameters

### For Data Collection Phase

```json
{
    "video_path": "/path/to/your/calibration_video.mkv",
    "board_width": 11,          // Inner corners per row
    "board_height": 8,          // Inner corners per column
    "square_size": 0.030,       // Size in meters (30mm)
    "output_file": "calibration_results.yaml",
    "undistorted_image_output": "undistorted_sample.jpg",
    "report_file": "calibration_report.txt",
    "output_model": "OCam",     // OCam, KB4, KB8, DS, or EUCM
    "show_detection": false,    // Set true to visualize detection during processing
    "frame_step": 5,            // Process every 5th frame (reduce computation)
    
    // Outlier rejection (two-stage calibration)
    "max_frames": 0,                        // 0 = unlimited
    "outlier_reject_threshold_px": 10.0,    // Keep frames with RMS ≤ 10px
    "outlier_reject_top_fraction": 0.2,     // Reject worst 20% of frames
    
    // Smart frame selection (automatically select diverse frames)
    "smart_frame_selection": true,          // Enable intelligent selection
    "grid_divisions": 5,                    // 5x5 grid = 25 cells
    "min_frames_per_cell": 2,               // Target 2+ frames per cell
    "min_orientation_diff": 15.0            // Min 15° rotation between frames
}
```

### Parameter Guidelines for Different Cameras

**For Standard Fisheye (FOV < 180°)**:
- `grid_divisions`: 5 (5x5 = 25 cells)
- `min_frames_per_cell`: 2-3
- Expected coverage: 80-95%

**For Ultra-Wide Fisheye (FOV 180°-220°)**:
- `grid_divisions`: 5 (5x5 = 25 cells)
- `min_frames_per_cell`: 2
- Expected coverage: 50-70% (edges physically unreachable)
- `output_model`: "DS" or "EUCM" recommended (better than KB4 for >180°)

**For Moderate Fisheye (FOV 120°-170°)**:
- `grid_divisions`: 5-7
- `min_frames_per_cell`: 3-4
- Expected coverage: 90-100%

---

## Expected Results

### Quality Metrics

| Metric | Excellent | Very Good | Good | Acceptable | Poor |
|--------|-----------|-----------|------|------------|------|
| **Stage-2 RMS** | < 0.5 px | 0.5-1.0 px | 1.0-2.0 px | 2.0-5.0 px | > 5.0 px |
| **FOV Coverage** | > 85% | 70-85% | 50-70% | 30-50% | < 30% |
| **Frames Selected** | 80-150 | 60-100 | 40-80 | 20-60 | < 20 |

**TV_CAM Results**:
- Stage-2 RMS: **0.974 px** → VERY GOOD
- FOV Coverage: **60%** → GOOD (limited by checkerboard size vs FOV)
- Frames Selected: **86** → VERY GOOD

### Interpreting the Report

After calibration, the system generates a detailed report (`calibration_report.txt`):

1. **Frame Selection Summary**:
   - Total frames processed
   - Frames with board detected (~20% detection rate is normal)
   - Frames kept vs skipped (smart selection keeps ~5-10% of detections)

2. **FOV Coverage Grid**:
   - Visual representation of checkerboard distribution
   - Identifies under-covered regions
   - For ultra-wide FOV: expect gaps at top/bottom edges

3. **Calibration Results**:
   - Stage-1 RMS: Initial calibration (often 50-500 px)
   - Stage-2 RMS: After outlier rejection (target < 1.0 px)

4. **Quality Assessment**:
   - Automatic rating based on RMS error
   - Recommendations if quality is poor

---

## Troubleshooting Common Issues

### Issue 1: Low Detection Rate (< 10% of frames)

**Possible Causes**:
- Checkerboard too small or too large
- Motion blur from fast movement
- Poor lighting / low contrast
- Board not flat

**Solutions**:
- Use larger checkerboard squares (30mm+)
- Move slower, hold poses longer
- Improve lighting (bright, diffuse)
- Use rigid board material

### Issue 2: Poor FOV Coverage (< 30%)

**Possible Causes**:
- Not enough varied poses in video
- Checkerboard stays in center
- Video too short

**Solutions**:
- Record longer video (5+ minutes)
- Systematically cover all regions (follow Step 2 pattern)
- For ultra-wide FOV: accept 50-70% coverage as normal

### Issue 3: High RMS Error (> 2.0 px)

**Possible Causes**:
- Motion blur in frames
- Checkerboard not flat (bent/curved)
- Poor corner detection quality
- Wrong square_size in config

**Solutions**:
- Verify `square_size` matches physical board
- Use flatter/more rigid board
- Improve lighting to increase corner detection accuracy
- Reduce motion blur (slower movement, better camera settings)

### Issue 4: Top/Bottom Grid Rows Empty

**This is NORMAL for ultra-wide FOV cameras**:
- Checkerboard physically too large to fit at extreme angles
- Solution: Accept limited coverage OR use smaller checkerboard (trade-off: less accuracy)
- TV_CAM had 0 frames in top/bottom rows but still achieved 0.974 px RMS

---

## Camera-Specific Recommendations

### For Each New Camera:

1. **Determine FOV**:
   - Check camera specifications
   - Approximate from initial test recording

2. **Select Model**:
   - FOV < 180°: `KB4` or `KB8`
   - FOV 180°-220°: `DS` (Double Sphere) or `EUCM`
   - Unknown/General: `OCam` (works for all FOV ranges)

3. **Adjust Checkerboard Size**:
   - Wider FOV → Use smaller board for better edge coverage
   - Narrower FOV → Use larger board for better accuracy

4. **Record Test Video**:
   - 3-5 minutes following movement pattern
   - Review coverage report

5. **Iterate if Needed**:
   - If coverage < 30%: record new video with more edge coverage
   - If RMS > 2.0 px: check board flatness and lighting

---

## Checklist for Each Camera

- [ ] Camera specifications verified (resolution, FOV)
- [ ] Checkerboard prepared (flat, high contrast, correct size)
- [ ] Lighting setup (bright, even, no glare)
- [ ] Config file updated (board_width, board_height, square_size, video_path)
- [ ] Test recording (10 seconds) to verify detection
- [ ] Full calibration video recorded (3-5 minutes)
- [ ] Video covers center, edges, and corners systematically
- [ ] Various orientations included (0°, 45°, 90°, 135°)
- [ ] Multiple distances recorded (near, medium, far)
- [ ] Calibration executed: `./fisheye_calib config.json`
- [ ] Report reviewed (`calibration_report.txt`)
- [ ] Quality assessment: RMS < 2.0 px and coverage > 30%
- [ ] Results saved to repository

---

## Files Generated Per Camera

After successful calibration, you will have:

1. **`calibration_results.yaml`**: Calibration parameters (K matrix, distortion coefficients, model-specific params)
2. **`calibration_report.txt`**: Detailed analysis report with FOV coverage grid
3. **`undistorted_sample.jpg`**: Sample undistorted image for visual verification

**Storage**: Save all three files with camera-specific names:
```
results/
  ├── camera1_calibration_results.yaml
  ├── camera1_calibration_report.txt
  ├── camera1_undistorted_sample.jpg
  ├── camera2_calibration_results.yaml
  ├── camera2_calibration_report.txt
  └── camera2_undistorted_sample.jpg
```

---

## Summary: Key Takeaways from TV_CAM

✅ **What Worked Well**:
- Smart frame selection filtered 2,001 detections → 86 diverse frames
- 60% FOV coverage was sufficient for excellent 0.974 px RMS
- OCam model handled 200° FOV successfully
- Systematic movement pattern provided good center/mid-edge coverage

⚠️ **Limitations Encountered**:
- Top/bottom edges had zero coverage (checkerboard too large for extreme angles)
- This is **normal and acceptable** for ultra-wide FOV lenses
- Quality (RMS) is more important than 100% grid coverage

📋 **Best Practices**:
- Record 3-5 minutes (ensures 2000+ detections)
- Move slowly and smoothly (avoid motion blur)
- Keep entire checkerboard visible (watch frame edges)
- Trust smart frame selection (skips redundant frames automatically)
- For ultra-wide FOV: expect 50-70% coverage, target RMS < 1.5 px

---

**Document Version**: 1.0  
**Date**: December 14, 2025  
**Reference Camera**: TV_CAM (200° FOV, 1920x1080)
