#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <ctime>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>

using json = nlohmann::json;
using namespace std;
using namespace cv;

struct Config {
    string video_path;
    int board_width;
    int board_height;
    float square_size;
    string output_file;
    string undistorted_image_output;
    string output_model;
    bool show_detection;
    int frame_step;
    int max_frames;
    double outlier_reject_threshold_px;
    double outlier_reject_top_fraction;
    double undistort_fov_scale;  // FOV scale for undistortion (0.3-0.8, lower=wider)
    // Intelligent frame selection
    bool smart_frame_selection;
    int grid_divisions;        // Divide image into NxN grid for FOV coverage
    int min_frames_per_cell;   // Minimum frames per grid cell
    double min_orientation_diff; // Minimum orientation difference (degrees)
    double max_tilt_threshold; // Maximum tilt ratio (0.0-1.0, higher = more tilted)
    double max_rotation_angle; // Maximum absolute rotation angle from horizontal (degrees)
    bool save_selected_frames; // Save selected frames to folder
    // Report output
    string report_file;
};

// Structure to hold calibration report data
struct CalibrationReport {
    // Frame statistics
    int total_frames_processed = 0;
    int total_frames_detected = 0;
    int frames_kept = 0;
    int frames_skipped_similar = 0;
    
    // Smart selection settings
    bool smart_selection_enabled = false;
    int grid_divisions = 5;
    double min_orientation_diff = 15.0;
    
    // FOV coverage
    vector<int> grid_cell_counts;
    int total_cells = 0;
    int covered_cells = 0;
    int well_covered_cells = 0;
    double coverage_percentage = 0.0;
    double well_covered_percentage = 0.0;
    
    // Calibration results
    double stage1_rms = 0.0;
    double stage2_rms = 0.0;
    int stage1_views = 0;
    int stage2_views = 0;
    double min_view_rms = 0.0;
    double max_view_rms = 0.0;
    double avg_view_rms = 0.0;
    int views_below_1px = 0;
    int views_1_5px = 0;
    int views_5_10px = 0;
    int views_above_10px = 0;
    
    // Camera parameters
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    Size image_size;
    string model_type;
    
    // Timing
    string timestamp;
};

Config loadConfig(const string& filename) {
    ifstream f(filename);
    if (!f.is_open()) {
        throw runtime_error("Could not open config file: " + filename);
    }
    json data = json::parse(f);
    Config cfg;
    cfg.video_path = data.value("video_path", "input_video.mp4");
    cfg.board_width = data.value("board_width", 9);
    cfg.board_height = data.value("board_height", 6);
    cfg.square_size = data.value("square_size", 0.025f);
    cfg.output_file = data.value("output_file", "calibration_results.yaml");
    cfg.undistorted_image_output = data.value("undistorted_image_output", "undistorted.jpg");
    cfg.output_model = data.value("output_model", "KB4");
    cfg.show_detection = data.value("show_detection", true);
    cfg.frame_step = data.value("frame_step", 10);
    cfg.max_frames = data.value("max_frames", 80);
    cfg.outlier_reject_threshold_px = data.value("outlier_reject_threshold_px", -1.0);
    cfg.outlier_reject_top_fraction = data.value("outlier_reject_top_fraction", 0.2);
    cfg.undistort_fov_scale = data.value("undistort_fov_scale", 0.5);  // default 0.5 for wide FOV
    // Intelligent frame selection parameters
    cfg.smart_frame_selection = data.value("smart_frame_selection", false);
    cfg.grid_divisions = data.value("grid_divisions", 5);  // 5x5 grid = 25 cells
    cfg.min_frames_per_cell = data.value("min_frames_per_cell", 2);
    cfg.min_orientation_diff = data.value("min_orientation_diff", 15.0);  // degrees
    cfg.max_tilt_threshold = data.value("max_tilt_threshold", 0.6);  // 0.6 = reject frames with >60% tilt
    cfg.max_rotation_angle = data.value("max_rotation_angle", 45.0);  // reject boards rotated >45° from horizontal
    cfg.save_selected_frames = data.value("save_selected_frames", false);
    // Report output file
    cfg.report_file = data.value("report_file", "calibration_report.txt");
    return cfg;
}

// Helper function to create directory
static bool createDirectory(const string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        // Directory doesn't exist, create it
#ifdef _WIN32
        return mkdir(path.c_str()) == 0;
#else
        return mkdir(path.c_str(), 0755) == 0;
#endif
    } else if (info.st_mode & S_IFDIR) {
        // Directory already exists
        return true;
    }
    return false;
}

// Extract filename without extension from full path
static string getVideoBaseName(const string& videoPath) {
    // Find last directory separator
    size_t lastSlash = videoPath.find_last_of("/\\");
    string filename = (lastSlash != string::npos) ? videoPath.substr(lastSlash + 1) : videoPath;
    
    // Remove extension
    size_t lastDot = filename.find_last_of(".");
    if (lastDot != string::npos) {
        filename = filename.substr(0, lastDot);
    }
    
    return filename;
}

// ============ Intelligent Frame Selection Helpers ============

// Compute the centroid of detected corners
static Point2f computeCentroid(const vector<Point2f>& corners) {
    Point2f centroid(0, 0);
    for (const auto& c : corners) {
        centroid.x += c.x;
        centroid.y += c.y;
    }
    centroid.x /= corners.size();
    centroid.y /= corners.size();
    return centroid;
}

// Compute board orientation from corners (angle of first row)
static double computeBoardOrientation(const vector<Point2f>& corners, int boardWidth) {
    // Use first and last corner of first row to compute orientation
    Point2f p1 = corners[0];
    Point2f p2 = corners[boardWidth - 1];
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return atan2(dy, dx) * 180.0 / CV_PI;  // degrees
}

// Compute board size (diagonal span) to estimate distance
static double computeBoardSize(const vector<Point2f>& corners) {
    Point2f minPt = corners[0], maxPt = corners[0];
    for (const auto& c : corners) {
        minPt.x = min(minPt.x, c.x);
        minPt.y = min(minPt.y, c.y);
        maxPt.x = max(maxPt.x, c.x);
        maxPt.y = max(maxPt.y, c.y);
    }
    return sqrt((maxPt.x - minPt.x) * (maxPt.x - minPt.x) + 
                (maxPt.y - minPt.y) * (maxPt.y - minPt.y));
}

// Compute tilt angles from corner pattern (perspective distortion)
static pair<double, double> computeBoardTilt(const vector<Point2f>& corners, int boardWidth, int boardHeight) {
    // Horizontal tilt: compare top and bottom row lengths
    double topRowLen = norm(corners[boardWidth - 1] - corners[0]);
    double botRowLen = norm(corners[(boardHeight - 1) * boardWidth + boardWidth - 1] - 
                           corners[(boardHeight - 1) * boardWidth]);
    double hTilt = (topRowLen - botRowLen) / max(topRowLen, botRowLen) * 45.0;  // rough angle estimate
    
    // Vertical tilt: compare left and right column lengths
    double leftColLen = norm(corners[(boardHeight - 1) * boardWidth] - corners[0]);
    double rightColLen = norm(corners[(boardHeight - 1) * boardWidth + boardWidth - 1] - 
                             corners[boardWidth - 1]);
    double vTilt = (leftColLen - rightColLen) / max(leftColLen, rightColLen) * 45.0;
    
    return {hTilt, vTilt};
}

// Get grid cell index for a point
static int getGridCell(const Point2f& pt, const Size& imageSize, int gridDivisions) {
    int cellX = min(gridDivisions - 1, max(0, (int)(pt.x / imageSize.width * gridDivisions)));
    int cellY = min(gridDivisions - 1, max(0, (int)(pt.y / imageSize.height * gridDivisions)));
    return cellY * gridDivisions + cellX;
}

// Structure to hold frame info for smart selection
struct FrameInfo {
    int frameIdx;
    vector<Point2f> corners;
    Point2f centroid;
    double orientation;
    double boardSize;
    pair<double, double> tilt;
    int gridCell;
};

// Check if new frame is sufficiently different from existing frames
static bool isFrameDiverse(const FrameInfo& newFrame, 
                          const vector<FrameInfo>& existingFrames,
                          double minOrientationDiff,
                          int gridDivisions,
                          double maxTiltThreshold,
                          double maxRotationAngle) {
    // First check: reject frames with excessive tilt (extreme perspective distortion)
    // This prevents poor calibration from highly angled edge/corner frames
    double maxAbsTilt = max(abs(newFrame.tilt.first), abs(newFrame.tilt.second));
    double tiltRatio = maxAbsTilt / 45.0;  // Normalize to 0-1 range (45° is reference)
    if (tiltRatio > maxTiltThreshold) {
        return false;  // Frame too tilted - would degrade calibration quality
    }
    
    // Second check: reject frames with excessive rotation angle
    // Normalize orientation to -90 to +90 range (relative to horizontal)
    double rotation = newFrame.orientation;
    while (rotation > 90) rotation -= 180;
    while (rotation < -90) rotation += 180;
    if (abs(rotation) > maxRotationAngle) {
        return false;  // Board rotated too much from horizontal - poor corner detection
    }
    
    // Count frames in same grid cell
    int sameCellCount = 0;
    for (const auto& f : existingFrames) {
        if (f.gridCell == newFrame.gridCell) {
            sameCellCount++;
            
            // Check orientation difference
            double orientDiff = abs(f.orientation - newFrame.orientation);
            if (orientDiff > 180) orientDiff = 360 - orientDiff;
            
            // Check tilt difference
            double tiltDiff = abs(f.tilt.first - newFrame.tilt.first) + 
                             abs(f.tilt.second - newFrame.tilt.second);
            
            // Check size difference (distance proxy)
            double sizeDiff = abs(f.boardSize - newFrame.boardSize) / max(f.boardSize, newFrame.boardSize) * 100;
            
            // If very similar frame exists in same cell, reject
            if (orientDiff < minOrientationDiff && tiltDiff < 10 && sizeDiff < 20) {
                return false;
            }
        }
    }
    
    return true;
}

// ============ Report Generation Functions ============

static string getCurrentTimestamp() {
    time_t now = time(nullptr);
    char buf[80];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&now));
    return string(buf);
}

static void generateReport(const CalibrationReport& report, const string& reportPath, const Config& cfg) {
    // Create report content
    stringstream ss;
    
    ss << "================================================================================\n";
    ss << "                    FISHEYE CAMERA CALIBRATION REPORT\n";
    ss << "================================================================================\n";
    ss << "Generated: " << report.timestamp << "\n";
    ss << "Video: " << cfg.video_path << "\n";
    ss << "Output Model: " << report.model_type << "\n";
    ss << "\n";
    
    // Frame Selection Summary
    ss << "--------------------------------------------------------------------------------\n";
    ss << "                         FRAME SELECTION SUMMARY\n";
    ss << "--------------------------------------------------------------------------------\n";
    ss << "\n";
    ss << "| Metric                    | Value                                    |\n";
    ss << "|---------------------------|------------------------------------------|\n";
    ss << "| Smart Frame Selection     | " << (report.smart_selection_enabled ? "ENABLED" : "DISABLED") << setw(33 - (report.smart_selection_enabled ? 7 : 8)) << " |\n";
    ss << "| Total frames processed    | " << setw(40) << left << report.total_frames_processed << " |\n";
    ss << "| Frames with board detected| " << setw(40) << left << report.total_frames_detected << " |\n";
    
    if (report.smart_selection_enabled) {
        stringstream fs;
        fs << report.frames_kept << " kept, " << report.frames_skipped_similar << " skipped";
        ss << "| Frames after filtering    | " << setw(40) << left << fs.str() << " |\n";
        ss << "| Similar frames skipped    | " << setw(40) << left << report.frames_skipped_similar << " |\n";
    } else {
        ss << "| Frames used               | " << setw(40) << left << report.frames_kept << " |\n";
    }
    ss << "\n";
    
    // FOV Coverage Analysis
    if (report.smart_selection_enabled && !report.grid_cell_counts.empty()) {
        ss << "--------------------------------------------------------------------------------\n";
        ss << "                          FOV COVERAGE ANALYSIS\n";
        ss << "--------------------------------------------------------------------------------\n";
        ss << "\n";
        
        ss << "Grid: " << report.grid_divisions << "x" << report.grid_divisions << " (" << report.total_cells << " cells)\n";
        ss << "Min orientation difference: " << report.min_orientation_diff << " degrees\n\n";
        
        // Coverage metrics
        stringstream cov1, cov2;
        cov1 << report.covered_cells << "/" << report.total_cells << " (" << fixed << setprecision(0) << report.coverage_percentage << "%)";
        cov2 << report.well_covered_cells << "/" << report.total_cells << " (" << fixed << setprecision(0) << report.well_covered_percentage << "%)";
        
        ss << "| Metric                    | Value                                    |\n";
        ss << "|---------------------------|------------------------------------------|\n";
        ss << "| FOV coverage (any frames) | " << setw(40) << left << cov1.str() << " |\n";
        ss << "| Well-covered cells (>=2)  | " << setw(40) << left << cov2.str() << " |\n";
        ss << "\n";
        
        // Grid visualization
        ss << "FOV Coverage Grid (frames per cell):\n\n";
        int g = report.grid_divisions;
        
        // Top border
        ss << "    +";
        for (int x = 0; x < g; x++) ss << "-----+";
        ss << "\n";
        
        // Row labels
        ss << "    |";
        for (int x = 0; x < g; x++) {
            string label;
            if (x == 0) label = " L ";
            else if (x == g-1) label = " R ";
            else if (x == g/2) label = " C ";
            else label = "   ";
            ss << " " << label << " |";
        }
        ss << "   (L=Left, C=Center, R=Right)\n";
        
        ss << "    +";
        for (int x = 0; x < g; x++) ss << "-----+";
        ss << "\n";
        
        for (int y = 0; y < g; y++) {
            string rowLabel;
            if (y == 0) rowLabel = " T ";
            else if (y == g-1) rowLabel = " B ";
            else if (y == g/2) rowLabel = " M ";
            else rowLabel = "   ";
            
            ss << rowLabel << " |";
            for (int x = 0; x < g; x++) {
                int count = report.grid_cell_counts[y * g + x];
                ss << " " << setw(3) << count << " |";
            }
            
            if (y == 0) ss << "   T=Top";
            else if (y == g/2) ss << "   M=Middle";
            else if (y == g-1) ss << "   B=Bottom";
            ss << "\n";
            
            ss << "    +";
            for (int x = 0; x < g; x++) ss << "-----+";
            ss << "\n";
        }
        ss << "\n";
        
        // Coverage advice
        if (report.coverage_percentage < 50) {
            ss << "⚠️  WARNING: Low FOV coverage! For best calibration of wide-angle lenses:\n";
            ss << "    - Move the checkerboard to the edges and corners of the image\n";
            ss << "    - Include more poses at different distances and tilts\n";
            ss << "    - Aim for at least 60% grid coverage\n\n";
        } else if (report.coverage_percentage < 75) {
            ss << "💡 TIP: Coverage is moderate. For a 200° FOV lens, try to get more frames\n";
            ss << "    at the image edges/corners where distortion is highest.\n\n";
        } else {
            ss << "✓  Good FOV coverage! The calibration should be reliable across the FOV.\n\n";
        }
    }
    
    // Calibration Results
    ss << "--------------------------------------------------------------------------------\n";
    ss << "                          CALIBRATION RESULTS\n";
    ss << "--------------------------------------------------------------------------------\n";
    ss << "\n";
    ss << "| Stage                     | RMS Error      | Views Used                |\n";
    ss << "|---------------------------|----------------|---------------------------|\n";
    ss << "| Stage-1 (initial)         | " << setw(12) << fixed << setprecision(3) << report.stage1_rms << " px | " << setw(25) << report.stage1_views << " |\n";
    ss << "| Stage-2 (after outliers)  | " << setw(12) << fixed << setprecision(3) << report.stage2_rms << " px | " << setw(25) << report.stage2_views << " |\n";
    ss << "\n";
    
    // Per-view RMS distribution
    ss << "Per-View Reprojection Error Distribution:\n";
    ss << "| Range              | Count      | Percentage  |\n";
    ss << "|--------------------|------------|-------------|\n";
    int total_views = report.views_below_1px + report.views_1_5px + report.views_5_10px + report.views_above_10px;
    if (total_views > 0) {
        ss << "| < 1 px             | " << setw(10) << report.views_below_1px << " | " << setw(9) << fixed << setprecision(1) << (100.0 * report.views_below_1px / total_views) << "% |\n";
        ss << "| 1 - 5 px           | " << setw(10) << report.views_1_5px << " | " << setw(9) << fixed << setprecision(1) << (100.0 * report.views_1_5px / total_views) << "% |\n";
        ss << "| 5 - 10 px          | " << setw(10) << report.views_5_10px << " | " << setw(9) << fixed << setprecision(1) << (100.0 * report.views_5_10px / total_views) << "% |\n";
        ss << "| > 10 px            | " << setw(10) << report.views_above_10px << " | " << setw(9) << fixed << setprecision(1) << (100.0 * report.views_above_10px / total_views) << "% |\n";
    }
    ss << "\n";
    ss << "Min/Max/Avg view RMS: " << fixed << setprecision(2) << report.min_view_rms << " / " << report.max_view_rms << " / " << report.avg_view_rms << " px\n\n";
    
    // Camera Parameters
    ss << "--------------------------------------------------------------------------------\n";
    ss << "                          CAMERA PARAMETERS\n";
    ss << "--------------------------------------------------------------------------------\n";
    ss << "\n";
    ss << "Image Size: " << report.image_size.width << " x " << report.image_size.height << " pixels\n";
    ss << "Model: " << report.model_type << "\n\n";
    ss << "Intrinsic Matrix K:\n";
    ss << "  [ " << fixed << setprecision(4) << report.fx << "    0.0000    " << report.cx << " ]\n";
    ss << "  [   0.0000    " << report.fy << "    " << report.cy << " ]\n";
    ss << "  [   0.0000      0.0000      1.0000 ]\n";
    ss << "\n";
    
    // Quality Assessment
    ss << "--------------------------------------------------------------------------------\n";
    ss << "                          QUALITY ASSESSMENT\n";
    ss << "--------------------------------------------------------------------------------\n";
    ss << "\n";
    
    string quality;
    if (report.stage2_rms < 0.5) quality = "EXCELLENT";
    else if (report.stage2_rms < 1.0) quality = "VERY GOOD";
    else if (report.stage2_rms < 2.0) quality = "GOOD";
    else if (report.stage2_rms < 5.0) quality = "ACCEPTABLE";
    else quality = "POOR - consider recalibrating";
    
    ss << "Overall Quality: " << quality << " (RMS = " << fixed << setprecision(3) << report.stage2_rms << " px)\n\n";
    
    if (report.stage2_rms > 2.0) {
        ss << "Recommendations for improvement:\n";
        ss << "  - Use a larger/flatter checkerboard\n";
        ss << "  - Ensure better lighting conditions\n";
        ss << "  - Reduce motion blur in video capture\n";
        ss << "  - Capture more frames with varied poses\n";
    }
    
    ss << "\n================================================================================\n";
    ss << "Output files:\n";
    ss << "  - Calibration: " << cfg.output_file << "\n";
    ss << "  - Report: " << cfg.report_file << "\n";
    ss << "  - Undistorted sample: " << cfg.undistorted_image_output << "\n";
    ss << "================================================================================\n";
    
    string reportContent = ss.str();
    
    // Print to terminal
    cout << "\n" << reportContent;
    
    // Save to file
    ofstream reportFile(reportPath);
    if (reportFile.is_open()) {
        reportFile << reportContent;
        reportFile.close();
        cout << "\nReport saved to: " << reportPath << endl;
    } else {
        cerr << "Warning: Could not save report to " << reportPath << endl;
    }
}

// ============ End of Report Generation Functions ============

// ============ End of Intelligent Frame Selection Helpers ============

static double computeViewRms(const vector<Point3f>& obj,
                             const vector<Point2f>& img,
                             const Vec3d& rvec,
                             const Vec3d& tvec,
                             const Mat& K,
                             const Mat& D) {
    vector<Point2f> proj;
    fisheye::projectPoints(obj, proj, rvec, tvec, K, D);
    double errSq = 0.0;
    for (size_t j = 0; j < obj.size(); ++j) {
        const double dx = img[j].x - proj[j].x;
        const double dy = img[j].y - proj[j].y;
        errSq += dx * dx + dy * dy;
    }
    return std::sqrt(errSq / static_cast<double>(obj.size()));
}

struct CalibResult {
    Mat K;
    Mat D;
    vector<Vec3d> rvecs;
    vector<Vec3d> tvecs;
    double rms;
};

static CalibResult runFisheyeCalib(const vector<vector<Point3f>>& objectPoints,
                                  const vector<vector<Point2f>>& imagePoints,
                                  const Size& imageSize,
                                  const Mat& K_init,
                                  const Mat& D_init,
                                  bool checkCond = false) {
    CalibResult res;
    res.K = K_init.clone();
    res.D = D_init.clone();
    int flags = fisheye::CALIB_RECOMPUTE_EXTRINSIC | fisheye::CALIB_FIX_SKEW | fisheye::CALIB_USE_INTRINSIC_GUESS;
    if (checkCond) {
        flags |= fisheye::CALIB_CHECK_COND;
    }
    res.rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, res.K, res.D, res.rvecs, res.tvecs, flags,
                                 TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 100, 1e-6));
    return res;
}

int main(int argc, char** argv) {
    string configPath = "config.json";
    if (argc > 1) {
        configPath = argv[1];
    }

    Config cfg;
    try {
        cfg = loadConfig(configPath);
    } catch (const exception& e) {
        cerr << "Error loading config: " << e.what() << endl;
        return -1;
    }

    // Initialize calibration report
    CalibrationReport report;
    report.timestamp = getCurrentTimestamp();
    report.smart_selection_enabled = cfg.smart_frame_selection;
    report.grid_divisions = cfg.grid_divisions;
    report.min_orientation_diff = cfg.min_orientation_diff;
    report.model_type = cfg.output_model;

    VideoCapture cap(cfg.video_path);
    if (!cap.isOpened()) {
        cerr << "Error opening video file: " << cfg.video_path << endl;
        return -1;
    }

    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;
    vector<Point3f> obj;
    
    // Define the checkerboard pattern in 3D
    for (int i = 0; i < cfg.board_height; i++) {
        for (int j = 0; j < cfg.board_width; j++) {
            obj.push_back(Point3f(j * cfg.square_size, i * cfg.square_size, 0));
        }
    }

    Mat frame, gray;
    int frameCount = 0;
    Size imageSize;
    int validFrames = 0;
    int skippedSimilar = 0;
    int totalDetections = 0;

    // For intelligent frame selection
    vector<FrameInfo> selectedFrames;
    vector<int> gridCellCounts;
    string outputFolder;
    
    if (cfg.smart_frame_selection) {
        gridCellCounts.resize(cfg.grid_divisions * cfg.grid_divisions, 0);
        
        // Create output folder for selected frames if enabled
        if (cfg.save_selected_frames) {
            string videoBaseName = getVideoBaseName(cfg.video_path);
            outputFolder = videoBaseName + "_selected_frames";
            
            if (createDirectory(outputFolder)) {
                cout << "Created output folder: " << outputFolder << endl;
            } else {
                cerr << "Warning: Could not create output folder " << outputFolder << endl;
                cfg.save_selected_frames = false;  // Disable saving
            }
        }
    }

    cout << "Processing video..." << endl;
    if (cfg.smart_frame_selection) {
        cout << "Smart frame selection ENABLED (grid: " << cfg.grid_divisions << "x" << cfg.grid_divisions 
             << ", min orientation diff: " << cfg.min_orientation_diff << " deg)" << endl;
        if (cfg.save_selected_frames) {
            cout << "Saving selected frames to: " << outputFolder << "/" << endl;
        }
    }

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        if (frameCount % cfg.frame_step == 0) {
            if (imageSize.width == 0) {
                imageSize = frame.size();
            }

            cvtColor(frame, gray, COLOR_BGR2GRAY);
            vector<Point2f> corners;
            bool found = findChessboardCorners(gray, Size(cfg.board_width, cfg.board_height), corners,
                CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

            if (found) {
                totalDetections++;
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));
                
                bool acceptFrame = true;
                
                if (cfg.smart_frame_selection) {
                    // Compute frame characteristics
                    FrameInfo fi;
                    fi.frameIdx = frameCount;
                    fi.corners = corners;
                    fi.centroid = computeCentroid(corners);
                    fi.orientation = computeBoardOrientation(corners, cfg.board_width);
                    fi.boardSize = computeBoardSize(corners);
                    fi.tilt = computeBoardTilt(corners, cfg.board_width, cfg.board_height);
                    fi.gridCell = getGridCell(fi.centroid, imageSize, cfg.grid_divisions);
                    
                    // Check if frame is sufficiently different (includes tilt and rotation filtering)
                    acceptFrame = isFrameDiverse(fi, selectedFrames, cfg.min_orientation_diff, cfg.grid_divisions, cfg.max_tilt_threshold, cfg.max_rotation_angle);
                    
                    if (acceptFrame) {
                        selectedFrames.push_back(fi);
                        gridCellCounts[fi.gridCell]++;
                    } else {
                        skippedSimilar++;
                    }
                }
                
                if (acceptFrame) {
                    imagePoints.push_back(corners);
                    objectPoints.push_back(obj);
                    validFrames++;
                    
                    // Save selected frame to output folder
                    if (cfg.save_selected_frames && !outputFolder.empty()) {
                        stringstream ss;
                        ss << outputFolder << "/frame_" << setfill('0') << setw(6) << validFrames 
                           << "_f" << frameCount << ".jpg";
                        imwrite(ss.str(), frame);
                    }

                    if (cfg.max_frames > 0 && validFrames >= cfg.max_frames) {
                        cout << "\nReached max_frames=" << cfg.max_frames << ", stopping capture." << endl;
                        break;
                    }
                }
                else if (cfg.save_selected_frames && cfg.smart_frame_selection && !acceptFrame) {
                    // Frame was detected but skipped - optionally could save to separate folder
                    // Currently just skipping without saving
                }
                
                if (cfg.show_detection) {
                    Mat displayFrame = frame.clone();
                    drawChessboardCorners(displayFrame, Size(cfg.board_width, cfg.board_height), corners, found);
                    
                    // Add text showing frame info
                    string text = "Frame: " + to_string(frameCount) + " | Valid: " + to_string(validFrames);
                    if (cfg.smart_frame_selection) {
                        text += " | Skipped: " + to_string(skippedSimilar);
                        if (!acceptFrame) {
                            putText(displayFrame, "SKIPPED (similar)", Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                        }
                    }
                    putText(displayFrame, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                    
                    imshow("Detection", displayFrame);
                    waitKey(1);
                }
                cout << "Frame " << frameCount << " | Valid: " << validFrames;
                if (cfg.smart_frame_selection) {
                    cout << " | Skipped similar: " << skippedSimilar;
                }
                cout << "\r" << flush;
            }
        }
        frameCount++;
    }
    cout << endl;
    
    // Populate report with frame statistics
    report.total_frames_processed = frameCount;
    report.total_frames_detected = totalDetections;
    report.frames_kept = validFrames;
    report.frames_skipped_similar = skippedSimilar;
    report.image_size = imageSize;
    
    // Print FOV coverage statistics if smart selection was used
    if (cfg.smart_frame_selection && !gridCellCounts.empty()) {
        cout << "\n=== FOV Coverage Analysis ===" << endl;
        int totalCells = cfg.grid_divisions * cfg.grid_divisions;
        int coveredCells = 0;
        int wellCoveredCells = 0;
        
        cout << "Grid coverage (frames per cell):" << endl;
        for (int y = 0; y < cfg.grid_divisions; y++) {
            cout << "  ";
            for (int x = 0; x < cfg.grid_divisions; x++) {
                int count = gridCellCounts[y * cfg.grid_divisions + x];
                cout << setw(3) << count << " ";
                if (count > 0) coveredCells++;
                if (count >= cfg.min_frames_per_cell) wellCoveredCells++;
            }
            cout << endl;
        }
        cout << "Cells with data: " << coveredCells << "/" << totalCells 
             << " (" << (100.0 * coveredCells / totalCells) << "%)" << endl;
        cout << "Well-covered cells (>=" << cfg.min_frames_per_cell << " frames): " << wellCoveredCells << "/" << totalCells 
             << " (" << (100.0 * wellCoveredCells / totalCells) << "%)" << endl;
        cout << "Total frames skipped (too similar): " << skippedSimilar << endl;
        cout << "============================\n" << endl;
        
        // Populate report with FOV coverage data
        report.grid_cell_counts = gridCellCounts;
        report.total_cells = totalCells;
        report.covered_cells = coveredCells;
        report.well_covered_cells = wellCoveredCells;
        report.coverage_percentage = 100.0 * coveredCells / totalCells;
        report.well_covered_percentage = 100.0 * wellCoveredCells / totalCells;
    }
    
    if (validFrames < 1) {
        cerr << "No valid frames found for calibration." << endl;
        return -1;
    }

    cout << "Initial calibration with " << validFrames << " frames..." << endl;

    Mat K_init = Mat::eye(3, 3, CV_64F);
    K_init.at<double>(0, 0) = imageSize.width / 2.0;
    K_init.at<double>(1, 1) = imageSize.width / 2.0;
    K_init.at<double>(0, 2) = imageSize.width / 2.0;
    K_init.at<double>(1, 2) = imageSize.height / 2.0;
    Mat D_init = Mat::zeros(4, 1, CV_64F);

    CalibResult calib1 = runFisheyeCalib(objectPoints, imagePoints, imageSize, K_init, D_init);


    cout << "Stage-1 RMS (OpenCV): " << calib1.rms << " pixels" << endl;
    cout << "Stage-1 K:" << endl << calib1.K << endl;
    cout << "Stage-1 D (k1,k2,k3,k4): " << calib1.D.t() << endl;
    
    // Populate stage-1 report data
    report.stage1_rms = calib1.rms;
    report.stage1_views = validFrames;

    vector<double> perViewRms(objectPoints.size(), 0.0);
    for (size_t i = 0; i < objectPoints.size(); ++i) {
        perViewRms[i] = computeViewRms(objectPoints[i], imagePoints[i], calib1.rvecs[i], calib1.tvecs[i], calib1.K, calib1.D);
    }

    // Print per-view RMS for diagnostics
    cout << "\n--- Per-View Reprojection RMS (Stage-1) ---" << endl;
    double minRms = *std::min_element(perViewRms.begin(), perViewRms.end());
    double maxRms = *std::max_element(perViewRms.begin(), perViewRms.end());
    double avgRms = std::accumulate(perViewRms.begin(), perViewRms.end(), 0.0) / perViewRms.size();
    cout << "Min: " << minRms << " px, Max: " << maxRms << " px, Avg: " << avgRms << " px" << endl;
    
    // Show distribution
    int below1 = 0, below5 = 0, below10 = 0, above10 = 0;
    for (double rms : perViewRms) {
        if (rms < 1.0) below1++;
        else if (rms < 5.0) below5++;
        else if (rms < 10.0) below10++;
        else above10++;
    }
    cout << "Distribution: <1px: " << below1 << ", 1-5px: " << below5 
         << ", 5-10px: " << below10 << ", >10px: " << above10 << endl;
    
    // Populate report with per-view RMS statistics
    report.min_view_rms = minRms;
    report.max_view_rms = maxRms;
    report.avg_view_rms = avgRms;
    report.views_below_1px = below1;
    report.views_1_5px = below5;
    report.views_5_10px = below10;
    report.views_above_10px = above10;

    // Select inliers
    vector<int> indices(objectPoints.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](int a, int b) { return perViewRms[a] < perViewRms[b]; });

    size_t keepCount = indices.size();
    if (cfg.outlier_reject_top_fraction > 0.0 && cfg.outlier_reject_top_fraction < 1.0) {
        const size_t drop = static_cast<size_t>(std::floor(cfg.outlier_reject_top_fraction * static_cast<double>(indices.size())));
        if (drop < indices.size()) keepCount = indices.size() - drop;
    }

    vector<int> inlierIdx;
    inlierIdx.reserve(keepCount);
    for (size_t k = 0; k < keepCount; ++k) {
        const int idx = indices[k];
        if (cfg.outlier_reject_threshold_px > 0.0) {
            if (perViewRms[idx] <= cfg.outlier_reject_threshold_px) inlierIdx.push_back(idx);
        } else {
            inlierIdx.push_back(idx);
        }
    }

    if (inlierIdx.size() < 5) {
        cout << "Outlier rejection would leave too few views (" << inlierIdx.size() << "), skipping stage-2." << endl;
        inlierIdx.clear();
    }

    CalibResult calib2 = calib1;
    vector<vector<Point3f>> obj2;
    vector<vector<Point2f>> img2;
    if (!inlierIdx.empty()) {
        obj2.reserve(inlierIdx.size());
        img2.reserve(inlierIdx.size());
        for (int idx : inlierIdx) {
            obj2.push_back(objectPoints[idx]);
            img2.push_back(imagePoints[idx]);
        }
        cout << "Stage-2 recalibration using " << inlierIdx.size() << " inlier views..." << endl;
        calib2 = runFisheyeCalib(obj2, img2, imageSize, calib1.K, calib1.D);
        cout << "Stage-2 RMS (OpenCV): " << calib2.rms << " pixels" << endl;
        cout << "Stage-2 K:" << endl << calib2.K << endl;
        cout << "Stage-2 D (k1,k2,k3,k4): " << calib2.D.t() << endl;
    }
    
    // Populate report with stage-2 results
    report.stage2_rms = calib2.rms;
    report.stage2_views = (!inlierIdx.empty()) ? static_cast<int>(inlierIdx.size()) : report.stage1_views;
    report.fx = calib2.K.at<double>(0, 0);
    report.fy = calib2.K.at<double>(1, 1);
    report.cx = calib2.K.at<double>(0, 2);
    report.cy = calib2.K.at<double>(1, 2);

    const Mat& K = calib2.K;
    const Mat& D = calib2.D;
    const auto& rvecs = calib2.rvecs;
    const auto& tvecs = calib2.tvecs;
    const auto& objForErr = (!inlierIdx.empty()) ? obj2 : objectPoints;
    const auto& imgForErr = (!inlierIdx.empty()) ? img2 : imagePoints;
    const double rms = calib2.rms;

    // Calculate Reprojection Error for KB4 (Standard OpenCV)
    double totalErrorSq = 0;
    size_t totalPoints = 0;
    vector<Point2f> imagePoints2;

    for (size_t i = 0; i < objForErr.size(); i++) {
        fisheye::projectPoints(objForErr[i], imagePoints2, rvecs[i], tvecs[i], K, D);
        for (size_t j = 0; j < objForErr[i].size(); j++) {
            double dx = imgForErr[i][j].x - imagePoints2[j].x;
            double dy = imgForErr[i][j].y - imagePoints2[j].y;
            totalErrorSq += dx * dx + dy * dy;
        }
        totalPoints += objForErr[i].size();
    }
    
    double meanError = sqrt(totalErrorSq / totalPoints);
    cout << "Mean Reprojection Error (KB4): " << meanError << " pixels" << endl;

    // Model Conversion and Output
    FileStorage fs(cfg.output_file, FileStorage::WRITE);
    fs << "Model" << cfg.output_model;
    fs << "K" << K;
    fs << "BoardWidth" << cfg.board_width;
    fs << "BoardHeight" << cfg.board_height;
    fs << "SquareSize" << cfg.square_size;
    fs << "RMS" << rms;
    fs << "MeanReprojectionError" << meanError;
    fs << "UsedViews" << static_cast<int>((!inlierIdx.empty()) ? inlierIdx.size() : objectPoints.size());
    fs << "OutlierRejectTopFraction" << cfg.outlier_reject_top_fraction;
    fs << "OutlierRejectThresholdPx" << cfg.outlier_reject_threshold_px;

    // Always save the original KB4 distortion coefficients
    fs << "DistortionCoeffs_KB4" << D;
    
    if (cfg.output_model == "KB8") {
        cout << "Converting to KB8 model..." << endl;
        // KB8: theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + ... + k8*theta^16)
        // OpenCV KB4: theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8)
        // For KB8, the first 4 coefficients are the same as KB4, the rest are 0
        
        Mat kb8_coeffs = Mat::zeros(8, 1, CV_64F);
        kb8_coeffs.at<double>(0) = D.at<double>(0);  // k1
        kb8_coeffs.at<double>(1) = D.at<double>(1);  // k2
        kb8_coeffs.at<double>(2) = D.at<double>(2);  // k3
        kb8_coeffs.at<double>(3) = D.at<double>(3);  // k4
        // k5-k8 remain 0 (no higher order terms from KB4 calibration)
        
        fs << "DistortionCoeffs_KB8" << kb8_coeffs;
        cout << "KB8 Coefficients (k1-k8): " << kb8_coeffs.t() << endl;

    } else if (cfg.output_model == "OCam") {
        cout << "Fitting OCam model (Scaramuzza format)..." << endl;
        // OCam (Scaramuzza) model - standard format used by camera suppliers
        // 
        // Unprojection (image -> 3D ray):
        //   Given pixel (u, v), compute 3D direction
        //   x' = c*(u - cx) + d*(v - cy)
        //   y' = e*(u - cx) + (v - cy)
        //   rho = sqrt(x'^2 + y'^2)
        //   z = pol(rho) = a0 + a1*rho + a2*rho^2 + a3*rho^3 + a4*rho^4
        //   3D ray direction: (x', y', z) normalized
        //
        // Projection (3D -> image):
        //   Given 3D point (X, Y, Z), compute theta = atan2(sqrt(X^2+Y^2), Z)
        //   rho = invpol(theta) = b0 + b1*theta + b2*theta^2 + ... 
        //   Then project to image using affine params
        //
        // Key insight: pol(rho) gives the z-coordinate, and for equidistant projection
        // at rho=0, z should be pointing forward, so pol(0) = a0 ≈ -focal_length
        
        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double cx = K.at<double>(0, 2);
        double cy = K.at<double>(1, 2);
        
        // For 200° FOV, max_theta ~ 100° = 1.745 rad
        double max_theta = CV_PI * 100.0 / 180.0;
        int num_samples = 1000;
        
        vector<double> rho_samples, theta_samples;
        
        // Generate (rho, theta) pairs from KB4 model
        for (int i = 1; i <= num_samples; ++i) {
            double theta = max_theta * i / num_samples;
            double th2 = theta * theta;
            double th4 = th2 * th2;
            double th6 = th4 * th2;
            double th8 = th4 * th4;
            
            // KB4 model: theta_d = theta * (1 + k1*th^2 + k2*th^4 + k3*th^6 + k4*th^8)
            double theta_d = theta * (1.0 + D.at<double>(0)*th2 + D.at<double>(1)*th4 + 
                                      D.at<double>(2)*th6 + D.at<double>(3)*th8);
            double rho = fx * theta_d;  // radius in pixels
            
            rho_samples.push_back(rho);
            theta_samples.push_back(theta);
        }
        
        // ============ Forward polynomial (pol): z = f(rho) ============
        // OCam convention: z = pol(rho) where the 3D point is (x', y', z)
        // For a unit vector at angle theta from optical axis:
        //   x' = rho (the image distance from center)
        //   z = rho / tan(theta)  for perspective, but for fisheye it's different
        // 
        // The relationship is: if we have image point at distance rho from center,
        // the 3D ray has z = rho / tan(theta) where theta is the angle from optical axis
        // But we need to express z as polynomial in rho
        //
        // From rho and theta, the z-component scaled to match x'=rho:
        // z = rho * cos(theta) / sin(theta) = rho * cot(theta)
        // But for OCam, the convention is often z = -f + higher order terms
        
        int pol_order = 4;  // 5 coefficients (a0 to a4)
        Mat A_pol(num_samples, pol_order + 1, CV_64F);
        Mat b_pol(num_samples, 1, CV_64F);
        
        for (int i = 0; i < num_samples; ++i) {
            double rho = rho_samples[i];
            double theta = theta_samples[i];
            
            // z such that (rho, z) gives direction at angle theta
            // tan(theta) = rho / (-z)  =>  z = -rho / tan(theta)
            double z = -rho / tan(theta);
            
            double rho_pow = 1.0;
            for (int j = 0; j <= pol_order; ++j) {
                A_pol.at<double>(i, j) = rho_pow;
                rho_pow *= rho;
            }
            b_pol.at<double>(i, 0) = z;
        }
        
        Mat pol_coeffs;
        solve(A_pol, b_pol, pol_coeffs, DECOMP_SVD);
        
        // ============ Inverse polynomial (invpol): rho = g(theta) ============
        // invpol maps theta (angle from optical axis) to rho (image distance)
        // This is essentially the KB4 projection stored as polynomial
        
        int invpol_order = 12;  // 13 coefficients
        Mat A_invpol(num_samples, invpol_order + 1, CV_64F);
        Mat b_invpol(num_samples, 1, CV_64F);
        
        for (int i = 0; i < num_samples; ++i) {
            double theta = theta_samples[i];
            double rho = rho_samples[i];
            
            double theta_pow = 1.0;
            for (int j = 0; j <= invpol_order; ++j) {
                A_invpol.at<double>(i, j) = theta_pow;
                theta_pow *= theta;
            }
            b_invpol.at<double>(i, 0) = rho;
        }
        
        Mat invpol_coeffs;
        solve(A_invpol, b_invpol, invpol_coeffs, DECOMP_SVD);
        
        // Affine parameters (for non-square pixels or sensor misalignment)
        // ac (c): ~1.0 (aspect ratio), ad (d): ~0 (skew), ae (e): ~0 (skew)
        double ac = fy / fx;  // aspect ratio correction
        double ad = 0.0;      // skew parameter d
        double ae = 0.0;      // skew parameter e
        
        // Write OCam format matching supplier's YAML structure
        fs << "model_type" << "Ocam";
        fs << "camera_name" << "calibrated";
        fs << "image_width" << imageSize.width;
        fs << "image_height" << imageSize.height;
        
        // Forward polynomial (for unprojection)
        fs << "length_pol" << (pol_order + 1);
        // Transpose to row vector (1 x N) to match supplier format
        fs << "pol_data" << pol_coeffs.t();
        
        // Inverse polynomial (for projection)
        fs << "length_invpol" << (invpol_order + 1);
        // Transpose to row vector (1 x N) to match supplier format
        fs << "invpol_data" << invpol_coeffs.t();
        
        // Affine transformation parameters
        fs << "ac" << ac;
        fs << "ad" << ad;
        fs << "ae" << ae;
        
        // Principal point
        fs << "center_x" << cx;
        fs << "center_y" << cy;
        
        cout << "OCam Model Parameters:" << endl;
        cout << "  Image size: " << imageSize.width << "x" << imageSize.height << endl;
        cout << "  Center: (" << cx << ", " << cy << ")" << endl;
        cout << "  Affine: ac=" << ac << ", ad=" << ad << ", ae=" << ae << endl;
        cout << "  Forward pol (length=" << (pol_order+1) << "): " << pol_coeffs.t() << endl;
        cout << "  Inverse invpol (length=" << (invpol_order+1) << "): " << invpol_coeffs.t() << endl;
        
    } else if (cfg.output_model == "DS" || cfg.output_model == "DoubleSphere") {
        cout << "Fitting Double Sphere model (best for >180° FOV)..." << endl;
        // Double Sphere model: Good for ultra-wide FOV (180°+)
        // Parameters: fx, fy, cx, cy, xi, alpha
        // Projection: 
        //   d1 = sqrt(x^2 + y^2 + z^2)
        //   d2 = sqrt(x^2 + y^2 + (xi*d1 + z)^2)
        //   u = fx * x / (alpha*d2 + (1-alpha)*(xi*d1 + z)) + cx
        //   v = fy * y / (alpha*d2 + (1-alpha)*(xi*d1 + z)) + cy
        
        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double cx = K.at<double>(0, 2);
        double cy = K.at<double>(1, 2);
        
        // Fit xi and alpha by minimizing reprojection error over sampled angles
        // For 200° FOV, typical values: xi ~ 0.5-1.0, alpha ~ 0.5-0.6
        double best_xi = 0.0, best_alpha = 0.5;
        double best_error = 1e10;
        
        // Sample theta values up to ~100° (half of 200° FOV)
        double max_theta = CV_PI * 100.0 / 180.0;
        int num_samples = 500;
        
        // Grid search for xi and alpha
        for (double xi = -0.5; xi <= 1.5; xi += 0.05) {
            for (double alpha = 0.0; alpha <= 1.0; alpha += 0.05) {
                double total_error = 0.0;
                int valid_samples = 0;
                
                for (int i = 1; i <= num_samples; ++i) {
                    double theta = max_theta * i / num_samples;
                    
                    // KB4 model: get distorted radius
                    double th2 = theta * theta;
                    double th4 = th2 * th2;
                    double th6 = th4 * th2;
                    double th8 = th4 * th4;
                    double theta_d = theta * (1.0 + D.at<double>(0)*th2 + D.at<double>(1)*th4 + 
                                              D.at<double>(2)*th6 + D.at<double>(3)*th8);
                    double r_kb4 = fx * theta_d;  // KB4 radius in pixels
                    
                    // Double Sphere model: compute radius for same 3D direction
                    // 3D point on unit sphere: (sin(theta), 0, cos(theta))
                    double x = sin(theta);
                    double z = cos(theta);
                    double d1 = 1.0;  // sqrt(x^2 + z^2) = 1 for unit sphere
                    double d2 = sqrt(x*x + (xi*d1 + z)*(xi*d1 + z));
                    double denom = alpha*d2 + (1.0-alpha)*(xi*d1 + z);
                    
                    if (denom > 0.001) {
                        double r_ds = fx * x / denom;  // DS radius in pixels
                        double err = r_kb4 - r_ds;
                        total_error += err * err;
                        valid_samples++;
                    }
                }
                
                if (valid_samples > 0) {
                    double rms_error = sqrt(total_error / valid_samples);
                    if (rms_error < best_error) {
                        best_error = rms_error;
                        best_xi = xi;
                        best_alpha = alpha;
                    }
                }
            }
        }
        
        fs << "DoubleSphere_fx" << fx;
        fs << "DoubleSphere_fy" << fy;
        fs << "DoubleSphere_cx" << cx;
        fs << "DoubleSphere_cy" << cy;
        fs << "DoubleSphere_xi" << best_xi;
        fs << "DoubleSphere_alpha" << best_alpha;
        fs << "DoubleSphere_FitError" << best_error;
        
        cout << "Double Sphere Parameters:" << endl;
        cout << "  fx=" << fx << ", fy=" << fy << ", cx=" << cx << ", cy=" << cy << endl;
        cout << "  xi=" << best_xi << ", alpha=" << best_alpha << endl;
        cout << "  Fit error (vs KB4): " << best_error << " pixels" << endl;
        
    } else if (cfg.output_model == "EUCM") {
        cout << "Fitting Extended Unified Camera Model (EUCM)..." << endl;
        // EUCM parameters: fx, fy, cx, cy, alpha, beta
        // Projection:
        //   d = sqrt(beta*(x^2+y^2) + z^2)
        //   u = fx * x / (alpha*d + (1-alpha)*z) + cx
        //   v = fy * y / (alpha*d + (1-alpha)*z) + cy
        
        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double cx = K.at<double>(0, 2);
        double cy = K.at<double>(1, 2);
        
        double best_alpha = 0.5, best_beta = 1.0;
        double best_error = 1e10;
        
        double max_theta = CV_PI * 100.0 / 180.0;
        int num_samples = 500;
        
        // Grid search for alpha and beta
        for (double alpha = 0.0; alpha <= 1.0; alpha += 0.05) {
            for (double beta = 0.5; beta <= 2.0; beta += 0.05) {
                double total_error = 0.0;
                int valid_samples = 0;
                
                for (int i = 1; i <= num_samples; ++i) {
                    double theta = max_theta * i / num_samples;
                    
                    // KB4 model
                    double th2 = theta * theta;
                    double th4 = th2 * th2;
                    double th6 = th4 * th2;
                    double th8 = th4 * th4;
                    double theta_d = theta * (1.0 + D.at<double>(0)*th2 + D.at<double>(1)*th4 + 
                                              D.at<double>(2)*th6 + D.at<double>(3)*th8);
                    double r_kb4 = fx * theta_d;
                    
                    // EUCM model
                    double x = sin(theta);
                    double z = cos(theta);
                    double d = sqrt(beta*x*x + z*z);
                    double denom = alpha*d + (1.0-alpha)*z;
                    
                    if (denom > 0.001) {
                        double r_eucm = fx * x / denom;
                        double err = r_kb4 - r_eucm;
                        total_error += err * err;
                        valid_samples++;
                    }
                }
                
                if (valid_samples > 0) {
                    double rms_error = sqrt(total_error / valid_samples);
                    if (rms_error < best_error) {
                        best_error = rms_error;
                        best_alpha = alpha;
                        best_beta = beta;
                    }
                }
            }
        }
        
        fs << "EUCM_fx" << fx;
        fs << "EUCM_fy" << fy;
        fs << "EUCM_cx" << cx;
        fs << "EUCM_cy" << cy;
        fs << "EUCM_alpha" << best_alpha;
        fs << "EUCM_beta" << best_beta;
        fs << "EUCM_FitError" << best_error;
        
        cout << "EUCM Parameters:" << endl;
        cout << "  fx=" << fx << ", fy=" << fy << ", cx=" << cx << ", cy=" << cy << endl;
        cout << "  alpha=" << best_alpha << ", beta=" << best_beta << endl;
        cout << "  Fit error (vs KB4): " << best_error << " pixels" << endl;
        
    } else {
        // Default KB4
        fs << "DistortionCoeffs" << D;
    }

    fs.release();
    cout << "Calibration saved to " << cfg.output_file << endl;

    // Undistort a sample image with proper handling for ultra-wide FOV
    cap.open(cfg.video_path);
    Mat sampleFrame;
    if (cap.read(sampleFrame)) {
        Mat undistorted;
        Mat map1, map2;
        
        // Option 1: Use original K matrix (preserves aspect ratio but may crop)
        // This is the most reliable for fisheye lenses
        Mat newK = K.clone();
        
        // Option 2: Scale down focal length to fit more of the undistorted image
        // For ultra-wide FOV (200°), use a scaling factor to reduce distortion stretch
        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double cx = K.at<double>(0, 2);
        double cy = K.at<double>(1, 2);
        
        // Scale focal lengths by 0.4-0.6 for 200° FOV to reduce stretching
        // Lower value = wider FOV but more stretch at edges
        // Higher value = less stretch but crops more
        double fov_scale = cfg.undistort_fov_scale;  // Adjust this if needed (0.3-0.8 range)
        
        newK.at<double>(0, 0) = fx * fov_scale;  // fx
        newK.at<double>(1, 1) = fy * fov_scale;  // fy
        newK.at<double>(0, 2) = cx;              // cx (keep centered)
        newK.at<double>(1, 2) = cy;              // cy (keep centered)
        
        fisheye::initUndistortRectifyMap(K, D, Mat::eye(3, 3, CV_64F), newK, imageSize, CV_16SC2, map1, map2);
        remap(sampleFrame, undistorted, map1, map2, INTER_LINEAR);

        imwrite(cfg.undistorted_image_output, undistorted);
        cout << "Undistorted sample saved to " << cfg.undistorted_image_output << endl;
    }

    // Generate and display calibration report
    generateReport(report, cfg.report_file, cfg);

    return 0;
}
