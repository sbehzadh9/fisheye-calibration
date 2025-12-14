#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <numeric>
#include <algorithm>

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
    return cfg;
}

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

    cout << "Processing video..." << endl;

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
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));
                
                imagePoints.push_back(corners);
                objectPoints.push_back(obj);
                validFrames++;

                if (cfg.max_frames > 0 && validFrames >= cfg.max_frames) {
                    cout << "Reached max_frames=" << cfg.max_frames << ", stopping capture." << endl;
                    break;
                }
                
                if (cfg.show_detection) {
                    Mat displayFrame = frame.clone();
                    drawChessboardCorners(displayFrame, Size(cfg.board_width, cfg.board_height), corners, found);
                    
                    // Add text showing frame info
                    string text = "Frame: " + to_string(frameCount) + " | Valid: " + to_string(validFrames);
                    putText(displayFrame, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                    
                    imshow("Detection", displayFrame);
                    waitKey(1);
                }
                cout << "Found corners in frame " << frameCount << ". Total valid frames: " << validFrames << "\r" << flush;
            }
        }
        frameCount++;
    }
    cout << endl;
    
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
        cout << "Fitting OCam model..." << endl;
        // OCam (Scaramuzza): Projects world point to image using polynomial
        // Forward model: rho = invpol(theta) where invpol is a polynomial
        // Inverse model: theta = pol(rho) - used for unprojection
        // We fit the inverse polynomial: theta = a0 + a1*rho + a2*rho^2 + ... + aN*rho^N
        
        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double f_avg = (fx + fy) / 2.0;
        
        vector<double> rho_data, theta_data_ocam;
        double max_theta = CV_PI / 2.0 * 0.95; // Avoid singularity
        int num_samples = 1000;
        
        for (int i = 0; i < num_samples; ++i) {
            double th = max_theta * (i + 1) / num_samples;
            double th2 = th * th;
            double th4 = th2 * th2;
            double th6 = th4 * th2;
            double th8 = th4 * th4;
            
            // OpenCV KB4 model
            double scale = 1.0 + D.at<double>(0) * th2 + D.at<double>(1) * th4 + 
                           D.at<double>(2) * th6 + D.at<double>(3) * th8;
            double th_d = th * scale;
            double rho = f_avg * th_d; // Distorted radius in pixels
            
            rho_data.push_back(rho);
            theta_data_ocam.push_back(th); // Original angle
        }
        
        // Fit polynomial: theta = a0 + a1*rho + a2*rho^2 + ... + a5*rho^5
        int poly_order = 5;
        Mat A(num_samples, poly_order + 1, CV_64F);
        Mat b(num_samples, 1, CV_64F);
        
        for (int i = 0; i < num_samples; ++i) {
            double rho = rho_data[i];
            double theta = theta_data_ocam[i];
            
            double rho_pow = 1.0;
            for (int j = 0; j <= poly_order; ++j) {
                A.at<double>(i, j) = rho_pow;
                rho_pow *= rho;
            }
            b.at<double>(i, 0) = theta;
        }
        
        Mat coeffs;
        solve(A, b, coeffs, DECOMP_SVD);
        
        fs << "DistortionCoeffs_OCam" << coeffs;
        fs << "CenterX" << K.at<double>(0, 2);
        fs << "CenterY" << K.at<double>(1, 2);
        fs << "AffineC" << 1.0;  // c parameter (typically 1.0)
        fs << "AffineD" << 0.0;  // d parameter (typically 0.0)
        fs << "AffineE" << 1.0;  // e parameter (typically 1.0)
        cout << "OCam Polynomial Coefficients (a0-a5): " << coeffs.t() << endl;
        
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

    // Undistort a sample image
    // We'll use the last valid frame we captured (or reload the video if needed, but let's just use the last 'frame' variable if it's not empty, 
    // actually 'frame' is empty at loop exit. Let's reload the video or just keep one valid frame).
    // Better: capture a frame specifically for undistortion or use the first one.
    
    cap.open(cfg.video_path);
    Mat sampleFrame;
    if (cap.read(sampleFrame)) {
        Mat undistorted;
        Mat map1, map2;
        // New camera matrix for undistortion (can be same as K or optimized)
        Mat newK = K.clone(); 
        // Balance sets the new focal length. 0.0 to 1.0. 
        // fisheye::estimateNewCameraMatrixForUndistortRectify is useful here.
        double balance = 0.5; // Adjust balance between 0.0 (max FOV) and 1.0 (no distortion)
        fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, imageSize, Mat::eye(3, 3, CV_64F), newK, balance, imageSize, 1.0);
        
        fisheye::initUndistortRectifyMap(K, D, Mat::eye(3, 3, CV_64F), newK, imageSize, CV_16SC2, map1, map2);
        remap(sampleFrame, undistorted, map1, map2, INTER_LINEAR);

        imwrite(cfg.undistorted_image_output, undistorted);
        cout << "Undistorted sample saved to " << cfg.undistorted_image_output << endl;
    }

    return 0;
}
