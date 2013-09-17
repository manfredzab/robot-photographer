/**
 * @brief     Camera calibration tool, which is used to calibrate the photographic camera (by
 *            removing tangential and radial lens distortion), which is necessary for
 *            photographic camera and depth camera alignment.
 *
 * @copyright Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *            CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */

// C++ includes
#include <iostream>
#include <cstdio>

// STL includes
#include <string>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string CALIBRATION_WINDOW_TITLE = "Calibration output";
static const bool        SHOW_UNDISTORTED = true;
static const cv::Size    BOARD_SIZE(8, 6);
static const float       SQUARE_SIZE = 0.03f; // Meters
static const int         IMAGE_COUNT = 49;
static const char*       IMAGE_FILE_TEMPLATE = "P (%d).JPG";
static const std::string OUTPUT_INTRINSICS_FILE = "intrinsics.txt";

/**
 * Calibrates the camera and saves the obtained camera matrix/distortion coefficients in a given
 * output file.
 * @param image_size                Input image size.
 * @param camera_matrix             Resulting camera matrix.
 * @param distortion_coefficients   Resulting distortion coefficients.
 * @param image_corner_points       Corner points in images.
 * @param output_file               Output file name.
 * @returns Operation's success status.
 */
bool calibrateCameraAndSaveIntrinsics(cv::Size image_size, cv::Mat& camera_matrix, cv::Mat& distortion_coefficients, std::vector<std::vector<cv::Point2f> > image_corner_points, const std::string& output_file);

/**
 * Camera calibration tool's entry point.
 * @param argc Argument count.
 * @param argv Arguments (not used).
 * @returns Exit status (should be 0).
 */
int main(int argc, char* argv[])
{
    cv::namedWindow(CALIBRATION_WINDOW_TITLE, 0);
    cv::resizeWindow(CALIBRATION_WINDOW_TITLE, 800, 600);

    cv::Size image_size;
    std::vector<std::vector<cv::Point2f> > image_corner_points;
    // Detect chessboard corners in each of the input images
    for (int i = 0; i < IMAGE_COUNT; i++)
    {
        char image_filename[256];
        sprintf(image_filename, IMAGE_FILE_TEMPLATE, i + 1);

        cv::Mat image = cv::imread(image_filename, CV_LOAD_IMAGE_COLOR);
        image_size = image.size();

        std::vector<cv::Point2f> corners;
        bool corners_found = cv::findChessboardCorners(image, BOARD_SIZE, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        if (corners_found)
        {
            cv::Mat image_grayscale;
            cv::cvtColor(image, image_grayscale, CV_BGR2GRAY);
            cv::cornerSubPix(image_grayscale, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            image_corner_points.push_back(corners);

            cv::drawChessboardCorners(image, BOARD_SIZE, cv::Mat(corners), corners_found);
        }
        else
        {
            std::cout << "WARNING: chessboard corners not found in image " << image_filename << "." << std::endl;
        }

        cv::imshow(CALIBRATION_WINDOW_TITLE, image);
        cv::waitKey(30);
    }

    // Run camera calibration
    cv::Mat camera_matrix, distortion_coefficients;
    calibrateCameraAndSaveIntrinsics(image_size, camera_matrix, distortion_coefficients, image_corner_points, OUTPUT_INTRINSICS_FILE);

    // Show undistorted images
    if (SHOW_UNDISTORTED)
    {
        cv::Mat image, undistorted_image, map_x, map_y;
        cv::initUndistortRectifyMap(camera_matrix, distortion_coefficients, cv::Mat(), camera_matrix, image_size, CV_16SC2, map_x, map_y);

        for (int i = 0; i < IMAGE_COUNT; i++)
        {
            char image_filename[256];
            sprintf(image_filename, IMAGE_FILE_TEMPLATE, i + 1);

            image = cv::imread(image_filename, 1);
            cv::remap(image, undistorted_image, map_x, map_y, cv::INTER_LINEAR);
            cv::imshow(CALIBRATION_WINDOW_TITLE, undistorted_image);
            cv::waitKey();
        }
    }

    return 0;
}

/**
 * Calculates the average reprojection error.
 * @param object_points              Calculated chessboard corner points in 3D.
 * @param image_corner_points        Chessboard corner points in training images.
 * @param output_rotation_vectors    Rotation vectors (from calibration).
 * @param output_translation_vectors Translation vectors (from calibration).
 * @param camera_matrix              Camera matrix (from calibration).
 * @param distortion_coefficients    Distortion coefficients (from calibration).
 * @returns Average reprojection error.
 */
static double calculateReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& object_points,
                                          const std::vector<std::vector<cv::Point2f> >& image_corner_points,
                                          const std::vector<cv::Mat>& output_rotation_vectors,
                                          const std::vector<cv::Mat>& output_translation_vectors,
                                          const cv::Mat& camera_matrix,
                                          const cv::Mat& distortion_coefficients,
                                          std::vector<float>& reprojection_errors)
{
    reprojection_errors.resize(object_points.size());

    double cumulative_error_square = 0.0;
    for (unsigned i = 0; i < IMAGE_COUNT; i++)
    {
        std::vector<cv::Point2f> projected_image_corner_points;
        cv::projectPoints(object_points[i],
                      output_rotation_vectors[i],
                      output_translation_vectors[i],
                      camera_matrix,
                      distortion_coefficients,
                      projected_image_corner_points);

        double projection_error = norm(cv::Mat(image_corner_points[i]), cv::Mat(projected_image_corner_points), CV_L2);
        double projection_error_square = projection_error * projection_error;

        cumulative_error_square += projection_error_square;

        double average_projection_error_per_image = std::sqrt(projection_error_square / (double)BOARD_SIZE.area());
        reprojection_errors[i] = (float)average_projection_error_per_image;
    }

    return std::sqrt(cumulative_error_square / (double)(IMAGE_COUNT * BOARD_SIZE.area()));
}

/**
 * Calculates the coordinates of the corner points on the chessboard.
 * @param object_points Output chessboard corner points in 3D.
 */
static void calculateObjectPoints(std::vector<std::vector<cv::Point3f> >& object_points)
{
    object_points.resize(1);
    for (int y = 0; y < BOARD_SIZE.height; y++)
    {
        for (int x = 0; x < BOARD_SIZE.width; x++)
        {
            object_points[0].push_back(cv::Point3f((float)x * SQUARE_SIZE, (float)y * SQUARE_SIZE, 0.0f));
        }
    }
    object_points.resize(IMAGE_COUNT, object_points[0]);
}

/**
 * Performs the camera calibration.
 * @param image_size                 Training image size.
 * @param camera_matrix              Calibrated camera matrix.
 * @param distortion_coefficients    Calibrated distortion coefficients.
 * @param image_corner_points        Chessboard corner points in training images.
 * @param output_rotation_vectors    Rotation vectors for chessboards.
 * @param output_translation_vectors Translation vectors for chessboards.
 * @param reprojection_errors        Reprojection errors for each of the chessboards.
 * @param average_error              Average reprojection error.
 * @returns Calibration's success status.
 */
static bool runCalibration(cv::Size& image_size,
                           cv::Mat& camera_matrix,
                           cv::Mat& distortion_coefficients,
                           std::vector<std::vector<cv::Point2f> > image_corner_points,
                           std::vector<cv::Mat>& output_rotation_vectors,
                           std::vector<cv::Mat>& output_translation_vectors,
                           std::vector<float>& reprojection_errors,
                           double& average_error)
{
    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    distortion_coefficients = cv::Mat::zeros(8, 1, CV_64F);

    // Create the array of object point arrays
    std::vector<std::vector<cv::Point3f> > object_points;
    calculateObjectPoints(object_points);

    // Find camera parameters
    double reprojection_error = calibrateCamera(object_points,
                                                image_corner_points,
                                                image_size,
                                                camera_matrix,
                                                distortion_coefficients,
                                                output_rotation_vectors,
                                                output_translation_vectors,
                                                CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

    bool status = checkRange(camera_matrix) && checkRange(distortion_coefficients);

    average_error = calculateReprojectionErrors(object_points,
                                              image_corner_points,
                                              output_rotation_vectors,
                                              output_translation_vectors,
                                              camera_matrix,
                                              distortion_coefficients,
                                              reprojection_errors);

    return status;
}

/**
 * Saves camera intrinsics to a given file.
 * @param camera_matrix              Calibrated camera matrix.
 * @param distortion_coefficients    Calibrated distortion coefficients.
 * @param reprojection_errors        Reprojection errors for each of the chessboards.
 * @param average_error              Average reprojection error.
 * @param output_file                Output file name.
 */
static void saveCameraIntrinsics(cv::Mat& camera_matrix,
                                 cv::Mat& distortion_coefficients,
                                 const std::vector<float>& reprojection_errors,
                                 double average_error,
                                 const std::string& output_file)
{
    cv::FileStorage file_storage(output_file, cv::FileStorage::WRITE);

    file_storage << "Camera_matrix" << camera_matrix;
    file_storage << "Distortion_coefficients" << distortion_coefficients;

    file_storage << "Average_reprojection_error" << average_error;
    if (!reprojection_errors.empty())
    {
        file_storage << "Individual_image_reprojection_errors" << cv::Mat(reprojection_errors);
    }
}

/**
 * Calibrates the photographic camera and saves the camera intrinsics to a given file.
 * @param image_size                 Training image size.
 * @param camera_matrix              Calibrated camera matrix.
 * @param distortion_coefficients    Calibrated distortion coefficients.
 * @param image_corner_points        Corner points in training images.
 * @param output_file                Output file name.
 */
bool calibrateCameraAndSaveIntrinsics(cv::Size image_size,
                                      cv::Mat& camera_matrix,
                                      cv::Mat& distortion_coefficients,
                                      std::vector<std::vector<cv::Point2f> > image_corner_points,
                                      const std::string& output_file)
{
    std::vector<cv::Mat> output_rotation_vectors, output_translation_vectors;
    std::vector<float> reprojection_errors;

    double average_error = 0.0;
    bool status = runCalibration(image_size,
                                 camera_matrix,
                                 distortion_coefficients,
                                 image_corner_points,
                                 output_rotation_vectors,
                                 output_translation_vectors,
                                 reprojection_errors,
                                 average_error);

    std::cout << (status ? "Calibration succeeded" : "Calibration failed") << "." << std::endl;
    std::cout << "Reprojection error: " << average_error;

    if (status)
    {
        saveCameraIntrinsics(camera_matrix, distortion_coefficients, reprojection_errors, average_error, output_file);
    }
    return status;
}
