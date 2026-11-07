#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace
{
bool ParseInt(const char* text, int& value)
{
    try
    {
        std::size_t parsed = 0;
        const int result = std::stoi(text, &parsed);
        if (parsed != std::string{text}.size())
        {
            return false;
        }
        value = result;
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool ParsePositiveInt(const char* text, int& value)
{
    return ParseInt(text, value) && value > 0;
}

bool ParsePositiveDouble(const char* text, double& value)
{
    try
    {
        std::size_t parsed = 0;
        const double result = std::stod(text, &parsed);
        if (parsed != std::string{text}.size() ||
            !std::isfinite(result) || result <= 0.0)
        {
            return false;
        }
        value = result;
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

void PrintUsage(const char* exe)
{
    std::cerr << "Usage: " << exe
              << " <inner_cols> <inner_rows> <square_size_mm> <output.yml>"
                 " [camera_index] [width] [height]\n"
              << "Example: " << exe
              << " 9 6 25 camera_intrinsics.yml 0 1280 720\n";
}

std::vector<cv::Point3f> MakeBoardPoints(cv::Size boardSize,
                                         double squareSizeMm)
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(boardSize.area()));
    for (int row = 0; row < boardSize.height; ++row)
    {
        for (int col = 0; col < boardSize.width; ++col)
        {
            points.emplace_back(
                static_cast<float>(col * squareSizeMm),
                static_cast<float>(row * squareSizeMm), 0.0F);
        }
    }
    return points;
}

bool CalibrateAndSave(
    const std::string& outputPath,
    cv::Size imageSize,
    cv::Size boardSize,
    double squareSizeMm,
    const std::vector<std::vector<cv::Point3f>>& objectPoints,
    const std::vector<std::vector<cv::Point2f>>& imagePoints)
{
    if (imagePoints.size() < 5)
    {
        std::cerr << "有效照片不足 5 张，当前为 " << imagePoints.size()
                  << " 张；请继续采集。\n";
        return false;
    }
    if (imagePoints.size() < 15)
    {
        std::cerr << "警告: 有效照片少于 15 张，建议增加不同位置和倾斜角度的照片。\n";
    }

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    double calibrationRms = 0.0;
    try
    {
        calibrationRms = cv::calibrateCamera(
            objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs,
            rvecs, tvecs);
    }
    catch (const cv::Exception& ex)
    {
        std::cerr << "OpenCV 标定失败: " << ex.what() << '\n';
        return false;
    }

    if (!std::isfinite(calibrationRms) ||
        !cv::checkRange(cameraMatrix) || !cv::checkRange(distCoeffs))
    {
        std::cerr << "标定结果包含无效数值。\n";
        return false;
    }

    double totalErrSq = 0.0;
    std::size_t totalCorners = 0;
    for (std::size_t i = 0; i < objectPoints.size(); ++i)
    {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i],
                          cameraMatrix, distCoeffs, projected);
        double viewErrSq = 0.0;
        for (std::size_t j = 0; j < projected.size(); ++j)
        {
            const cv::Point2f err = projected[j] - imagePoints[i][j];
            viewErrSq += err.dot(err);
        }
        totalErrSq += viewErrSq;
        totalCorners += projected.size();
        std::cout << "照片 " << i + 1 << " 重投影 RMS: "
                  << std::sqrt(viewErrSq / projected.size()) << " px\n";
    }

    const double reprojectionRms =
        std::sqrt(totalErrSq / static_cast<double>(totalCorners));
    try
    {
        cv::FileStorage output(outputPath, cv::FileStorage::WRITE);
        if (!output.isOpened())
        {
            std::cerr << "无法创建标定结果文件: " << outputPath << '\n';
            return false;
        }
        output << "image_width" << imageSize.width;
        output << "image_height" << imageSize.height;
        output << "board_inner_cols" << boardSize.width;
        output << "board_inner_rows" << boardSize.height;
        output << "square_size_mm" << squareSizeMm;
        output << "used_images" << static_cast<int>(imagePoints.size());
        output << "calibration_rms_px" << calibrationRms;
        output << "reprojection_rms_px" << reprojectionRms;
        output << "camera_matrix" << cameraMatrix;
        output << "dist_coeffs" << distCoeffs;
    }
    catch (const cv::Exception& ex)
    {
        std::cerr << "写入标定结果失败: " << ex.what() << '\n';
        return false;
    }

    std::cout << "\n标定完成，使用照片数: " << imagePoints.size()
              << "\n图像分辨率: " << imageSize.width << 'x' << imageSize.height
              << "\n标定 RMS: " << calibrationRms << " px"
              << "\n整体重投影 RMS: " << reprojectionRms << " px"
              << "\n相机矩阵:\n" << cameraMatrix
              << "\n畸变系数 [k1, k2, p1, p2, k3]: " << distCoeffs
              << "\n结果已保存到: " << outputPath << '\n';
    return true;
}
}

int main(int argc, char** argv)
{
    if (argc < 5 || argc > 8)
    {
        PrintUsage(argv[0]);
        return 2;
    }

    int innerCols = 0;
    int innerRows = 0;
    double squareSizeMm = 0.0;
    int cameraIndex = 0;
    int requestedWidth = 1280;
    int requestedHeight = 720;
    constexpr int kRequestedFps = 30;
    if (!ParsePositiveInt(argv[1], innerCols) || innerCols < 2 ||
        !ParsePositiveInt(argv[2], innerRows) || innerRows < 2 ||
        !ParsePositiveDouble(argv[3], squareSizeMm) ||
        (argc >= 6 && (!ParseInt(argv[5], cameraIndex) || cameraIndex < 0)) ||
        (argc >= 7 && !ParsePositiveInt(argv[6], requestedWidth)) ||
        (argc >= 8 && !ParsePositiveInt(argv[7], requestedHeight)))
    {
        std::cerr << "棋盘参数、相机编号或分辨率无效。\n";
        PrintUsage(argv[0]);
        return 2;
    }

    cv::VideoCapture camera;
    try
    {
        if (!camera.open(cameraIndex))
        {
            std::cerr << "相机打开失败，camera_index=" << cameraIndex << '\n';
            return 1;
        }
        const bool mjpgSet = camera.set(
            cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        const bool widthSet =
            camera.set(cv::CAP_PROP_FRAME_WIDTH, requestedWidth);
        const bool heightSet =
            camera.set(cv::CAP_PROP_FRAME_HEIGHT, requestedHeight);
        const bool fpsSet = camera.set(cv::CAP_PROP_FPS, kRequestedFps);
        if (!mjpgSet || !widthSet || !heightSet || !fpsSet)
        {
            std::cerr << "警告: 相机驱动未确认接受 MJPG/分辨率/FPS 的全部请求；"
                         "将检查实际图像分辨率。\n";
        }
    }
    catch (const cv::Exception& ex)
    {
        std::cerr << "打开或配置相机失败: " << ex.what() << '\n';
        return 1;
    }

    const cv::Size boardSize{innerCols, innerRows};
    const std::vector<cv::Point3f> boardPoints =
        MakeBoardPoints(boardSize, squareSizeMm);
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize;
    bool saved = false;
    std::cout << "请求采集格式: MJPG, " << requestedWidth << 'x'
              << requestedHeight << " @ " << kRequestedFps << " FPS\n"
              << "空格: 检测到棋盘时采集照片 | U: 撤销一张 | "
                 "C: 标定并保存 | Q: 退出\n";

    cv::namedWindow("Camera Calibration", cv::WINDOW_NORMAL);
    cv::Mat frame;
    cv::Mat display;
    bool captureFailed = false;
    for (;;)
    {
        if (!camera.read(frame) || frame.empty())
        {
            std::cerr << "相机取帧失败。\n";
            captureFailed = true;
            break;
        }
        if (imageSize.empty())
        {
            imageSize = frame.size();
            std::cout << "相机实际输出分辨率: " << imageSize.width << 'x'
                      << imageSize.height << '\n';
            if (imageSize != cv::Size{requestedWidth, requestedHeight})
            {
                std::cerr << "实际分辨率与请求不符；请检查相机支持的模式，"
                             "不会继续标定。\n";
                captureFailed = true;
                break;
            }
        }
        else if (frame.size() != imageSize)
        {
            std::cerr << "相机输出分辨率发生变化，停止采集。\n";
            captureFailed = true;
            break;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        const bool found = cv::findChessboardCorners(
            gray, boardSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found)
        {
            cv::cornerSubPix(
                gray, corners, cv::Size{11, 11}, cv::Size{-1, -1},
                cv::TermCriteria{
                    cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.001});
        }

        display = frame.clone();
        if (found)
        {
            cv::drawChessboardCorners(display, boardSize, corners, found);
        }
        cv::putText(display,
                    "usable: " + std::to_string(imagePoints.size()) +
                        " | " + (found ? "BOARD OK" : "BOARD NOT FOUND"),
                    cv::Point{16, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    found ? cv::Scalar{0, 255, 0} : cv::Scalar{0, 0, 255}, 2);
        cv::putText(display,
                    "SPACE capture  U undo  C calibrate/save  Q quit",
                    cv::Point{16, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar{0, 255, 255}, 2);
        cv::imshow("Camera Calibration", display);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27)
        {
            break;
        }
        if (key == ' ')
        {
            if (!found)
            {
                std::cout << "当前画面未检测到完整棋盘，不计入照片数。\n";
                continue;
            }
            imagePoints.push_back(std::move(corners));
            objectPoints.push_back(boardPoints);
            saved = false;
            std::cout << "采集有效照片: " << imagePoints.size() << '\n';
        }
        else if (key == 'u' || key == 'U')
        {
            if (!imagePoints.empty())
            {
                imagePoints.pop_back();
                objectPoints.pop_back();
                saved = false;
                std::cout << "撤销一张，当前有效照片数: "
                          << imagePoints.size() << '\n';
            }
        }
        else if (key == 'c' || key == 'C')
        {
            saved = CalibrateAndSave(
                argv[4], imageSize, boardSize, squareSizeMm,
                objectPoints, imagePoints);
        }
    }

    camera.release();
    cv::destroyAllWindows();
    if (!saved)
    {
        std::cout << "退出。有效照片数: " << imagePoints.size()
                  << "；本次未保存新的标定结果。\n";
    }
    return captureFailed ? 1 : 0;
}
