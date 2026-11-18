#include "DashBoard.h"

#include <iostream>

int main()
{
    DashBoard dashboard;
    dashboard.setLogField("first", 1);
    const cv::Mat first = dashboard.renderLog();
    dashboard.setLogField("first", 2);
    const cv::Mat updated = dashboard.renderLog();
    if (first.size() != updated.size() || cv::norm(first, updated, cv::NORM_L1) == 0)
    {
        std::cerr << "Updating a field did not replace its displayed value\n";
        return 1;
    }
    dashboard.setLogField("second", "a longer value");
    dashboard.setLogField("third", 3.14);
    if (dashboard.renderLog().rows <= updated.rows)
    {
        std::cerr << "New fields did not add rows\n";
        return 1;
    }

    cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC3);
    cv::Mat mask = cv::Mat::zeros(120, 160, CV_8UC1);
    bool fire = false;
    TargetColor color = TargetColor::Any;
    if (!dashboard.handleKey('F', image, mask, fire, color) || !fire ||
        !dashboard.handleKey('c', image, mask, fire, color) || color != TargetColor::Red ||
        !dashboard.handleKey('C', image, mask, fire, color) || color != TargetColor::Blue ||
        !dashboard.handleKey('c', image, mask, fire, color) || color != TargetColor::Any ||
        dashboard.handleKey(27, image, mask, fire, color))
    {
        std::cerr << "Keyboard controls failed\n";
        return 1;
    }

    DashboardFrame frame{};
    frame.aimPoint = {80, 60};
    frame.aimTolerance = 10;
    TargetPx target;
    target.valid = true;
    target.center = {50, 50};
    target.radius = 5;
    frame.target.push_back(target);
    frame.feedbackValid = true;
    frame.targetProjectionValid = true;
    frame.center = {0.0, 0.0, -1.0};
    frame.predictedTarget = frame.center;
    dashboard.updateLog(frame);
    dashboard.draw(image, frame, CameraProjection{});
    if (cv::norm(image, cv::NORM_L1) == 0)
    {
        std::cerr << "Dashboard did not draw the target\n";
        return 1;
    }

    TargetMatcher matcher;
    const frc::Translation3d center{2.0, 0.0, 0.0};
    const frc::Translation3d firstTarget{2.0, -0.4, 0.0};
    if (!matcher.markers(0.4).empty())
    {
        std::cerr << "Uninitialized matcher produced markers\n";
        return 1;
    }
    matcher.initialize(center, firstTarget, 0);
    auto markers = matcher.markers(0.4);
    if (markers.size() != 5 || markers[0].active ||
        (markers[0].position - firstTarget).Norm().value() > 1e-6)
    {
        std::cerr << "Initialized target markers are incorrect\n";
        return 1;
    }
    if (!matcher.select({firstTarget}, 0, 0.0))
    {
        std::cerr << "Could not select first target\n";
        return 1;
    }
    matcher.markFired();
    markers = matcher.markers(0.4);
    if (!markers[0].active || markers[1].active)
    {
        std::cerr << "Target marker states are incorrect\n";
        return 1;
    }

    CameraProjectionConfig config;
    config.fx = config.fy = 100.0;
    config.cx = 80.0;
    config.cy = 60.0;
    CameraProjection projection{config};
    frame.target.clear();
    frame.targetMarkers = markers;
    frame.center = center;
    frame.radius = 0.4;
    frame.aimPoint = {0, 0};
    image = cv::Mat::zeros(120, 160, CV_8UC3);
    dashboard.draw(image, frame, projection);
    const auto hit = projection.worldToPixel(markers[0].position, frame.gimbalAngles);
    const auto open = projection.worldToPixel(markers[1].position, frame.gimbalAngles);
    if (!hit || !open ||
        image.at<cv::Vec3b>(cvRound(hit->y), cvRound(hit->x)) != cv::Vec3b(0, 0, 255) ||
        image.at<cv::Vec3b>(cvRound(open->y), cvRound(open->x)) != cv::Vec3b(255, 0, 0))
    {
        std::cerr << "Target marker colors are incorrect\n";
        return 1;
    }
    return 0;
}