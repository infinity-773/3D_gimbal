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
    frame.target.valid = true;
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
    return 0;
}