#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "CameraProjection.h"
#include "MathUtils.h"
#include "RingDetector.h"
#include "Serial.h"
#include "TargetMatcher.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

struct DashboardFrame
{
	std::vector<TargetPx> target;
	std::vector<TargetMarker> targetMarkers;
	cv::Point2d aimPoint;
	double aimTolerance;
	bool aimedStable;
	bool targetProjectionValid;
	bool feedbackReceived;
	bool feedbackValid;
	bool fireEnabled;
	bool modelAccepted;
	int shotsFired;
	std::size_t ringCount;
	AxisData angles;
	AxisData leadAngles;
	double residual;
	frc::Translation3d center;
	std::optional<frc::Translation3d> fittedCenter;
	double radius;
	double measuredOmega;
	double matchedOmega;
	bool omegaReady;
	frc::Translation3d predictedTarget;
	GimbalAngles gimbalAngles;
	TRoMaC::GimbalFeedback feedback;
	TRoMaC::VisionData command;
	TargetColor targetColor;
	std::chrono::steady_clock::time_point receivedAt;
};

class DashBoard
{
public:
	template <typename T>
	void setLogField(const std::string& name, const T& value)
	{
		std::ostringstream stream;
		stream << value;
		const auto field = std::find_if(fields_.begin(), fields_.end(),
			[&name](const auto& entry) { return entry.first == name; });
		if (field == fields_.end())
			fields_.emplace_back(name, stream.str());
		else
			field->second = stream.str();
	}

	cv::Mat renderLog() const
	{
		constexpr int lineHeight = 24;
		constexpr int margin = 12;
		int width = 400;
		for (const auto& field : fields_)
		{
			const std::string text = field.first + ": " + field.second;
			width = std::max(width, cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
												   0.5, 1, nullptr).width + 2 * margin);
		}
		cv::Mat log(std::max(80, static_cast<int>(fields_.size()) * lineHeight + 2 * margin),
					width, CV_8UC3, cv::Scalar(24, 24, 24));
		int y = margin + 16;
		for (const auto& field : fields_)
		{
			cv::putText(log, field.first + ": " + field.second, cv::Point(margin, y),
						cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(230, 230, 230), 1,
						cv::LINE_AA);
			y += lineHeight;
		}
		return log;
	}

	void updateLog(const DashboardFrame& frame)
	{
		if (hasPreviousFrame_)
		{
			const double dt = std::chrono::duration<double>(frame.receivedAt - previousFrame_).count();
			if (dt > 1e-6)
				fps_ = fps_ <= 0.0 ? 1.0 / dt : 0.9 * fps_ + 0.1 / dt;
		}
		previousFrame_ = frame.receivedAt;
		hasPreviousFrame_ = true;

		setLogField("target", std::any_of(frame.target.begin(), frame.target.end(),
			[](const TargetPx& item) { return item.valid; }) ? "detected" : "none");
		setLogField("target yaw / pitch", !frame.target.empty() && frame.target.front().valid
			? cv::format("%.2f / %.2f", frame.angles.yaw, frame.angles.pitch) : "n/a");
		setLogField("lead yaw / pitch", frame.targetProjectionValid
			? cv::format("%.2f / %.2f", frame.leadAngles.yaw, frame.leadAngles.pitch) : "n/a");
		setLogField("residual", frame.residual);
		setLogField("center (m)", cv::format("%.3f, %.3f, %.3f", frame.center.X().value(),
											frame.center.Y().value(), frame.center.Z().value()));
		setLogField("model accepted", frame.modelAccepted ? "yes" : "no");
		setLogField("omega measured", frame.omegaReady
			? cv::format("%.3f rad/s", frame.measuredOmega)
			: "fitting");
		setLogField("omega matched", cv::format("%.3f rad/s", frame.matchedOmega));
		setLogField("shots", frame.shotsFired);
		setLogField("fire", frame.fireEnabled ? "ON" : "OFF");
		setLogField("color", colorName(frame.targetColor));
		setLogField("feedback", frame.feedbackReceived
			? cv::format("yaw=%.2f pitch=%.2f%s", frame.feedback.yaw_angle,
						 frame.feedback.pitch_angle, frame.feedbackValid ? "" : " (stale)")
			: "not received");
		setLogField("friction command", cv::format("L=%u R=%u",
			static_cast<unsigned>(frame.command.left_speed), static_cast<unsigned>(frame.command.right_speed)));
		setLogField("friction feedback", frame.feedbackReceived
			? cv::format("L=%u R=%u%s", static_cast<unsigned>(frame.feedback.left_speed),
						 static_cast<unsigned>(frame.feedback.right_speed),
						 frame.feedbackValid ? "" : " (stale)") : "not received");
		setLogField("fps", cvRound(fps_));
		setLogField("rings", frame.ringCount);
	}

	void draw(cv::Mat& image, const DashboardFrame& frame,
			  const CameraProjection& projection) const
	{
		const cv::Point aim(cvRound(frame.aimPoint.x), cvRound(frame.aimPoint.y));
		cv::circle(image, aim, 6, cv::Scalar(0, 255, 0), 2);
		if (frame.feedbackValid)
		{
			if (frame.fittedCenter)
				drawTrajectory(image, *frame.fittedCenter, frame.radius, frame.gimbalAngles,
							   projection, cv::Scalar(255, 255, 255));
			if (frame.modelAccepted)
				drawTrajectory(image, frame.center, frame.radius, frame.gimbalAngles,
							   projection, cv::Scalar(0, 255, 0));
		}
		const bool hasTarget = std::any_of(frame.target.begin(), frame.target.end(),
			[](const TargetPx& item) { return item.valid; });
		if (hasTarget)
		{
			for (const TargetPx& item : frame.target)
			{
				if (!item.valid)
					continue;
				const cv::Point target(cvRound(item.center.x), cvRound(item.center.y));
				cv::circle(image, target, cvRound(item.radius),
					frame.aimedStable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
				cv::line(image, target, aim, cv::Scalar(0, 255, 255), 1);
			}
			if (frame.targetProjectionValid)
			{
				const auto predicted = projection.worldToPixel(frame.predictedTarget, frame.gimbalAngles);
				if (predicted)
				{
					const cv::Point point(cvRound(predicted->x), cvRound(predicted->y));
					cv::drawMarker(image, point, cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 20, 2);
					cv::putText(image, "PRED", point + cv::Point(14, -12),
								cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 0, 255), 1);
				}
			}
		}
		if (showDebug_)
		{
			drawAim(image, aim, frame.aimTolerance);
			drawLegend(image);
		}
		if (frame.feedbackValid)
		{
			for (const TargetMarker& marker : frame.targetMarkers)
			{
				const auto pixel = projection.worldToPixel(marker.position, frame.gimbalAngles);
				if (!pixel)
					continue;
				const cv::Point point(cvRound(pixel->x), cvRound(pixel->y));
				const cv::Scalar stateColor = marker.active
					? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
				if (marker.lastFired)
					cv::circle(image, point, 18, cv::Scalar(0, 165, 255), 2);
				cv::drawMarker(image, point, stateColor, cv::MARKER_CROSS, 24, 2);
				cv::putText(image, marker.active ? "HIT" : "OPEN", point + cv::Point(12, -10),
					cv::FONT_HERSHEY_SIMPLEX, 0.38, stateColor, 1);
				if (marker.lastFired)
					cv::putText(image, "LAST FIRED", point + cv::Point(12, 18),
						cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(0, 165, 255), 1);
			}
		}
	}

	void show(const cv::Mat& image, const cv::Mat& mask) const
	{
		cv::namedWindow("Camera", cv::WINDOW_NORMAL);
		cv::imshow("Camera", image);
		cv::imshow("Log", renderLog());
		if (showDebug_)
			cv::imshow("Mask", mask);
	}

	bool handleKey(int key, const cv::Mat& image, const cv::Mat& mask,
				   bool& fireEnabled, TargetColor& targetColor)
	{
		if (key == 'q' || key == 'Q' || key == 27)
			return false;
		if (key == 's' || key == 'S')
			saveImages(image, mask);
		else if (key == 'd' || key == 'D')
		{
			showDebug_ = !showDebug_;
			if (!showDebug_)
				cv::destroyWindow("Mask");
		}
		else if (key == 'f' || key == 'F')
		{
			fireEnabled = !fireEnabled;
			std::cout << "开火开关: " << (fireEnabled ? "ON" : "OFF") << '\n';
		}
		else if (key == 'c' || key == 'C')
		{
			targetColor = targetColor == TargetColor::Red ? TargetColor::Blue
				: targetColor == TargetColor::Blue ? TargetColor::Any : TargetColor::Red;
			std::cout << "目标颜色: " << colorName(targetColor) << '\n';
		}
		return true;
	}

	bool processKeys(const cv::Mat& image, const cv::Mat& mask,
					 bool& fireEnabled, TargetColor& targetColor)
	{
		return handleKey(cv::waitKey(1), image, mask, fireEnabled, targetColor);
	}

private:
	static void drawTrajectory(cv::Mat& image, const frc::Translation3d& center,
							   double radius, const GimbalAngles& angles,
							   const CameraProjection& projection, const cv::Scalar& color)
	{
		const frc::Translation3d edge{center.X().value(), center.Y().value() + radius,
									 center.Z().value()};
		const auto centerPixel = projection.worldToPixel(center, angles);
		const auto edgePixel = projection.worldToPixel(edge, angles);
		if (centerPixel && edgePixel)
			cv::circle(image, *centerPixel, cvRound(cv::norm(*edgePixel - *centerPixel)),
					   color, 2);
	}

	static const char* colorName(TargetColor color)
	{
		switch (color)
		{
			case TargetColor::Red: return "red";
			case TargetColor::Blue: return "blue";
			case TargetColor::Any: return "any";
		}
		return "unknown";
	}

	static void drawAim(cv::Mat& image, const cv::Point& aim, double tolerance)
	{
		cv::drawMarker(image, aim, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 1);
		cv::circle(image, aim, cvRound(tolerance), cv::Scalar(0, 255, 0), 1);
		cv::putText(image, "AIM", aim + cv::Point(14, -12), cv::FONT_HERSHEY_SIMPLEX,
					0.45, cv::Scalar(0, 255, 0), 1);
	}

	static void drawLegend(cv::Mat& image)
	{
		if (image.cols < 226 || image.rows < 126)
			return;
		const int x = image.cols - 216;
		const int y = image.rows - 116;
		cv::rectangle(image, cv::Rect(x - 10, y - 10, 216, 116), cv::Scalar(0, 0, 0), cv::FILLED);
		cv::circle(image, cv::Point(x + 4, y + 6), 4, cv::Scalar(0, 255, 0), -1);
		cv::putText(image, "AIM   impact point", cv::Point(x + 16, y + 11),
					cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 255, 0), 1);
		cv::circle(image, cv::Point(x + 4, y + 30), 4, cv::Scalar(0, 0, 255), -1);
		cv::putText(image, "TGT   detected ring", cv::Point(x + 16, y + 35),
					cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 0, 255), 1);
		cv::putText(image, "yellow line = error", cv::Point(x + 4, y + 60),
					cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(200, 200, 200), 1);
		cv::drawMarker(image, cv::Point(x + 4, y + 82), cv::Scalar(255, 0, 0),
			cv::MARKER_CROSS, 10, 2);
		cv::putText(image, "OPEN     ", cv::Point(x + 16, y + 87),
			cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(255, 0, 0), 1);
		cv::drawMarker(image, cv::Point(x + 82, y + 82), cv::Scalar(0, 0, 255),
			cv::MARKER_CROSS, 10, 2);
		cv::putText(image, "HIT", cv::Point(x + 94, y + 87),
			cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(0, 0, 255), 1);
		cv::circle(image, cv::Point(x + 4, y + 101), 6,
			cv::Scalar(0, 165, 255), 2);
		cv::putText(image, "LAST FIRED", cv::Point(x + 16, y + 106),
			cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(0, 165, 255), 1);
	}

	void saveImages(const cv::Mat& image, const cv::Mat& mask)
	{
		const std::string name = "cap_" + std::to_string(frameIndex_++);
		const bool savedImage = cv::imwrite(name + ".png", image);
		const bool savedMask = cv::imwrite(name + "_mask.png", mask);
		std::cout << (savedImage && savedMask ? "已保存 " : "保存失败 ") << name << '\n';
	}

	bool showDebug_ = true;
	int frameIndex_ = 0;
	bool hasPreviousFrame_ = false;
	double fps_ = 0.0;
	std::chrono::steady_clock::time_point previousFrame_;
	std::vector<std::pair<std::string, std::string>> fields_;
};

#endif // DASHBOARD_H