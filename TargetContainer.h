#ifndef TARGET_CONTAINER_H_
#define TARGET_CONTAINER_H_

#include <vector>
#include <opencv2/core.hpp>

class BoxContainer
{
public:
	BoxContainer() = default;

	explicit BoxContainer(const std::vector<cv::Rect>& boxes)
		: boxes_(boxes)
	{}

	void setBoxes(const std::vector<cv::Rect>& boxes)
	{
		boxes_ = boxes;
	}

	std::size_t size() const
	{
		return boxes_.size();
	}

	cv::Point2d getCenter(std::size_t index) const
	{
		const cv::Rect& box = boxes_.at(index);
		return cv::Point2d(box.x + box.width * 0.5,
						   box.y + box.height * 0.5);
	}

private:
	std::vector<cv::Rect> boxes_;
};

#endif // TARGET_CONTAINER_H_