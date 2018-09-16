#include "ArmorDetector.h"

using cv::Mat;
using cv::Point;
using cv::Point2f;
using cv::RotatedRect;
using cv::Size;
using cv::Size2f;
using std::vector;

ArmorDetector::ArmorDetector()
{
	// cv::FileStorage setting("../setting.xml",cv::FileStorage::READ);
	// if(!setting.isOpened()) 
	//     throw std::runtime_error("No Setting files!");
	// setting["max_vertical_angle_for_armor"] >> max_vertical_angle_for_armor;
	// setting["max_vertical_angle_for_light"] >> max_vertical_angle_for_light;
	// setting["threshould"] >> threshould;
	// setting["max_rate_for_armor"] >> max_rate_for_armor;
	// setting["min_rate_for_armor"] >> min_rate_for_armor;
	// setting["diff_value"] >> diff_value;

	enemy_is_red = true;
	too_far = false;
	loss_count = 0;
	lastboard_rect = cv::Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
	cv::Mat template_jpg;
	template_jpg = cv::imread("");
}

/**
 * @brief ArmorDetector::adjustRRect adjust the rect position
 * @param rect input rect
 * @return if rect width<height return the input_rect else return the input_rect rotating 90 degrees
 */
//保证下面得到的rRect中，height是确定大于width的
RotatedRect ArmorDetector::adjustRRect(const RotatedRect &rect)
{
	const Size2f &s = rect.size;
	if (s.width < s.height)
		return rect;
	return RotatedRect(rect.center, Size2f(s.height, s.width), rect.angle + 90.0);
}

/**
 * @brief ArmorDetector::boundingRRect
 * @param left the left light-lamp
 * @param right the right light-lamp
 * @return the rotaterect between the left-rect and the right-rect which is the area we should shoot to.
 */
RotatedRect ArmorDetector::boundingRRect(const RotatedRect &left, const RotatedRect &right)
{
	const Point &pl = left.center, &pr = right.center;
	Point2f center = (pl + pr) / 2.0;
	Size2f wh_l = left.size;
	Size2f wh_r = right.size;
	//这里修改了宽度，为了使width能够完全包含到两边的灯条
	double width = POINT_DIST(pl, pr) + (wh_l.width + wh_r.width) / 2.0;
	double height = std::max(wh_l.height, wh_r.height);
	//float height = (wh_l.height + wh_r.height) / 2.0;
	double angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

/**
 * @breif 将特定区域转化为水平的ROI区域
 * @param 输入的点按顺序排列为 左上，右上，左下，右下
 * @param 输出尺寸为size 为了和装甲的灯条匹配，使用了100 * 43
 */
void ArmorDetector::perspectiveTransformation(Mat src, Mat &dst, Point2f *src_vertices, Size size)
{
	//目标
	Point2f dst_vertices[4];
	dst_vertices[0] = Point(0, 0);
	dst_vertices[1] = Point(105, 0);
	dst_vertices[2] = Point(0, 44);
	dst_vertices[3] = Point(105, 44);

	dst = Mat(Size(105, 44), src.type());
	Mat warpMatrix = getPerspectiveTransform(src_vertices, dst_vertices);
	warpPerspective(src, dst, warpMatrix, dst.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	resize(dst, dst, size);
}

void ArmorDetector::rRect_resize(cv::RotatedRect &rRect, double width_rate, double height_rate)
{
	auto center = rRect.center;
	center.x *= width_rate;
	center.y *= height_rate;
	auto angle = rRect.angle;
	rRect = cv::RotatedRect(center, cv::Size2f(rRect.size.width / 2, rRect.size.height / 2), angle);
}

// 更新下一帧应该处理的装甲位置
void ArmorDetector::updatePosInfo(const ArmorPos &armor)
{
	
	if (!too_far)
	{
		cv::Rect rec = armor.rRect.boundingRect();
		rec += rec.size();
		Point p = cv::Point(rec.width * 0.25, rec.height * 0.25);
		rec -= p;
		if (rec.x < 0)
			rec.x = 0;
		if (rec.y < 0)
			rec.y = 0;
		if (rec.x + rec.width > IMAGE_WIDTH)
			rec.x = IMAGE_WIDTH - rec.width;
		if (rec.y + rec.height > IMAGE_HEIGHT)
			rec.y = IMAGE_HEIGHT - rec.height;
		this->lastboard_rect = rec;
	}
	else
	{
		// we use 1*4 size of src
		Point2f center;
		for (auto p : armor.Pos)
			center += p;
		center /= 4; // center of the armor

		// Pos[0] Pos[1] for height (y) Pos[1].y - Pos[0].y
		// Pos[0] Pos[2] for width  (x) abs(Pos[0].x - Pos[2].x)
		/**
		  [0]0000000000[2]
		  ***0000000000***
		  [1]0000000000[3]
		 **/
		double pos_width = abs(armor.Pos[0].x - armor.Pos[2].x);
		if(pos_width < 0.0) pos_width = 0.0;
		double pos_height = armor.Pos[1].y - armor.Pos[0].y;
		if(pos_height < 0.0) pos_height = 0.0;
		cv::Rect rec(center - Point2f(pos_width, pos_height), center + Point2f(pos_width, pos_height));
		if (rec.x < 0)
			rec.x = 0;
		if (rec.y < 0)
			rec.y = 0;
		if (rec.x + rec.width > IMAGE_WIDTH)
			rec.x = IMAGE_WIDTH - rec.width - 1;
		if (rec.y + rec.height > IMAGE_HEIGHT)
			rec.y = IMAGE_HEIGHT - rec.height - 1;
		this->lastboard_rect = rec;
	}
}

cv::Mat ArmorDetector::hsvReg(const cv::Mat &srcImage, bool is_red)
{
	cv::Mat hsvImg;
	cv::cvtColor(srcImage, hsvImg, CV_BGR2HSV);
	cv::Mat threshouldImg;
	if (is_red)
	{
		cv::Mat threshouldImg0, threshouldImg1;
		cv::inRange(hsvImg, cv::Scalar(0, 0, 0), cv::Scalar(30, 255, 255), threshouldImg0);
		cv::inRange(hsvImg, cv::Scalar(150, 0, 0), cv::Scalar(180, 255, 255), threshouldImg1);
		cv::bitwise_or(threshouldImg0, threshouldImg1, threshouldImg);
	}
	else
	{
		cv::inRange(hsvImg, cv::Scalar(90, 0, 10), cv::Scalar(150, 255, 255), threshouldImg);
	}

	return threshouldImg;
}

//更改了原来的代码，先按照Y进行划分，更为准确 下面返回的角点改为 左上 右上 左下 右下
/**
 * @brief 以左上为起点，走Z形画出来的
 * @param 存储顺序的容器
 * 
 */
void ArmorDetector::getTarget2dPoinstion(const RotatedRect &rect, vector<Point> &target2d)
{
	//矩形rect的四个顶点存储在vertices里
	Point2f vertices[4];
	rect.points(vertices);
	Point2f lu, ld, ru, rd;
	//结果是对vertices这个数组按照y从小到大进行排列的
	std::sort(vertices, vertices + 4, [](const Point2f &p1, const Point2f &p2) -> bool { return p1.y < p2.y; });
	if (vertices[0].x < vertices[1].x)
	{
		lu = vertices[0];
		ru = vertices[1];
	}
	else
	{
		lu = vertices[1];
		ru = vertices[0];
	}
	if (vertices[2].x < vertices[3].x)
	{
		ld = vertices[2];
		rd = vertices[3];
	}
	else
	{
		ld = vertices[3];
		rd = vertices[2];
	}
	target2d.clear();
	target2d.push_back(lu);
	target2d.push_back(ru);
	target2d.push_back(ld);
	target2d.push_back(rd);
}

/**
 * @brief 保证四个点按照Z顺序走下来，以便于进行透射
 * @param 四边形的四个顶点
 */
bool ArmorDetector::makeBoxSafe(Point2f *src_vertices)
{
	//理论上的Z
	Point2f lu = src_vertices[0];
	Point2f ru = src_vertices[1];
	Point2f ld = src_vertices[2];
	Point2f rd = src_vertices[3];
	//判断这个z是否符合实际的Z
	bool _lu = lu.x < ru.x && lu.x < rd.x && lu.y < ld.y && lu.y < rd.y;
	bool _ru = ru.x > lu.x && ru.x > ld.x && ru.y < ld.y && ru.y < rd.y;
	bool _ld = ld.x < ru.x && ld.x < rd.x && ld.y > ru.y && ld.y > lu.y;
	bool _rd = rd.x > lu.x && rd.x > ld.x && rd.y > lu.y && rd.y > ru.y;

	if (_lu && _ru && _ld && _rd)
		return true;
	else
		return false;
}

/**
 * @brief 测量图像的相似性，返回Scalar类型，相似的越高
 * @param 检测目标和模板图
 * 
 */
cv::Scalar_<double> ArmorDetector::getMSSIM(Mat &i1)
{
	const double C1 = 6.5025, C2 = 58.5225;
	/***************************** INITS **********************************/
	int d = CV_32F;

	Mat I1, I2;
	i1.convertTo(I1, d); // cannot calculate on one byte large values
	enemy_is_red ?_binary_small_template.convertTo(I2, d) : _binary_large_template.convertTo(I2, d);
	

	Mat I2_2 = I2.mul(I2);  // I2^2
	Mat I1_2 = I1.mul(I1);  // I1^2
	Mat I1_I2 = I1.mul(I2); // I1 * I2

	Mat mu1, mu2; // PRELIMINARY COMPUTING
	GaussianBlur(I1, mu1, Size(11, 11), 1.5);
	GaussianBlur(I2, mu2, Size(11, 11), 1.5);

	Mat mu1_2 = mu1.mul(mu1);
	Mat mu2_2 = mu2.mul(mu2);
	Mat mu1_mu2 = mu1.mul(mu2);

	Mat sigma1_2, sigma2_2, sigma12;

	cv::GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
	sigma1_2 -= mu1_2;

	cv::GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
	sigma2_2 -= mu2_2;

	cv::GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
	sigma12 -= mu1_mu2;

	///////////////////////////////// FORMULA ////////////////////////////////
	Mat t1, t2, t3;

	t1 = 2 * mu1_mu2 + C1;
	t2 = 2 * sigma12 + C2;
	t3 = t1.mul(t2); // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

	t1 = mu1_2 + mu2_2 + C1;
	t2 = sigma1_2 + sigma2_2 + C2;
	t1 = t1.mul(t2); // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

	Mat ssim_map;
	divide(t3, t1, ssim_map); // ssim_map =  t3./t1;

	cv::Scalar_<double> mssim = mean(ssim_map); // mssim = average of ssim map
	return mssim;
}

vector<cv::Point2f> chooseTarget(Mat &srcImage, ArmorDetector &detector)
{

	if (!srcImage.data)
	{
		// throw std::runtime_error("No Image Data With func: chooseTarget(Mat &srcImage)!");
		std::cout << "NO DATA!" << std::endl;
		return vector<cv::Point2f>();
	}

	cv::cvtColor(srcImage(detector.lastboard_rect), detector.grayImage, CV_BGR2GRAY);
	detector.grayImage = detector.grayImage > 20;
	// cv::Mat regImg = detector.hsvReg(srcImage(detector.lastboard_rect), detector.enemy_is_red);
	// cv::bitwise_and(regImg, detector.grayImage, regImg);

	vector<ArmorPos> armors_pos;
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	vector<vector<cv::Point>> final_contours;

	vector<Point2f> board_position;

	/**
 * 此处是为了判断因为目标过小而导致的无法识别，其实主要是识别的时候目标太小过不了第一轮筛选
 * 所以无法进行第二轮的判断。
 * 因为过小的时候轮廓几乎变成了圆的形状，而且轮廓也变得没有规则，
 * 所以要进行一些特征放大的操作。为了符合灯条的检测，最好使用椭圆的操作
 **/
	// if (detector.lastboard_rect.area() < FARSIZE)
	// {
	// 	detector.too_far = true;
	// 	// dilate(regImg, regImg, kernel);
	// 	erode(regImg, regImg, detector.kernel);
	// 	resize(regImg, regImg, cv::Size(), 1.0, 4.0);

	// 	regImg = regImg > 1;
	// }
    cv::Mat kernel_blue = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 7));
	
	// if(detector.enemy_is_red == false)
	// {
	// 	cv::erode(regImg, regImg, kernel_blue);
	// 	cv::dilate(regImg, regImg, kernel_blue);
	// }

	cv::findContours(detector.grayImage, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// 第一轮筛选
	for (int i = 0; i < contours.size() && contours.size() > 2; ++i)
	{
		//if contour.size<5, eillpse cannot be used
		if (contours[i].size() < 5)
			continue;

		//然后根据轮廓的角度，筛掉那些不满足垂直方向角度偏移
		RotatedRect minRect = fitEllipse(contours[i]);
		minRect = detector.adjustRRect(minRect);

		auto angle = minRect.angle;
		angle = 90 - angle;
		angle = angle < 0 ? angle + 180 : angle;
		if (std::abs(angle - 90) > 20)
			continue;
		final_contours.push_back(contours[i]);
	}

	if (contours.size() == 2)
		final_contours.push_back(contours[0]), final_contours.push_back(contours[1]);

	// 目前得到的contours是比较符合灯条形状的final_contours,接下来就是分别连接这些contour，判断形成的准装甲板是否符合一定条件
	for (int i = 0; i < final_contours.size(); ++i)
	{
		RotatedRect rRect1 = cv::minAreaRect(final_contours[i]);
		rRect1 = detector.adjustRRect(rRect1);

		vector<cv::Point> pos_1;
		detector.getTarget2dPoinstion(rRect1, pos_1);

		Point2f mid_u1, mid_d1;
		if (!detector.too_far)
		{
			mid_u1 = (pos_1[0] + pos_1[1]) / 2 + detector.lastboard_rect.tl();
			mid_d1 = (pos_1[2] + pos_1[3]) / 2 + detector.lastboard_rect.tl();
		}
		else
		{
			mid_u1 = Point(pos_1[0].x + pos_1[1].x, (pos_1[0].y + pos_1[1].y) / 4) / 2 + detector.lastboard_rect.tl();
			mid_d1 = Point(pos_1[2].x + pos_1[3].x, (pos_1[2].y + pos_1[3].y) / 4) / 2 + detector.lastboard_rect.tl();
		}

		auto angle_1 = atan2(mid_d1.y - mid_u1.y, mid_d1.x - mid_u1.x) * 180 / CV_PI;

		for (int j = i + 1; j < final_contours.size(); ++j)
		{

			RotatedRect rRect2 = cv::minAreaRect(final_contours[j]);
			vector<cv::Point> pos_2;
			rRect2 = detector.adjustRRect(rRect2);
			detector.getTarget2dPoinstion(rRect2, pos_2);
			Point2f mid_u2, mid_d2;
			if (!detector.too_far)
			{
				mid_u2 = (pos_2[0] + pos_2[1]) / 2 + detector.lastboard_rect.tl();
				mid_d2 = (pos_2[2] + pos_2[3]) / 2 + detector.lastboard_rect.tl();
			}
			else
			{
				mid_u2 = Point(pos_2[0].x + pos_2[1].x, pos_2[0].y / 4 + pos_2[1].y / 4) / 2 + detector.lastboard_rect.tl();
				mid_d2 = Point(pos_2[2].x + pos_2[3].x, pos_2[2].y / 4 + pos_2[3].y / 4) / 2 + detector.lastboard_rect.tl();
			}
			auto angle_2 = atan2(mid_d2.y - mid_u2.y, mid_d2.x - mid_u2.x) * 180 / CV_PI;
			if (std::abs(angle_1 - angle_2) > 2.0)
				continue;

			// current board
			RotatedRect board = detector.boundingRRect(rRect1, rRect2);
			if (detector.too_far)
				detector.rRect_resize(board, 1.0, 0.25);
			board.center += Point2f(detector.lastboard_rect.tl());
			vector<cv::Point> pos_board;
			detector.getTarget2dPoinstion(board, pos_board);

			// 角度筛选
			double angle = 0;
			if (pos_2[1].x != pos_1[2].x)
				angle = std::atan2(pos_1[2].y - pos_2[1].y, pos_2[1].x - pos_1[2].x);
			if (angle > 5)
				continue;

			//长宽筛选
			auto width = POINT_DIST(pos_board[0], pos_board[1]);
			auto height = POINT_DIST(pos_board[0], pos_board[2]);
			auto rate = width / height;
			// cout << rate << endl;
			if (rate > 3.5 || rate < 0.5)
				continue;

			Mat dst;
			Point2f roi[4] = {pos_board[0], pos_board[1], pos_board[2], pos_board[3]};
			if (!detector.makeBoxSafe(roi))
				continue;
			// TODO
			detector.perspectiveTransformation(srcImage, dst, roi, Size(105, 44));

			// BGR for the default, choose channel 2
			double dist = 0;
			dist = (detector.enemy_is_red ? (detector.getMSSIM(dst))[2] : (detector.getMSSIM(dst))[0]);

			board_position = {mid_u1, mid_d1, mid_u2, mid_d2};

			//各自灯条的垂直间距不能太小
			if(mid_d1.y - mid_u1.y < 4 || mid_d2.y - mid_u2.y < 4) continue;
			// // 两个灯条的垂直间距不能太大，水平间距不能太小
			if(std::abs(mid_d1.y - mid_d2.y) > 13 || std::abs(mid_d1.x - mid_d2.x) < 30) continue;
			// //两个灯条的长度之比
			double rate_for_light = (mid_d1.y - mid_u1.y) / (mid_d2.y - mid_u2.y);
			if(rate_for_light > 1.4 || rate_for_light < 0.6) continue;
			//height and weight detect for x

			// for run_time value: this value ma be wrong, could not as mainly factor
			if (dist > 0.15)
			{
				armors_pos.emplace_back(dist, board, board_position);
			}
		}
	}
	// use sort to find the best position
	sort(armors_pos.begin(), armors_pos.end(), [](ArmorPos a, ArmorPos b) { return a.diff > b.diff; });

	if (armors_pos.size() != 0)
	{
		ArmorPos final_board = armors_pos[0];
		// if (detector.makeBoxSafe(p))
		// {
		detector.updatePosInfo(final_board);
		for (int t = 0; t < 4; ++t)
		{
			cv::line(srcImage, final_board.Pos[t], final_board.Pos[(t + 1) % 4], cv::Scalar(0, 255, 0));
		}
		detector.loss_count = 0;
		cv::rectangle(srcImage, detector.lastboard_rect, cv::Scalar(0, 0, 255));
		for(auto i : final_board.Pos) i *= 2;
		return final_board.Pos;
		// }
	}
	else
	{
		// wait for 5 fps for normal armor
		if (detector.loss_count < 20)
		{
			++detector.loss_count;
		}
		// quit waiting
		else
		{
			detector.loss_count = 0;
			detector.lastboard_rect = cv::Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
		}
	}
	return vector<cv::Point2f>();
}