#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;
using namespace std;

// Mat base_img;
// Mat binary_img;
// Mat perspect_img;

int mid_point;
int left_start_index;
int right_start_index;
int nwindows = 12;
int w = 800;
int h = 500;
int prev_left_index = 0;
int prev_right_index = 0;
Mat inverse_matrix;
Point2f src_pts[4], dst_pts[4];
std::vector<Point> mpoints(nwindows);
std::vector<Point> lpoints(nwindows);
std::vector<Point> rpoints(nwindows);
std::vector<Point> mmpoints(nwindows);

Mat Perspective(Mat& src) {
	


	src_pts[0] = Point2f(160, 467);
	src_pts[1] = Point2f(640, 467);
	src_pts[2] = Point2f(770, 590);
	src_pts[3] = Point2f(30, 590);

	// src_pts[0] = Point2f(282, 467);
	// src_pts[1] = Point2f(512, 467);
	// src_pts[2] = Point2f(720, 590);
	// src_pts[3] = Point2f(94, 590);

	// dst_pts[0] = Point2f(332, 337);
	// dst_pts[1] = Point2f(462, 337);
	// dst_pts[2] = Point2f(700, 590);
	// dst_pts[3] = Point2f(114, 590);
	dst_pts[0] = Point2f(0, 0);
	dst_pts[1] = Point2f(w-1, 0);
	dst_pts[2] = Point2f(w-1, h-1);
	dst_pts[3] = Point2f(0, h-1);

	Mat pers = getPerspectiveTransform(src_pts, dst_pts);
	Mat inv_pers = getPerspectiveTransform(dst_pts, src_pts);
	inverse_matrix = inv_pers;
	Mat dst;

	warpPerspective(src, dst, pers, Size(w, h));

	return dst;

}



Mat Houghtransform(Mat& image){

	Mat lineResult;
	cvtColor(image, lineResult, COLOR_GRAY2BGR);
    vector<Vec4i> lines;
	HoughLinesP(image, lines, 1, CV_PI/180, 100, 50, 250);
	for(size_t i = 0; i < lines.size(); i++){
		Vec4i l = lines[i];
		line(lineResult, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, LINE_AA);
	}

	return lineResult;
}

Mat RGB2Gray(Mat& image){
    Mat gray_img;
    cvtColor(image, gray_img, COLOR_BGR2GRAY);
    return gray_img;
}


Mat GRAY2RGB(Mat& image){
    Mat color_img;
    cvtColor(image, color_img, COLOR_GRAY2BGR);
    return color_img;
}

Mat GaussianFilter(Mat& image, int kernel_size = 10, double sigma = 1.5) {
    
	Mat kernel = getGaussianKernel(kernel_size, sigma);
    Mat filtered_img;
    filter2D(image, filtered_img, -1, kernel);
	Sobel(filtered_img, filtered_img, -1, 1, 0);
    return filtered_img;

}

Mat Contrast(Mat& image){

	Mat output_img;
	float alpha = 1.f;
	output_img = image + (image - mean(image)) * alpha;


	return output_img;	 

}


Mat Binarize(Mat& image){
   
	Mat output_img;
    int kernel_size = 5;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(kernel_size, kernel_size));
  
	adaptiveThreshold(image, output_img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 121, -20);
    morphologyEx(output_img, output_img, MORPH_ELLIPSE, Mat(),Point(-1,-1), 3);
	dilate(output_img, output_img, kernel);
	morphologyEx(output_img, output_img, MORPH_CLOSE, Mat(),Point(-1,-1), 5);
   	return output_img;
}


Mat CannyEdge(Mat& source)
{
    Mat dst;
    Canny(source, dst, 80, 120);
	morphologyEx(dst, dst, MORPH_CLOSE, Mat(), Point(-1, -1), 1);


    return dst;
}
Mat RegionOfInterest(Mat& source)
{
	// float trapezoidBottomWidth = 1.0; // Width of bottom edge of trapezoid, expressed as percentage of image width
    // float trapezoidTopWidth = 0.08; // Above comment also applies here, but then for the top edge of trapezoid
    // float trapezoidHeight = 0.45; // Height of the trapezoid expressed as percentage of image height
	
	// float smallBottomWidth = 0.45; // This will be added to trapezoidBottomWidth to create a less wide bottom edge
    // float smallTopWidth = 0.01; // We multiply the percentage trapoezoidTopWidth with this parameter to create a less wide top edge
    // float smallHeight = 1.0; // Height of the small trapezoid expressed as percentage of height of big trapezoid
	float trapezoidBottomWidth = 1.0; // Width of bottom edge of trapezoid, expressed as percentage of image width
    float trapezoidTopWidth = 1.0; // Above comment also applies here, but then for the top edge of trapezoid
    float trapezoidHeight = 1.0; // Height of the trapezoid expressed as percentage of image height
	
	float smallBottomWidth = 0.5; // This will be added to trapezoidBottomWidth to create a less wide bottom edge
    float smallTopWidth = 0.3; // We multiply the percentage trapoezoidTopWidth with this parameter to create a less wide top edge
    float smallHeight = 0.9; // Height of the small trapezoid expressed as percentage of height of big trapezoid

	float bar = 1;


	vector<vector<Point>> pts = { {Point((source.cols * (1 - trapezoidBottomWidth)) / 2, source.rows * bar), 
								Point((source.cols * (1 - trapezoidTopWidth)) / 2, source.rows - source.rows * trapezoidHeight),
								Point(source.cols - (source.cols * (1 - trapezoidTopWidth) ) / 2, source.rows - source.rows * trapezoidHeight),
								Point(source.cols - (source.cols * (1 - trapezoidBottomWidth)) / 2, source.rows * bar)} };

	pts[0].push_back(Point((source.cols * (1 - trapezoidBottomWidth + smallBottomWidth)) / 2, source.rows));
	pts[0].push_back(Point((source.cols * (1 - trapezoidTopWidth * smallTopWidth)) / 2, source.rows - source.rows * trapezoidHeight * smallHeight));
	pts[0].push_back(Point(source.cols - (source.cols * (1 - trapezoidTopWidth * smallTopWidth)) / 2, source.rows - source.rows * trapezoidHeight * smallHeight));
	pts[0].push_back(Point(source.cols - (source.cols * (1 - trapezoidBottomWidth + smallBottomWidth)) / 2, source.rows));
	
	// vector<vector<Point>> pts = { {Point(134, 590), Point(680, 590), Point(442, 337), Point(352, 337)} };

    Mat mask = Mat::zeros(source.size(), source.type());

    fillPoly(mask, pts, Scalar(255,255,255), LINE_AA);
	// return mask;
    Mat maskedImage;
    bitwise_and(source, mask, maskedImage);

    return maskedImage;
}

Mat GetHistImage(const Mat& img){

	// Calculate histogram by summing pixel values for each column
    int image_width = img.cols;
    int image_height = img.rows;
    vector<int> histogram(image_width, 0);

    for (int col = 0; col < image_width; col++) {
        int sum = 0;
        for (int row = 0; row < image_height; row++) {
            sum += img.at<uchar>(row, col);
        }
        histogram[col] = sum;
    }
	

    // Find the maximum values on the left and right sides of the histogram
    int center = image_width / 2;
    int max_left = 0, max_right = 0;
    int max_left_index = 0, max_right_index = 0;

	// cout << "-----------" << endl;
    for (int i = 0; i < center; i++) {
        if (histogram[i] > max_left) {
            max_left = histogram[i];
			// cout << "MAX_LEFT : " << max_left << endl;
			max_left_index = i;
			prev_left_index = max_left_index;
		}
    }
	if(max_left == 0){
		max_left_index = prev_left_index;
		// cout << "왼쪽 차선 없음" << endl;
	}

    for (int i = center; i < image_width; i++) {
        if (histogram[i] > max_right) {
            max_right = histogram[i];
			// cout << "MAX_right : " << max_right << endl;
			max_right_index = i;
			prev_right_index = max_right_index;
		}
    }
	if(max_right == 0){
			max_right_index = prev_right_index;
			// cout << "오른쪽 차선 없음" << endl;
	}
	left_start_index = max_left_index;
	right_start_index = max_right_index;



    // Normalize the histogram
  	int max_count = max(max_left, max_right);	
    int hist_height = 400;
    Mat hist_image(hist_height, image_width, CV_8UC3, Scalar(0, 0, 0));

    for (int col = 0; col < image_width - 1; col++) {
        int x1 = col;
        int y1 = hist_height - cvRound(hist_height * (static_cast<double>(histogram[col]) / max_count));
        int x2 = col + 1;
        int y2 = hist_height - cvRound(hist_height * (static_cast<double>(histogram[col + 1]) / max_count));

		if (col == max_left_index || col == max_right_index) {
            line(hist_image, Point(x1, y1), Point(x2, y2), Scalar(0, 0, 255), 3, LINE_AA);
        } else {
            line(hist_image, Point(x1, y1), Point(x2, y2), Scalar(255, 255, 255), 3, LINE_AA);
        }

        line(hist_image, Point(x1, y1), Point(x2, y2), Scalar(255, 255, 255), 2, LINE_AA);
    }
	return hist_image;
}

Mat SlidingWindow(Mat& binarized_image){
    
    // // 이미지의 중앙을 계산합니다.
    // int center_x = binarized_image.cols / 2;
    // // y축으로 이동할 스텝 사이즈를 정의합니다.
    // int step_y = 80;
	Mat color_img;
	cvtColor(binarized_image, color_img, COLOR_GRAY2BGR);
    // 윈도우의 너비와 높이를 정의합니다.
    int window_height = (int)(h / nwindows);
    int window_width = (int)(w / nwindows * 1.2);
    int margin = window_width / 2;

    // init value setting

    // 왼쪽과 오른쪽에서 시작 위치를 찾습니다.
    int left_start = left_start_index;
    int right_start = right_start_index;

	int lane_mid = w / 2;

	int win_y_high = h - window_height;
	int win_y_low = h;

	int win_x_leftb_right = left_start + margin;
	int win_x_leftb_left = left_start - margin;

	int win_x_rightb_right = right_start + margin;
	int win_x_rightb_left = right_start - margin;

	lpoints[0] = Point(left_start, (int)((win_y_high + win_y_low) / 2));
	rpoints[0] = Point(right_start, (int)((win_y_high + win_y_low) / 2));
	mpoints[0] = Point((int)((left_start + right_start) / 2), (int)((win_y_high + win_y_low) / 2));

    rectangle(binarized_image, Rect(win_x_leftb_left, win_y_high, window_width, window_height), Scalar(255, 255, 255), 1);
	rectangle(binarized_image, Rect(win_x_rightb_left, win_y_high, window_width, window_height), Scalar(255, 255, 255), 1);

    for (int window = 1; window < nwindows; window++) {

		win_y_high = h - (window + 1) * window_height;
		win_y_low = h - window * window_height;

		win_x_leftb_right = left_start + margin;
		win_x_leftb_left = left_start - margin;

		win_x_rightb_right = right_start + margin;
		win_x_rightb_left = right_start - margin;

		int offset = (int)((win_y_high + win_y_low) / 2);	

		int pixel_thres = window_width * 0.05;

		int ll = 0, lr = 0; int rl = 800, rr = 800;
		int li = 0; // nonzero가 몇개인지 파악하기 위한 벡터에 사용될 인자
		// window의 위치를 고려해서 벡터에 집어넣으면 불필요한 부분이 많아질 수 있다. 어차피 0의 개수를 구하기 위한 벡터이므로 0부터 window_width+1 개수만큼 생성	
		std::vector<int> lhigh_vector(window_width + 1);  // nonzero가 몇개 인지 파악할 때 사용할 벡터
		for (auto x = win_x_leftb_left; x < win_x_leftb_right; x++) {
			li++;
			lhigh_vector[li] = binarized_image.at<uchar>(offset, x); // li번째 벡터에 다음에 해당하는 픽셀 값을 넣는다.

			// 차선의 중앙을 계산하기 위해 255 시작점과 255 끝점을 계산
			if (binarized_image.at<uchar>(offset, x) == 255 && ll == 0) {
				ll = x;
				lr = x;
			}
			if (binarized_image.at<uchar>(offset, x) == 255 && lr != 0) {
				lr = x;
			}
		}
		int ri = 0;
		std::vector<int> rhigh_vector(window_width + 1);
		for (auto x = win_x_rightb_left; x < win_x_rightb_right; x++) {
			ri++;
			rhigh_vector[ri] = binarized_image.at<uchar>(offset, x);
			if (binarized_image.at<uchar>(offset, x) == 255 && rl == 800) {
				rl = x;
				rr = x;
			}
			if (binarized_image.at<uchar>(offset, x) == 255 && lr != 800) {
				rr = x;
			}
		}

		// window안에서 0이 아닌 픽셀의 개수를 구함
		int lnonzero = countNonZero(lhigh_vector);
		int rnonzero = countNonZero(rhigh_vector);

		// 방금 구했던 255 픽셀 시작 지점과 끝 지점의 중앙 값을 다음 window의 중앙으로 잡는다.
		if (lnonzero >= pixel_thres) {
			left_start = (ll + lr) / 2;
		}
		if (rnonzero >= pixel_thres) {
			right_start = (rl + rr) / 2;
		}

		// 차선 중앙과 탐지한 차선과의 거리 측정
		int lane_mid = (right_start + left_start) / 2;
		int left_diff = lane_mid - left_start;
		int right_diff = -(lane_mid - right_start);
// 1번째 if문
		// 한쪽 차선의 nonzero가 임계값을 넘지 못할 경우 중간을 기점으로 반대편 차선 위치를 기준으로 대칭
		if (lnonzero < pixel_thres && rnonzero > pixel_thres) {
			lane_mid = right_start - right_diff;
			left_start = lane_mid - right_diff;
            // cout << "우측 차선의 대칭 이동" << endl;
		}
		else if (lnonzero > pixel_thres && rnonzero < pixel_thres) {
			lane_mid = left_start + left_diff;
			right_start = lane_mid + left_diff;
            // cout << "좌측 차선의 대칭 이동" << endl;
		}
// 2번째 if문
			// 지난 프레임에서의 픽셀값을 기억하고 nonzero가 임계값을 넘지 못할 경우 지난 프레임의 해당 윈도우 번호의 값을 불러옴
		if (lnonzero < pixel_thres && rnonzero > pixel_thres) {
			left_start = lpoints[window].x;
			lane_mid = (right_start + left_start) / 2;
		}
		else if (lnonzero > pixel_thres && rnonzero < pixel_thres && rpoints[window].x != 0) {
			right_start = rpoints[window].x;
			lane_mid = (right_start + left_start) / 2;
		}
		// cvtColor(binarized_image, binarized_image, COLOR_GRAY2BGR);
		// draw window at v_thres
		rectangle(color_img, Rect(win_x_leftb_left, win_y_high, window_width, window_height), Scalar(0, 0, 255), 2);
		rectangle(color_img, Rect(win_x_rightb_left, win_y_high, window_width, window_height), Scalar(0, 255, 0), 2);
		//rectangle(s_img, Rect(win_x_leftb_left+50, win_y_high+200, window_width, window_height), Scalar(0, 255, 0), 1);
		//rectangle(s_img, Rect(win_x_rightb_left+50, win_y_high+200, window_width, window_height), Scalar(0, 0, 255), 1);


		mpoints[window] = Point(lane_mid, (int)((win_y_high + win_y_low) / 2));
		lpoints[window] = Point(left_start, (int)((win_y_high + win_y_low) / 2));
		rpoints[window] = Point(right_start, (int)((win_y_high + win_y_low) / 2));

    }


	polylines(color_img, lpoints, false, Scalar(0, 100, 200), 15, LINE_AA);
	polylines(color_img, rpoints, false, Scalar(200, 100, 0), 15, LINE_AA);



	return color_img;
}
Mat InversePerspective(Mat& input_img, Mat& output_img){
	Mat unwarp_img;
	Mat result_img;
	warpPerspective(input_img, unwarp_img, inverse_matrix, Size(output_img.cols, output_img.rows), INTER_LINEAR);
	addWeighted(output_img, 1.0, unwarp_img, 0.5, 0, result_img);
	return result_img;
	
}
void ImageCallback(const sensor_msgs::ImageConstPtr& msg){

	try
	{
		Mat base_img = cv_bridge::toCvShare(msg, "bgr8")->image;
		// std::cout << base_img.rows << ", " << base_img.cols << std::endl;
		Mat gray_img = RGB2Gray(base_img);
		Mat filtered_img = GaussianFilter(gray_img);
		Mat contrast_img = Contrast(filtered_img);	
		Mat perspect_img = Perspective(contrast_img);
		Mat binary_img = Binarize(perspect_img);
		// Mat hough_img = Houghtransform(binary_img);
		Mat roi_img = RegionOfInterest(binary_img);
		Mat hist_img = GetHistImage(roi_img);
		Mat search_img = SlidingWindow(roi_img);
		Mat unwarp_img = InversePerspective(search_img, base_img);

		imshow("RESULT Image", unwarp_img);
 	
		waitKey(30); 
	}
	catch (cv_bridge::Exception& e)
	{
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

}

int main (int argc, char** argv){

	ros::init(argc, argv, "sliding_window_node");
	ros::NodeHandle nh;


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/carla/ego_vehicle/rgb_front/image", 1, ImageCallback);
	while (ros::ok())
	{
		ros::spinOnce();
	}

}
