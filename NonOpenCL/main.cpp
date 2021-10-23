#include "stdafx.h"
#include <iostream>

#include "serial.h"

int main(int arg, char* args[]) {

	char RxData[256] = { 0, };
	int Rxlen = sizeof(RxData);
	int readResult = 0;
	///*
	Serial* SP = new Serial("\\\\.\\COM6");

	if (!SP->IsConnected()) {
		std::cout << "connection fail" << std::endl;
		return -1;
	}
	//*/

	cv::VideoCapture videocapture(0);

	if (!videocapture.isOpened()) {
		std::cout << "!camera" << std::endl;
		return -1;
	}

	double height = videocapture.get(cv::CAP_PROP_FRAME_HEIGHT);
	double width = videocapture.get(cv::CAP_PROP_FRAME_WIDTH);

	std::cout << "Size " << width << "x" << height << std::endl;
	//std::cout << "OpenCV Version - " << CV_VERSION << std::endl;


	std::string title = "applied robotics";
	cv::namedWindow(title);
	cv::moveWindow(title, 400, 200);

	bool uniq_lazer = false, uniq_cross = false;
	int count = 0;
	cv::Mat img_raw, img_raw_clone, mNull;
	
	//cross
	cv::Mat img_gray, img_thr, element, img_close, obj;
	cv::Mat labels, stats, centroid;
	cv::Point cross_pos, lazer_pos;

	cv::Mat img_norm, img_dilate;
	
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	//lazer
	cv::Mat img_hsv, mask_red, img_hsv_red;
	cv::Mat hsv_labels, hsv_stats, hsv_centroids;

	std::cout << "\nset ready\n" << std::endl;

	clock_t previous = 0, current = 0, end = 0;

	Sleep(200);
	while (1) {
		current = clock();

		videocapture >> img_raw;//read frame
		img_raw_clone = img_raw.clone();

		//----search cross
		cv::cvtColor(img_raw, img_gray, cv::COLOR_BGR2GRAY);
		cv::adaptiveThreshold(img_gray, img_thr, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 95, 9);//normalize 없을때 87,7
		element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
		cv::morphologyEx(img_thr, img_close, cv::MORPH_CLOSE, element);
		count = cv::connectedComponentsWithStats(img_close, labels, stats, centroid, 8, CV_32S);

		int num = 0;
		for (int i = 1; i < count; i++) {//
			int area = stats.at<int>(i, cv::CC_STAT_AREA);
			int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
			int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
			int y = stats.at<int>(i, cv::CC_STAT_TOP);
			int x = stats.at<int>(i, cv::CC_STAT_LEFT);
			double ratio = width / (double)height;

			//----------------labeling object
			if ((200 < area && area < 600) && (0.3 < ratio && ratio < 1.6)) {
				int offset = 0;
				int rect_x = x - offset, rect_y = y - offset, rect_width = width + offset * 2, rect_height = height + offset * 2;
				obj = img_close(cv::Rect(rect_x, rect_y, rect_width, rect_height));
				cv::findContours(obj, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

				auto contours_size = contours.size();
				num = 0;
				for (int i = 0; i < contours_size; i++) {
					double len = cv::arcLength(contours[i], true);
					if (200 < len) {
						num++;
						cross_pos = cv::Point(x + int(round(width / 2.)), y + int(round(height / 2.)));
						cv::rectangle(img_raw_clone, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 255));
						cv::circle(img_raw_clone, cross_pos, 3, cv::Scalar(0, 0, 255), 1);
					}
				}
			}
		}
		if (num == 1) {
			uniq_cross = true;
		}
		else {
			uniq_cross = false;
		}


		//----------------------------search lazer
		cv::cvtColor(img_raw, img_hsv, cv::COLOR_BGR2HSV);
		//빨강 레이저 포인트 lower(130, 80, 150) higher(180,255,255)
		cv::inRange(img_hsv, cv::Scalar(130, 80, 150), cv::Scalar(180, 255, 255), mask_red);
		count = cv::connectedComponentsWithStats(mask_red, hsv_labels, hsv_stats, hsv_centroids, 8, CV_32S);
		
		for (int i = 1; i < count; i++) {
			int area = hsv_stats.at<int>(i, cv::CC_STAT_AREA);
			int width = hsv_stats.at<int>(i, cv::CC_STAT_WIDTH);
			int height = hsv_stats.at<int>(i, cv::CC_STAT_HEIGHT);
			int y = hsv_stats.at<int>(i, cv::CC_STAT_TOP);
			int x = hsv_stats.at<int>(i, cv::CC_STAT_LEFT);
			double ratio = width / (double)height;
			if (count == 2) {
				uniq_lazer = true;
			}
			else {
				uniq_lazer = false;
			}
			lazer_pos = cv::Point(x + int(round(width / 2.)), y + int(round(height / 2.)));
			cv::rectangle(img_raw_clone, cv::Rect(x - 3, y - 3, width + 6, height + 6), cv::Scalar(0, 255, 0));
		}

		end = current - previous;
		previous = current;
		//------communication Serial
		///*
		if (uniq_cross == true && uniq_lazer == true) {
			std::string l_x = std::to_string(lazer_pos.x);
			std::string l_y = std::to_string(lazer_pos.y);
			std::string c_x = std::to_string(cross_pos.x);
			std::string c_y = std::to_string(cross_pos.y);

			std::string write = c_x + " " + c_y + " " + l_x + " " + l_y + "\n";
			SP->WriteData(write.c_str(), write.length());

		}//*/

		//-----display
		int fps = round(1000. / end);
		//std::cout << fps << std::endl;
		cv::putText(img_raw_clone, std::to_string(fps), cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));\
		cv::imshow(title, img_raw_clone);

		if (cv::waitKey(5) == 27) {
			std::cout << "exit" << std::endl;
			break;
		}
	}

	std::cout << "end th loop" << std::endl;

	videocapture.release();
	cv::destroyAllWindows();
	std::cout << "destroy windows" << std::endl;
	return 0;
}