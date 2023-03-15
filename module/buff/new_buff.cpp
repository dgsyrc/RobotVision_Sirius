/**
 * @file new_buff.hpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief 新能量机关检测
 * @date 2023-01-15
 * 
 * @copyright Copyright (c) 2023 Sirius
 * 
 */

#define gimbal_delay 800.0f //ms

#include "new_buff.hpp" 

namespace new_buff {
    void buff::set_config(std::string config_path) {
        cv::FileStorage conf(config_path, cv::FileStorage::READ);
        conf["ARMOR_HEIGHT"] >> config.armor_height;
        conf["ARMOR_LENGHT"] >> config.armor_lenght;
        conf["PIC_ARMOR_HEIGHT"] >> config.pic_armor_height;
        conf["PIC_ARMOR_LENGHT"] >> config.pic_armor_lenght;
        conf["PIC_DISTANCE"] >> config.pic_distance;
        conf["ARMOR_DISTANCE"] >> config.armor_distance;
        conf.release();
    }

    void buff::getcontour(cv::Mat imgDil,cv::Mat img, cv::Mat& img_src,new_buff::Check_Moudle moudle)//判断轮廓
    {
        int objCor;
        std::vector<std::vector<cv::Point>> contour;
        std::vector<cv::Vec4i> hierarchy;
        findContours(imgDil, contour, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS);
        
        std::vector<std::vector<cv::Point>> conPoly(contour.size());
        std::vector<cv::Rect> boundRect(contour.size());
        fmt::print("[img process] contour.size() {}\n",contour.size());
        for (int i = 0; i < contour.size(); i++)
        {
            int area;
            area = contourArea(contour[i]);//轮廓面积用于排除黑点
            std::string objectType;//形状

            if (area > 10)//排除小黑圆点的干扰
            {
                float peri = cv::arcLength(contour[i], true);
                approxPolyDP(contour[i], conPoly[i], 0.02 * peri, true);//把一个连续光滑曲线折线化			
                //std::cout << conPoly[i].size() << endl;//边数
                fmt::print("[img process] conPoly.size {}\n",conPoly[i].size());
                boundRect[i] = boundingRect(conPoly[i]);

                objCor = (int)conPoly[i].size();
                /*if (objCor == 3) { objectType = "Tri"; }
                else if (objCor == 4) {
                    float aspRatio = (float)boundRect[i].width / boundRect[i].height;//长宽比来区别正方形与矩形
                    //cout << aspRatio << endl;
                    fmt::print("[img process] aspRatio {}\n",aspRatio);
                    if (aspRatio > 0.95 && aspRatio < 1.05) { objectType = "Square"; }
                    else { objectType = "Rect"; }
                }
                else */
                fmt::print("[img process] Circle {} area {}\n",objCor,area);
                if (objCor > 4&&area<4800) {
                    
                    if(moudle==INACTION_MODE&&area>4200&&area<4700/*&&hierarchy[i][3]== 3*/)
                    {
                        objectType = "Circle";
                        fmt::print("[img process] Circle {} area {}\n",objCor,area);
                        fmt::print("[img process] hier {}\n",hierarchy[i][3]);
                        drawContours(img, conPoly, i, cv::Scalar(255, 0, 255), 2);
                        rectangle(img_src, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 0), 5);//框出图形
                        armor_last.x = armor_now.x;
                        armor_last.y = armor_now.y;
                        armor_now.x = (boundRect[i].tl().x + boundRect[i].br().x)/2;
                        armor_now.y = (boundRect[i].tl().y + boundRect[i].br().y)/2;
                        cv::putText(img_src, std::to_string(area), { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                    }
                    if(moudle==ACTION_MODE&&((area>1700&&area<1770))&&(hierarchy[i][3]>= 0/*&&hierarchy[i][3]<=3*/))
                    {
                        
                        objectType = "Circle";
                        fmt::print("[img process] Circle {} area {}\n",objCor,area);
                        drawContours(img, conPoly, i, cv::Scalar(255, 0, 255), 2);
                        rectangle(img_src, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 255, 0), 5);//框出图形
                        cv::putText(img_src, std::to_string(area), { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                    }
                    if(moudle==CIRCLE_MODE&&area>730&&area<760/*&&hierarchy[i][3]>=0*//*&&hierarchy[i][3]<=30*/)
                    {
                        objectType = "Circle";
                        fmt::print("[img process] Circle {} area {}\n",objCor,area);
                        fmt::print("[img process] hier {}\n",hierarchy[i][3]);
                        drawContours(img, conPoly, i, cv::Scalar(255, 0, 255), 2);
                        rectangle(img_src, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 255), 5);//框出图形
                        cv::putText(img_src, std::to_string(area), { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                        circles=(boundRect[i].tl()+boundRect[i].br())/2;
                    }
                }
                /*if(moudle==INACTION_MODE)
                {
                    cv::putText(img_src, "INACTION", { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                }
                if(moudle==ACTION_MODE)
                {
                    cv::putText(img_src, "ACTION", { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                }*/
                
            }
        }
    }
   

    void buff::predict(cv::Point2f &circle_r) {
        relay.x = (armor_last.x + armor_now.x) / 2.0;
        relay.y = (armor_last.y + armor_now.y) / 2.0;
        circles.x = (armor_last.x +armor_now.x)/ 2.0;
        circles.y = (armor_last.y +armor_now.y)/ 2.0;
        fmt::print("[buff] relay.x:{} relay.y:{}\n", relay.x, relay.y);
        fmt::print("[buff] circle_r.x:{} circle_r.y:{}\n", circle_r.x, circle_r.y);
        /*arg[0] = sqrt(pow(circle_r.x - relay.x,2) + pow(circle_r.y - relay.y,2));
        arg[1] = abs(armor_now.x - armor_last.x);
        arg[2] = abs(armor_now.y - armor_last.y);*/
        //alpha = 2 * acosf((pow(arg[1],2) + pow(arg[2],2) - pow(arg[0],2)) / (2 * arg[1] * arg[2]));
        fmt::print("[buff] sqrt:{}\n", sqrt(pow(circle_r.x - relay.x,2) + pow(circle_r.y - relay.y,2)) / sqrt(pow(circle_r.x - armor_now.x,2) + pow(circle_r.y - armor_now.y,2)));
        alpha = 2 * acos(sqrt(pow(circle_r.x - relay.x,2) + pow(circle_r.y - relay.y,2)) / sqrt(pow(circle_r.x - armor_now.x,2) + pow(circle_r.y - armor_now.y,2)));
        new_buff_fps.calculateFPSGlobal();
        fmt::print("[buff] fps {}\n", new_buff_fps.returnFps());
        d_t = 1.0 / new_buff_fps.returnFps();
        fmt::print("[buff] d_t {}\n", d_t);
        velocity = alpha / d_t;
        fmt::print("[buff] velocity {}\n", velocity);
        fmt::print("[buff] acos {}\n", acos(1.0));

        if(max_velocity == 0.0 && min_velocity == 0.0) { // 初始化
            max_velocity = velocity;
            min_velocity = velocity;
            status = NONE;
            start_T = false;
        }

        if(start_T) { // 若开始计时
            switch (status)
            {
                case MAX:
                    if(abs(velocity - max_velocity) < 0.1 || (now_velocity - last_velocity) * (velocity - now_velocity) < 0) {
                        T = (cv::getTickCount() - timex) / cv::getTickFrequency();
                        omega = 2 * PI / T;
                        A = (max_velocity - min_velocity) / 2.0;
                    }
                    break;
                case MIN:
                    if(abs(velocity - min_velocity) < 0.1 || (last_velocity - now_velocity) * (now_velocity - velocity) < 0) {
                        T = (cv::getTickCount() - timex) / cv::getTickFrequency();
                        omega = 2 * PI / T;
                        A = (max_velocity - min_velocity) / 2.0;
                    }
                    break;
            
                default:
                    break;
            }
        }
        if(now_velocity == 0) {
            last_velocity = now_velocity;
            now_velocity = velocity;
        } else {
            if(now_velocity < velocity) {
                if(max_velocity < velocity) {
                    max_velocity = velocity;
                }
                if(status == MIN) {  // 若上一状态为减，则上一状态所在点定为极小值
                    start_T = true;
                    sign = 1;
                    timex = last_timex; // 函数初始时间
                } else {
                    status = MAX;
                }
                last_timex = cv::getTickCount();
            } 

            if(now_velocity > velocity) {
                if(min_velocity > velocity) {
                    min_velocity = velocity;
                }
                last_timex = cv::getTickCount();
                if(status == MAX) { // 若上一状态为增，则上一状态所在点定为极大值
                    start_T = true;
                    sign = -1;
                    timex = last_timex; // 函数初始时间
                } else {
                    status = MIN;
                }
                last_timex = cv::getTickCount();
            }

            last_velocity = now_velocity;
            now_velocity = velocity;
        }
        
    }

    cv::Point2f buff::calculateCord(cv::Point2f &circle_r) {
        fmt::print("[buff] A:{} omega:{} sign:{} alpha:{}\n", A, omega, sign, alpha);
        beta = - A / omega * (cos(omega * ((cv::getTickCount() - timex) / cv::getTickFrequency() + gimbal_delay / 1000.0) + sign * PI / 4.0) - 
                              cos(omega * ((cv::getTickCount() - timex) / cv::getTickFrequency()) + sign * PI / 4.0))
               + (2.090 - A) * (gimbal_delay / 1000.0);
        fmt::print("[buff] beta:{}\n", beta);
        object.x = armor_now.x * cos(beta) - armor_now.y * sin(beta);
        object.y = armor_now.x * sin(beta) + armor_now.y * cos(beta);

        return object;
    }

     void buff::main_buff_checker(cv::Mat imgGray,cv::Mat img_src,new_buff::Check_Moudle moudle){
		cv::Mat /*imgGray, */imgBlur, imgCanny, imgDila, imgErode;
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        imgDila = imgGray.clone();
		//preprocessing
		//cvtColor(img, imgGray,cv::COLOR_BGR2GRAY, 0);
		/*GaussianBlur(imgGray, imgBlur, cv::Size(65, 65), 1, 1);
		Canny(imgBlur, imgCanny, 40, 120);
		dilate(imgCanny, imgDila, kernel);*/
        //morphologyEx(imgDila, imgDila, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 5);
        //morphologyEx(imgDila, imgDila, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 2);
        //morphologyEx(imgDila, imgDila, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 7);
        //morphologyEx(imgDila, imgDila, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 1);
        if(moudle==INACTION_MODE)
        {
            //morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 1);
            morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 10);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 3);
            morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 5);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 1);
            //morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 4);
            //morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 1);
        }
        /*if(moudle==ACTION_MODE)
        {
            morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 3);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 2);
        }
        if(moudle==CIRCLE_MODE)
        {
            morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 2);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 2);
        }*/
        GaussianBlur(imgGray, imgBlur, cv::Size(65, 65), 1, 1);
		Canny(imgBlur, imgCanny, 40, 120);
		dilate(imgCanny, imgDila, kernel);
//灰度->高斯滤波->Canny边缘算法->膨胀
		buff::getcontour(imgDila,imgGray,img_src,moudle);
	    imshow("Image", imgGray);
        imshow("Image2", imgDila);
        imshow("detect inaction", img_src);
	//imshow("Image Gray", imgGray);
	//imshow("Image Blur", imgBlur);
	//imshow("Image Canny", imgCanny);
	//imshow("Image dila", imgDila);
    }
}
