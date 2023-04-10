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
//#define RELEASE

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

    void buff::getcontour(cv::Mat imgDil,cv::Mat imgDil2, cv::Mat& img_src,new_buff::Check_Moudle moudle)//判断轮廓
    {
        int objCor,cnt_obj;
        cnt_obj=0;
        std::vector<std::vector<cv::Point>> contour;
        std::vector<cv::Vec4i> hierarchy;
        findContours(imgDil, contour, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS);
        
        std::vector<std::vector<cv::Point>> conPoly(contour.size());
        std::vector<cv::Rect> boundRect(contour.size());

        std::vector<armor_check> inaction_check;
        //fmt::print("[img process] contour.size() {}\n",contour.size());
        area_circles=0.0f;
        for (int i = 0; i < contour.size(); i++)
        {
            int area;
            int area_rect;
            area = contourArea(contour[i]);//轮廓面积用于排除黑点
            
            std::string objectType;//形状
            
            if (area > 10)//排除小黑圆点的干扰
            {
                float peri = cv::arcLength(contour[i], true);
                approxPolyDP(contour[i], conPoly[i], 0.02 * peri, true);//把一个连续光滑曲线折线化			
                //std::cout << conPoly[i].size() << endl;//边数
                //fmt::print("[img process] conPoly.size {}\n",conPoly[i].size());
                boundRect[i] = boundingRect(conPoly[i]);
                
                area_rect = (boundRect[i].br().x-boundRect[i].tl().x)*(boundRect[i].br().y-boundRect[i].tl().y);
                objCor = (int)conPoly[i].size();
               
                //fmt::print("[img process] Circle {} area {}\n",objCor,area);
                
                cv::imshow("imgdil2",imgDil2);
                if (objCor > 4&&area<4800) {
                    
                    if(moudle==INACTION_MODE){
                        //if(area>300){
                            //cv::rectangle(img_src, boundRect[i].tl() ,boundRect[i].br(),cv::Scalar(200, 255, 125), 2, 8);
                        //}
                        
                        cv::imshow("img_inaction",img_src);
                        double ratio,dis,max_dist;
                        cv::Point P_a,P_b;
                        cv::Point2f P_re;
                       
                        max_dist=0;
                        // 找最长两点连线
                        for(int j=0;j < conPoly[i].size();++j){
                            for(int k=j+1;k < conPoly[i].size();++k){
                                double dist;
                               
                                dist=returnDistance(conPoly[i][j],conPoly[i][k]);
                                if(dist>max_dist){
                                    max_dist=dist;
                                    P_a=conPoly[i][j];
                                    P_b=conPoly[i][k];
                                }
                            }
                        }
                        P_a.x=(int)kal.run(P_a.x);
                        P_a.y=(int)kal.run(P_a.y);
                        P_b.x=(int)kal.run(P_b.x);
                        P_b.y=(int)kal.run(P_b.y);
                        
                        
                        max_dist=returnDistance(P_a,P_b);
                        if(fabs(fabs(returnDistance((P_a+P_b)/2,circles)/max_dist-56.0/25.0))<0.2/*&&dis>60.0*/){
                            armor_check tmp;
                            
                            //dis=returnDistance((boundRect[i].tl()+boundRect[i].br())/2,circles);
                            dis=returnDistance((P_a+P_b)/2,circles);
                            //cv:line(img_src,boundRect[i].tl(),boundRect[i].br(),cv::Scalar(100, 255, 125),2);
                            cv:line(img_src,P_a,P_b,cv::Scalar(100, 255, 125),2);
                            ratio=fabs(fabs(returnDistance((P_a+P_b)/2,circles)/max_dist-56.0/25.0));
                            tmp.cord[0].x=((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*50.0/56.0+circles.x;
                            tmp.cord[0].y=((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*50.0/56.0+circles.y;
                            tmp.cord[1].x=((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*54.0/56.0+circles.x;
                            tmp.cord[1].y=((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*54.0/56.0+circles.y;
                            tmp.cord[2].x=((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*56.0/56.0+circles.x;
                            tmp.cord[2].y=((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*56.0/56.0+circles.y;
                            tmp.cord[3].x=((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*53.0/56.0+circles.x;
                            tmp.cord[3].y=((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*53.0/56.0+circles.y;
                            tmp.obj.x=((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*47.0/56.0+circles.x;
                            tmp.obj.y=((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*47.0/56.0+circles.y;
                            tmp.d=returnDistance(boundRect[i].tl(),boundRect[i].br())*0.5;
                            tmp.isPass=true;
                            inaction_check.push_back(tmp);
                            cv::circle(img_src,{tmp.cord[0].x,tmp.cord[0].y},1,cv::Scalar(100, 255, 125),2);
                            cv::circle(img_src,{tmp.cord[1].x,tmp.cord[1].y},1,cv::Scalar(100, 255, 125),2);
                            cv::circle(img_src,{tmp.cord[2].x,tmp.cord[2].y},1,cv::Scalar(100, 255, 125),2);
                            cv::circle(img_src,{tmp.obj.x,tmp.obj.y},1,cv::Scalar(100, 0, 125),2);
                            cv::circle(img_src,P_a,1,cv::Scalar(100, 255, 125),2);
                            cv::circle(img_src,P_b,1,cv::Scalar(100, 255, 125),2);
                            //cv::circle(img_src,(P_a+P_b)/2,1,cv::Scalar(100, 255, 125),2);
                            cv::rectangle(img_src, boundRect[i].tl() ,boundRect[i].br(),cv::Scalar(200, 255, 125), 2, 8);
                            cv::line(img_src,circles,tmp.obj,cv::Scalar(100, 255, 125),2);
                            cv::putText(img_src, std::to_string(ratio), { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                            cv::imshow("inaction",img_src);
                            //cv::circle(img_src,{((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*48.0/56.0+circles.x,((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*48.0/56.0+circles.y},2,cv::Scalar(255,0,255),1);
                        }/*else{
                            line(img_src,P_a,P_b,cv::Scalar(100, 255, 125),2);
                            if(returnDistance(P_a,circles)<returnDistance(P_b,circles)){
                                P_re.x = (P_a.x-circles.x) * cos(PI/180*15) - (P_a.y-circles.y) * sin(PI/180*15) + circles.x;
                                P_re.y = (P_a.x-circles.x) * sin(PI/180*15) + (P_a.y-circles.y) * cos(PI/180*15) + circles.y;
                                if(imgDil.at<uchar>((cv::Point)((P_re-circles)*0.5+circles))!=0){
                                    P_re.x = (P_a.x-circles.x) * cos(PI/180*15*(-1.0)) - (P_a.y-circles.y) * sin(PI/180*15*(-1.0)) + circles.x;
                                    P_re.y = (P_a.x-circles.x) * sin(PI/180*15*(-1.0)) + (P_a.y-circles.y) * cos(PI/180*15*(-1.0)) + circles.y;
                                    line(img_src,P_re,P_b,cv::Scalar(0, 96, 255),2);
                                }else{
                                    line(img_src,P_re,P_b,cv::Scalar(0, 96, 255),2);
                                }
                            
                            }else{
                                P_re.x = (P_b.x-circles.x) * cos(PI/180*15) - (P_b.y-circles.y) * sin(PI/180*15) + circles.x;
                                P_re.y = (P_b.x-circles.x) * sin(PI/180*15) + (P_b.y-circles.y) * cos(PI/180*15) + circles.y;
                                if(imgDil.at<uchar>((cv::Point)((P_re-circles)*0.5+circles))!=0){
                                    P_re.x = (P_b.x-circles.x) * cos(PI/180*15*(-1.0)) - (P_b.y-circles.y) * sin(PI/180*15*(-1.0)) + circles.x;
                                    P_re.y = (P_b.x-circles.x) * sin(PI/180*15*(-1.0)) + (P_b.y-circles.y) * cos(PI/180*15*(-1.0)) + circles.y;
                                    line(img_src,P_a,P_re,cv::Scalar(0, 96, 255),2);
                                }else{
                                    line(img_src,P_a,P_re,cv::Scalar(0, 96, 255),2);
                                }
                            
                            }
                            if(abs((boundRect[i].br().x-boundRect[i].tl().x)*1.0/(boundRect[i].br().y-boundRect[i].tl().y)*1.0-1.0)<0.2){
                                armor_check tmp;
                                tmp.cord[0].x=((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*38.0/39.0+circles.x;
                                tmp.cord[0].y=((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*38.0/39.0+circles.y;
                                tmp.cord[1].x=((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*36.0/39.0+circles.x;
                                tmp.cord[1].y=((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*36.0/39.0+circles.y;
                                tmp.cord[2].x=((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*41.0/39.0+circles.x;
                                tmp.cord[2].y=((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*41.0/39.0+circles.y;
                                tmp.cord[3].x=((boundRect[i].tl().x+boundRect[i].br().x)/2-circles.x)*43.0/39.0+circles.x;
                                tmp.cord[3].y=((boundRect[i].tl().y+boundRect[i].br().y)/2-circles.y)*43.0/39.0+circles.y;
                                tmp.obj.x=(boundRect[i].tl().x+boundRect[i].br().x)/2;
                                tmp.obj.y=(boundRect[i].tl().y+boundRect[i].br().y)/2;
                                cv::circle(img_src,{tmp.cord[0].x,tmp.cord[0].y},1,cv::Scalar(100, 255, 125),2);
                                cv::circle(img_src,{tmp.cord[1].x,tmp.cord[1].y},1,cv::Scalar(100, 255, 125),2);
                                cv::circle(img_src,{tmp.cord[2].x,tmp.cord[2].y},1,cv::Scalar(100, 255, 125),2);
                                cv::rectangle(img_src, boundRect[i].tl() ,boundRect[i].br(),cv::Scalar(200, 255, 125), 2, 8);
                                cv::putText(img_src, std::to_string(area), { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                                tmp.d=returnDistance(boundRect[i].tl(),boundRect[i].br());
                                 cv::putText(img_src, std::to_string(tmp.d), { tmp.obj.x,tmp.obj.y },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                                if(tmp.d<100.0){
                                    tmp.isPass=true;
                                    inaction_check.push_back(tmp);
                                    cv::circle(img_src,{tmp.cord[2].x,tmp.cord[2].y},1,cv::Scalar(255, 255, 125),2);
                                }
                                cv::imshow("inaction",img_src);
                                
                            }
                        }*/
                    }
                    /*
                    if(moudle==INACTION_MODE&&area>150&&area<300&&area_rect>200&&area_rect<390&&hierarchy[i][3]>= 1)
                    //if(moudle==INACTION_MODE&&area>3200&&area<3700&&area_rect>3200&&area_rect<5300&&hierarchy[i][3]>= 1)
                    {
                        objectType = "Circle";
                        //fmt::print("[img process] Circle {} area {}\n",objCor,area);
                        //fmt::print("[img process] hier {}\n",hierarchy[i][3]);
                        drawContours(img, conPoly, i, cv::Scalar(255, 0, 255), 2);
                        //rectangle(img_src, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 0), 5);//框出图形
                        tlx = boundRect[i].tl().x;
                        tlx =(int)kal.run(boundRect[i].tl().x);
                        tly = boundRect[i].tl().y;
                        tly =(int)kal.run(boundRect[i].tl().y);
                        brx = boundRect[i].br().x;
                        brx =(int)kal.run(boundRect[i].br().x);
                        bry = boundRect[i].br().y;
                        bry =(int)kal.run(boundRect[i].br().y);
                        //rectangle(img_src, {tlx,tly}, {brx,bry}, cv::Scalar(0, 255, 0), 5);
                        armor_last.x = armor_now.x;
                        armor_last.y = armor_now.y;
                        //armor_now.x = (boundRect[i].tl().x + boundRect[i].br().x)/2;
                        //armor_now.y = (boundRect[i].tl().y + boundRect[i].br().y)/2;
                        armor_now.x = (tlx + brx)/2;
                        armor_now.y = (tly + bry)/2;
                        //if(isfindcircleR){
                            cv::line(img_src,{(tlx+brx)/2,(tly+bry)/2},circles,cv::Scalar(0,255,255),2);
                            fmt::print("[armor action] cir {} {}\n",circles.x,circles.y);
                     cv::putText(img_src, std::to_string(imgDil.at<uchar>(inaction_check[j].cord[0])), { inaction_check[j].cord[0].x,inaction_check[j].cord[0].y-5},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);   //}
                        
                        cv::putText(img_src, std::to_string(area_rect), { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                        armor_last_rects = armor_now_rects;
                        tan_angle = atan(abs(armor_now.y-circles.y)/abs(armor_now.x-circles.x));
                        
                        armor_now_rects = cv::RotatedRect(armor_now , cv::Size(abs(brx-tlx), abs(bry-tly)) , tan_angle);
                        cv::rectangle(img_src, armor_now_rects.boundingRect(),cv::Scalar(0, 255, 0), 2, 8);
                    }
                    else
                    {
                        if(moudle==INACTION_MODE){

                        }
                    }*/
                    /*if(moudle==ACTION_MODE&&((area>1700&&area<1770))&&(hierarchy[i][3]>= 0&&hierarchy[i][3]<=3))
                    {
                        
                        objectType = "Circle";
                        //fmt::print("[img process] Circle {} area {}\n",objCor,area);
                        drawContours(img, conPoly, i, cv::Scalar(255, 0, 255), 2);
                        rectangle(img_src, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 255, 0), 5);//框出图形
                        cv::putText(img_src, std::to_string(area), { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                    }*/
                    isfindcircleR=false;
                    //cv::imshow("cir_img",imgDil);
                    if(moudle==CIRCLE_MODE&&area>10&&area<15&&abs((boundRect[i].br().x-boundRect[i].tl().x)*1.0/(boundRect[i].br().y-boundRect[i].tl().y)*1.0-1.0)<0.3&&area_rect>10&&area_rect<32&&hierarchy[i][3]==-1)
                    //if(moudle==CIRCLE_MODE&&area>400&&area<500&&abs((boundRect[i].br().x-boundRect[i].tl().x)*1.0/(boundRect[i].br().y-boundRect[i].tl().y)*1.0-1.0)<0.3&&area_rect>500&&area_rect<1000/*&&hierarchy[i][3]==1*/)
                    {
                        cv::imshow("img_cir",imgDil);
                        objectType = "Circle";
                        fmt::print("[img process] Circle {} area {}\n",objCor,area);
                        fmt::print("[img process] hier {}\n",hierarchy[i][3]);
                        //drawContours(img, conPoly, i, cv::Scalar(255, 0, 255), 2);
                        
                        cv::putText(img_src, std::to_string(area_rect), { boundRect[i].x,boundRect[i].y - 5 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                        tlx = boundRect[i].tl().x;
                        tlx =(int)kal.run(boundRect[i].tl().x);
                        tly = boundRect[i].tl().y;
                        tly =(int)kal.run(boundRect[i].tl().y);
                        brx = boundRect[i].br().x;
                        brx =(int)kal.run(boundRect[i].br().x);
                        bry = boundRect[i].br().y;
                        bry =(int)kal.run(boundRect[i].br().y);
                        //circles=(boundRect[i].tl()+boundRect[i].br())/2;
                        last_r=returnDistance(circles,armor_last);
                        now_r=returnDistance(circles,armor_now);
                        last_r=(last_r+now_r)/2.0;
                        now_r=returnDistance({(tlx+brx)/2,(tly+bry)/2},armor_now);
                        /*if(fabs(last_r-now_r)/((last_r+now_r)/2.0)>0.1&&fabs(armor_last.x-0.0)>0.01&&fabs(armor_last.y-0.0)>0.01)
                        {
                            fmt::print("[check] Err 1\n");
                            continue;
                        }*/
                        rectangle(img_src, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 255), 2);//框出图形
                        cv::imshow("circle",img_src);
                        /*if(returnDistance(circles,{tlx,tly})<700.0&&returnDistance(circles,{tlx,tly})>50.0&&last_circles.x!=0&&last_circles.y!=0){
                            continue;
                        }*/
                        //if(returnDistance({(tlx+brx)/2,(tly+bry)/2},armor_now))
                        
                        
                        last_circles=circles;
                        circles.x=(tlx+brx)/2;
                        circles.y=(tly+bry)/2;
                        fmt::print("[img process] circle {} {}\n", circles.x,circles.y);
                        //cv::line(img_src,armor_now,circles,cv::Scalar(0,255,255),2);
                       
                        isfindcircleR=true;
                        

                    }
                    now_r=0;
                    for(int j=0;j < inaction_check.size(); ++j){
                        int col_cnt;
                        col_cnt=0;
                        
                        /*for(int k=0;k<contour.size();++k) {
                            if(boundRect[k].tl().x<inaction_check[j].cord.x&&boundRect[k].tl().y<inaction_check[j].cord.y
                            &&boundRect[k].br().x>inaction_check[j].cord.x&&boundRect[k].br().y>inaction_check[j].cord.y){
                                inaction_check[j].isPass=false;
                                //rectangle(img_src, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 255), 2);
                                break;
                            }
                        }*/
                        //cv::circle(img_src,inaction_check[j].obj,2,cv::Scalar(255,0,255),1);
                        cv::imshow("check",img_src);
                        for(int k=38;k<=54;k++){
                            cv::circle(img_src,{(inaction_check[j].obj.x-circles.x)*k*1.0/47.0+circles.x,(inaction_check[j].obj.y-circles.y)*k*1.0/47.0+circles.y},3,cv::Scalar(255,0,255),3);
                            col_cnt=col_cnt+imgDil.at<uchar>({(inaction_check[j].obj.x-circles.x)*k*1.0/47.0+circles.x,(inaction_check[j].obj.y-circles.y)*k*1.0/47.0+circles.y});
                        }
                        cv::putText(img_src, std::to_string(col_cnt/(24.0*255.0)), { inaction_check[j].obj.x,inaction_check[j].obj.y-5},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 255, 255), 2);
                        fmt::print("[check] {}\n",col_cnt);
                        
                        if(col_cnt/(16.0*255.0)<0.29||returnDistance(inaction_check[j].obj,circles)<43.0){
                            inaction_check[j].isPass=false;
                            continue;
                        }
                        fmt::print("[check] pass\n");
                        inaction_check[j].isPass=true;
                        //cv::putText(img_src, std::to_string(imgDil.at<uchar>(inaction_check[j].cord[0])), { 30,20 },1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                        //cv::putText(img_src, std::to_string(imgDil.at<uchar>(inaction_check[j].cord[0])), { inaction_check[j].cord[0].x,inaction_check[j].cord[0].y-5},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                        //cv::putText(img_src, std::to_string(imgDil.at<uchar>(inaction_check[j].cord[1])), { inaction_check[j].cord[1].x,inaction_check[j].cord[1].y-5},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                        //cv::putText(img_src, std::to_string(imgDil.at<uchar>(inaction_check[j].cord[2])), { inaction_check[j].cord[2].x,inaction_check[j].cord[2].y-5},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                        //cv::putText(img_src, std::to_string(imgDil.at<uchar>(inaction_check[j].cord[3])), { inaction_check[j].cord[3].x,inaction_check[j].cord[3].y-5},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                        /*if(imgDil.at<uchar>(inaction_check[j].cord[0])+imgDil.at<uchar>(inaction_check[j].cord[1])+imgDil.at<uchar>(inaction_check[j].cord[2])+imgDil.at<uchar>(inaction_check[j].cord[3])==0){
                            inaction_check[j].isPass=true;
                            //cv::putText(img_src, std::to_string(imgDil.at<uchar>(inaction_check[j].cord[0])+imgDil.at<uchar>(inaction_check[j].cord[1])+imgDil.at<uchar>(inaction_check[j].cord[2])+imgDil.at<uchar>(inaction_check[j].cord[3])), { inaction_check[j].obj.x,inaction_check[j].obj.y-5},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                            
                        }else{
                            inaction_check[j].isPass=false;
                        }*/
                    }
                    
                    //circles.x=439;
                    //circles.y=388;
                    /*if(!isfindcircleR)
                    {
                        circles.x=0;
                        circles.y=0;
                    }*/
                }            
            }
        }
        for(int i=0;i<inaction_check.size();++i){
            if(inaction_check[i].isPass){   
                cv::Point2f calcord,circal;
                isFindTarget=true;
                cnt_obj++;
                calcord.x=(int)kal.run(inaction_check[i].obj.x);
                calcord.y=(int)kal.run(inaction_check[i].obj.y);
                
                //circal=calculateCircleR(calcord,armor_now,armor_last);
                //cv::circle(img_src,circal,3,cv::Scalar(100, 255, 125),2);
                //lasnow_r=0;t_r=returnDistance(circles,armor_last);
                //now_r=returnDistance(circles,armor_now);
                //last_r=(last_r+now_r)/2.0;
                armor_last.x = armor_now.x;
                armor_last.y = armor_now.y;
                
                //armor_now.x = (int)kal.run(inaction_check[i].cord.x);
                //armor_now.y = (int)kal.run(inaction_check[i].cord.y);
                armor_now.x = kal.run(calcord.x);
                armor_now.y = kal.run(calcord.y);

                //now_r=returnDistance(circles,armor_now);
               
                
                

                /*if(fabs(last_r-now_r)/((last_r+now_r)/2.0)>0.1&&fabs(armor_last.x-0.0)>0.01&&fabs(armor_last.y-0.0)>0.01)
                {
                    fmt::print("[check] Err 1\n");
                    circles={0,0};
                    continue;
                }*/
                cv::line(img_src,armor_now,circles,cv::Scalar(0,255,255),2);
                armor_last_rects = armor_now_rects;
                
                tan_angle = atan(abs(armor_now.y-circles.y)/abs(armor_now.x-circles.x));
                        
                armor_now_rects = cv::RotatedRect(armor_now , cv::Size((int)inaction_check[i].d, (int)inaction_check[i].d) , tan_angle);
                //cv::putText(img_src, std::to_string(inaction_check[i].d), { inaction_check[i].obj.x,inaction_check[i].obj.y-5},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
                cv::rectangle(img_src, armor_now_rects.boundingRect(),cv::Scalar(0, 200, 0), 2, 8);
                
                cv::putText(img_src, "Dectected", {10,10},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 255, 255), 2);
                cv::imshow("[dec]",img_src);
            }
        }
        if(cnt_obj==0)
        {
            isFindTarget=false;
        }
        /*for(int i=0;i<armor.size();++i){
            fmt::print("[armor_c] in\n");
            if(now_r==0){
                now_r=returnDistance(circles,armor[i].obj);
            }else{
                cv::putText(img_src, std::to_string(armor[i].d), { armor[i].obj.x,armor[i].obj.y-5},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 255, 255), 2);
                if(fabs(now_r-returnDistance(circles,armor[i].obj))/now_r<0.1){
                    continue;
                }else{
                    circles={0,0};
                }
            }
        }*/
    }
   

    cv::Point2f buff::stablePerdict(cv::Point2f &circle_r,cv::Mat &img){
        fmt::print("[buff] circleR {} {}\n", circle_r.x, circle_r.y);
        cv::circle(img,circle_r,3,cv::Scalar(0,255,255));
        cv::circle(img,armor_last,5,cv::Scalar(0,255,255));
        cv::circle(img,armor_now,5,cv::Scalar(0,255,255));
        object.x = (armor_now.x-circle_r.x) * cos(PI/180*42) - (armor_now.y-circle_r.y) * sin(PI/180*42) + circle_r.x;
        object.y = (armor_now.x-circle_r.x) * sin(PI/180*42) + (armor_now.y-circle_r.y) * cos(PI/180*42) + circle_r.y;
        cv::circle(img,object,5,cv::Scalar(0,255,0));
        center_dist = returnDistance(armor_last,armor_now);
        cv::circle(img,circle_r,returnDistance(circle_r,object),cv::Scalar(0,255,0));
        cv::putText(img, std::to_string(center_dist), {(int)armor_now.x,(int)armor_now.y},1,cv::FONT_HERSHEY_PLAIN, cv::Scalar(0, 69, 255), 1);
        
        cv::imshow("predict img",img);
        if(isFindTarget) {
            object_rects = cv::RotatedRect(object,armor_now_rects.size,armor_now_rects.angle);
            fmt::print("[obj rect] in {}\n",object_rects.size.height);
            return object;
        } else {
            return (cv::Point2f){0,0};
        }
    }

    void buff::predict(cv::Point2f &circle_r,cv::Mat &img) {
        
        
        cv::circle(img,armor_now,2,cv::Scalar(0,255,0));
        relay.x = (armor_last.x + armor_now.x) / 2.0;
        relay.y = (armor_last.y + armor_now.y) / 2.0;
        circles.x = (armor_last.x +armor_now.x)/ 2.0;
        circles.y = (armor_last.y +armor_now.y)/ 2.0;
        //fmt::print("[buff] relay.x:{} relay.y:{}\n", relay.x, relay.y);
        //fmt::print("[buff] circle_r.x:{} circle_r.y:{}\n", circle_r.x, circle_r.y);

        // delete
        //arg[0] = sqrt(pow(circle_r.x - relay.x,2) + pow(circle_r.y - relay.y,2));
        //arg[1] = abs(armor_now.x - armor_last.x);
        //arg[2] = abs(armor_now.y - armor_last.y);
        //alpha = 2 * acosf((pow(arg[1],2) + pow(arg[2],2) - pow(arg[0],2)) / (2 * arg[1] * arg[2]));
        // delete end

        fmt::print("[buff] sqrt:{}\n", sqrt(pow(circle_r.x - relay.x,2) + pow(circle_r.y - relay.y,2)) / sqrt(pow(circle_r.x - armor_now.x,2) + pow(circle_r.y - armor_now.y,2)));
        alpha = 2 * acos(sqrt(pow(circle_r.x - relay.x,2) + pow(circle_r.y - relay.y,2)) / sqrt(pow(circle_r.x - armor_now.x,2) + pow(circle_r.y - armor_now.y,2)));
        fmt::print("[buff] alpha {}\n", alpha);
        new_buff_fps.calculateFPSGlobal();
        fmt::print("[buff] fps {}\n", new_buff_fps.returnFps());
        d_t = 1.0 / (new_buff_fps.returnFps()/2.0);
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
                        fmt::print("[buff info] armor pass\n");
                    }
                    break;
                case MIN:
                    if(abs(velocity - min_velocity) < 0.1 || (last_velocity - now_velocity) * (now_velocity - velocity) < 0) {
                        T = (cv::getTickCount() - timex) / cv::getTickFrequency();
                        omega = 2 * PI / T;
                        A = (max_velocity - min_velocity) / 2.0;
                        fmt::print("[buff info] armor pass\n");
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
        fmt::print("[buff cal] {}\n",beta);
        fmt::print("[buff] beta:{}\n", beta);
        object.x = armor_now.x * cos(beta) - armor_now.y * sin(beta);
        object.y = armor_now.x * sin(beta) + armor_now.y * cos(beta);
        return object;
    }

     void buff::main_buff_checker(cv::Mat imgGray,cv::Mat img_src,new_buff::Check_Moudle moudle){
		cv::Mat /*imgGray, */imgBlur, imgCanny, imgDila, imgErode, imgDila2, imgGray2;
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        imgDila = imgGray.clone();
        imgGray2 = imgGray.clone();
        cv::imshow("act_img_11",imgGray2);

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

            //morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 1);
            //morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 1);


            /*morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 6);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 2);
            morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 2);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 1);
            */

            
            /*morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 10);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 3);
            morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 7);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 1);*/

            //morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 4);
            //morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 1);
            //GaussianBlur(imgGray, imgBlur, cv::Size(65, 65), 1, 1);
		    //Canny(imgBlur, imgCanny, 40, 120);
		    //dilate(imgCanny, imgDila, kernel);
           
        }
        /*if(moudle==ACTION_MODE)
        {
            morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 3);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 2);
        }*/
        if(moudle==CIRCLE_MODE)
        {
            morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 1);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 1);
            
/*  
            morphologyEx(imgGray, imgGray, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 2);
            morphologyEx(imgGray, imgGray, cv::MORPH_ERODE, ele_, cv::Point(-1, -1), 2);*/
            /*GaussianBlur(imgGray, imgBlur, cv::Size(65, 65), 1, 1);
		    Canny(imgBlur, imgCanny, 40, 120);
		    dilate(imgCanny, imgDila, kernel);
            cv::imshow("cir_img",imgDila);*/
        }

        GaussianBlur(imgGray, imgBlur, cv::Size(65, 65), 1, 1);
		//Canny(imgBlur, imgCanny, 40, 120);
		//dilate(imgCanny, imgDila, kernel);
        
        cv::imshow("act_img",imgDila);
//灰度->高斯滤波->Canny边缘算法->膨胀
        //morphologyEx(imgGray2, imgGray2, cv::MORPH_DILATE, ele_, cv::Point(-1, -1), 1);
		buff::getcontour(imgDila,imgGray2,img_src,moudle);
#ifndef RELEASE
	    //imshow("Image", imgGray);
        //imshow("Image2", imgDila);
        //imshow("detect inaction", img_src);
#endif
	//imshow("Image Gray", imgGray);
	//imshow("Image Blur", imgBlur);
	//imshow("Image Canny", imgCanny);
	//imshow("Image dila", imgDila);
    }

    cv::Point2f buff::calculateCircleR(cv::Point2f P1,cv::Point2f P2,cv::Point2f P3){
        float a,b,c,d,e,f;
        cv::Point2f cir;
        a=2*(P2.x-P1.x);
        b=2*(P2.y-P1.y);
        c=P2.x*P2.x+P2.y*P2.y-P1.x*P1.x-P1.y*P1.y;
        d=2*(P3.x-P2.x);
        e=2*(P3.y-P2.y);
        f=P3.x*P3.x+P3.y*P3.y-P2.x*P2.x-P2.y*P2.y;
        cir.x=(b*f-e*c)/(b*d-e*a);
        cir.y=(d*c-a*f)/(b*d-e*a);
        return cir;
    }

    cv::Point2f buff::returnCircleR(){
        return circles;
    }

    double buff::returnDistance(cv::Point2f x,cv::Point2f y){
        return sqrt((x.x-y.x)*(x.x-y.x)+(x.y-y.y)*(x.y-y.y));
    }

    cv::RotatedRect buff::returnArmorRect() {
        fmt::print("[obj rect] out {}\n",object_rects.size.height);
        return object_rects;
    }

    bool buff::returnCenterDistance(){
        return center_dist;
    }
}
