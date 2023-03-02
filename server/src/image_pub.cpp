#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <DjiRtspImageSource.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

static inline int64_t now()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

static int write_data_to_file(const char* name, uint8_t* data, int size)
{
	FILE* fd = fopen(name, "wb");
	if(fd)
	{
		int w = (int)fwrite(data, 1, size, fd);
		fclose(fd);
		return w;
	}
	else
	{
		return -1;
	}
}

char rtsp_url[] = "rtsp://127.0.0.1:8554/live";
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_pub");
    ros::NodeHandle nh;
    // 创建图像发布者
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/image",30);
    ros::Publisher count_pub = nh.advertise<std_msgs::String>("/image_count",30);

    if(argc < 1) return -1;
	if(argc == 1) 
	{
		std::cout << "Usage : " << argv[0] << " <url>" << std::endl;
	//	return -1;
	}
	int64_t ts = now();

    // uint8_t* rgbDatabuffer=new uint8_t[640 * 640 * 3];;
    // uint8_t widthbuffer=0;
    // uint8_t heightbuffer=0;
    // cv::Mat resize_img;
    // sensor_msgs::ImagePtr msg;

    DjiRtspImageSource service(rtsp_url);
    service.setImageCallback(nullptr, [&ts,image_pub,count_pub](void* handler, uint8_t* frmdata, int frmsize, int width, int height, int pixfmt) -> void {
        printf("Image %d@%p  --  %dx%d -- %d\n", frmsize, frmdata, width, height, pixfmt);
        if(frmdata)
        {
            int64_t t=now();
            if(t-ts>500)
            {
                ts=t;
                static int image_count=0;
                uint8_t* rgbData =new uint8_t[width * height * 3];
                DjiRtspImageSource::FrameDataToRGB(frmdata, frmsize, width, height, pixfmt, rgbData);
                // cv::Mat frame(height, width, CV_8UC3, rgbData);  // 图像是RGB格式
                // cv::Mat resize_img;
                // cv::resize(frame, resize_img, cv::Size(640, 640));
                // sensor_msgs::ImagePtr local_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resize_img).toImageMsg();
                // msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resize_img).toImageMsg();
                // msg=local_msg;
                // image_count++;
                // 创建消息并发布
                std_msgs::String count_msg;
                std::stringstream ss;
                ss<<"image is sended"<<image_count;
                count_msg.data=ss.str();
                count_pub.publish(count_msg);
                image_count++;
                printf("image is sended:%d\n",image_count);
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(rgbData))).toImageMsg();
                msg->header.stamp = ros::Time::now();
                // // 发布图像消息
                image_pub.publish(msg);
               
                delete[] rgbData;   
            }
        }  
        else{
            printf("frmdata is empty\n");
        }
    });
    service.start();

    while(ros::ok())
    {      
        // // 创建 cv::Mat 对象
        // cv::Mat frame(height, width, CV_8UC3, rgbDatabuffer);  // 图像是RGB格式
        // // 缩放到640x640的分辨率
        // cv::Mat resize_img;
        // cv::resize(frame, resize_img, cv::Size(640, 640));
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resize_img).toImageMsg();
        // msg->header.stamp = ros::Time::now();
        // // 发布图像消息
        // image_pub.publish(msg);
        ros::spinOnce();
    }

    // 释放资源
    service.stop();
	std::cout << "done." << std::endl;

    return 0;
}
 // image_count++;
// // 创建消息并发布
// std_msgs::String count_msg;
// std::stringstream ss;
// ss<<"image is sended"<<image_count;
// count_msg.data=ss.str();
// count_pub.publish(count_msg);
// image_count++;
// printf("image is sended:%d\n",image_count);