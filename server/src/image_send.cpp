#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Image.h"

// #include <detection_msgs/control_msg.h>
// #include <detection_msgs/Room.h>

#include <iostream>
#include <stdio.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <arpa/inet.h> // For UDP
#include <sstream>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <DjiRtspImageSource.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// 使用命名空间，避免重复使用 std::
using namespace std;

//图像传输控制
int image_status = 0 ;				//不使用
int64_t goods_position[3] ={0,0,0};
int send_num = 0;					//已发送图片数量
int send_status = 0;				//图像发送标志位

// // 定义共享队列和同步机制
// std::mutex mtx; // 用于同步队列访问的互斥锁
// std::queue<Image> imageQueue; // 图像队列
// std::condition_variable waitflag; // 条件变量，用于等待队列中有数据

//ROS define
ros::Subscriber control_msg_sub;
ros::Subscriber pc_sub;
ros::Publisher end_pub;
ros::Publisher img_pub;
ros::Publisher april_img_pub;
ros::Publisher room_pub;


static inline int64_t now()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

char rtsp_url[] = "rtsp://127.0.0.1:8554/live";

int main(int argc, char** argv) {

	ros::init(argc, argv, "image_send");
    ros::NodeHandle n;

	int64_t delay_time = 0;
	
	cout << "输入图像发送延迟时间" << endl;
	cin >>  delay_time ;
    

    img_pub = n.advertise<sensor_msgs::Image>("uav_img", 10);
	april_img_pub = n.advertise<sensor_msgs::Image>("uav_april_img", 10);


	ros::Rate loop_rate(10); // 每10秒循环一次

    int64_t ts = now();
	// int64_t ts = 0;
    DjiRtspImageSource service(rtsp_url);

	static int send_status = 0;

	n.setParam("uav_img_status",0);
	n.setParam("uav_img_service_status",0);

	service.setImageCallback(nullptr, [&ts,&img_pub,&april_img_pub,n,&send_status,&delay_time,&send_num](void* handler, uint8_t* frmdata, int frmsize, int width, int height, int pixfmt) -> void {

		n.getParam("uav_img_status",send_status);
		
		static int image_num = 0;
		if(send_status == 1){
			image_num = 3;
		}
		else if(send_status == 2){
			image_num = 1;
		}
		else {
			image_num = 0;
		}

		if(send_status != 0){

			if(frmdata){

				int64_t t = now();
				// static int send_num = 0;

				if(t - ts > delay_time){
					send_num ++;
					cout << "send_num :" <<send_num << endl;																			

					char name[64];																																																																																																			 
					static int counter = 0;
					printf("Image %d@%p  --  %dx%d -- %d\n", frmsize, frmdata, width, height, pixfmt);
					sprintf(name, "pictures/%dx%d-%d_%d.jpg", width, height, pixfmt, ++counter);
					uint8_t* rgbData = new uint8_t[width * height * 3];

					// DjiRtspImageSource::FrameDataToRGB(frmdata, frmsize, width, height, pixfmt, rgbData);
					int result = DjiRtspImageSource::FrameDataToRGB(frmdata, frmsize, width, height, pixfmt, rgbData);
					
					cv::Mat bgr_img(height, width, CV_8UC3, rgbData);
					
					//resize插补
					cv::Mat resize_img;
					cv::resize(bgr_img, resize_img, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);//缩放倍数：2.8
					sensor_msgs::ImagePtr img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resize_img).toImageMsg();

					if(send_status == 1){
						cout << "发送人脸图像" << endl;
						img_pub.publish(img);
					}
					else if(send_status == 2){
						cout << "发送April图像" << endl;
						april_img_pub.publish(img);
					}
					else{
						cout << "未知指令，发送失败" << endl;
					}

					//连发
					if(send_num == image_num){
						// cout << "image receive error" << endl;
						n.setParam("uav_img_status",0);
						// cout << "img_end_msg is send " << endl;
						send_num = 0;
					}
					//	
					
					// cout << "image is send" << endl;

					delete[] rgbData;
					ts = now();
				
				}
			}
			else{
				cout << "frmaedata is error " << endl;
			}
		}
		
	});
	// service.start();
	// for(;;)
	// //for(int i=0; i<30; i++)
	// {
	// 	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	// }
	// service.stop();
	// std::cout << "done." << std::endl;

	while(ros::ok()){

		n.getParam("uav_img_status",send_status);
		// cout << send_status << endl;

		if(send_status != 0){
			cout << "img_send_service start" << endl;
			service.start();
			n.setParam("uav_img_service_status",1);
			for(;;)
			//for(int i=0; i<30; i++)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				n.getParam("uav_img_status",send_status);
				if(send_status == 0){
					break;
				}
			}
			service.stop();
			ros::Duration(1.5).sleep();
			n.setParam("uav_img_service_status",0);
			cout << "img_send_service stop" << endl;
		}
		
		loop_rate.sleep();
		ros::spinOnce();
	}

    return 0;
}

// static int write_data_to_file(const char* name, uint8_t* data, int size)
// {
// 	FILE* fd = fopen(name, "wb");
// 	if(fd)
// 	{
// 		int w = (int)fwrite(data, 1, size, fd);
// 		fclose(fd);
// 		return w;
// 	}
// 	else
// 	{
// 		return -1;
// 	}
// }

// // 图像解码与发送（可用）
// void DecodeandsendThread(const string &ip, int port) {

// 	//UDP初始化
// 	int sockfd = socket(AF_INET, SOCK_DGRAM, 0); // 创建UDP套接字
//     if (sockfd < 0) {
//         cerr << "Could not create socket" << endl;
//         return;
//     }

//     struct sockaddr_in serverAddr;
//     memset(&serverAddr, 0, sizeof(serverAddr));
//     serverAddr.sin_family = AF_INET;
//     serverAddr.sin_port = htons(port);
//     inet_pton(AF_INET, ip.c_str(), &serverAddr.sin_addr);

// 	static int counter = 0;


//     // int64_t ts = now();
// 	int64_t ts = 0;
//     DjiRtspImageSource service(rtsp_url);
// 	service.setImageCallback(nullptr, [&ts,sockfd,serverAddr,&goods_position](void* handler, uint8_t* frmdata, int frmsize, int width, int height, int pixfmt) -> void {
// 		if(frmdata)
// 		{
// 			int64_t t = now();
			
// 			if(t - ts > 400)
// 			// if(image_status == 1)
// 			{
// 				image_status = 0;
// 				// for(int i=0;i<10;i++){
// 					// ts = t;
// 					char name[64];
// 					static int counter = 0;
// 					printf("Image %d@%p  --  %dx%d -- %d\n", frmsize, frmdata, width, height, pixfmt);
// 					sprintf(name, "pictures/%dx%d-%d_%d.jpg", width, height, pixfmt, ++counter);
// 					uint8_t* rgbData = new uint8_t[width * height * 3];

// 					DjiRtspImageSource::FrameDataToRGB(frmdata, frmsize, width, height, pixfmt, rgbData);
					
// 					cv::Mat bgr_img(height, width, CV_8UC3, rgbData);
					
// 					//resize插补
// 					cv::Mat resize_img;
// 					cv::resize(bgr_img, resize_img, cv::Size(450, 340), 0, 0, cv::INTER_LINEAR);//缩放倍数：2.8

// 					std::vector<uchar> buf;
// 					try {
// 						//printf(".jpg start encode\n");
// 						if (!bgr_img.empty()) {
// 							cv::imencode(".jpg", resize_img, buf);

// 							int64_t t1=now();
// 							size_t size_of_goods = sizeof(goods_position);
// 							// 创建新的 vector 来存储时间戳和图像数据
// 							std::vector<uchar> send_buf;
// 							send_buf.resize(size_of_goods + buf.size());
// 							// 将时间戳复制到新的 buffer
// 							memcpy(send_buf.data(), &goods_position, size_of_goods);	
// 							// 将图像数据复制到新的 buffer
// 							memcpy(send_buf.data() + size_of_goods, buf.data(), buf.size());
// 							ts = now();
// 							ssize_t sent = sendto(sockfd, send_buf.data(), send_buf.size(), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
// 							std::cout << "image is send " << std::endl;
// 							cout << "position is :" << goods_position[0] << "\t" << goods_position[1] << endl; 

// 							// printf(".jpg is encode\n");
// 							cout<<buf.size()<<endl;
// 							//std::cout << bgr_img << std::endl;
// 						}
// 					} catch (cv::Exception &e) {
// 						cerr << "Error encoding image: " << e.what() << endl;
// 					}
// 					delete[] rgbData;
// 				// }
			
// 			}

// 		}
// 	});

// 	service.start();
// 	for(;;)
// 	//for(int i=0; i<30; i++)
// 	{
// 		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
// 	}
// 	service.stop();
// 	std::cout << "done." << std::endl;

// }


// void control_msg_callback(const detection_msgs::control_msg::ConstPtr& msg){
// 	cout << "position_msg is receive" << endl;
// 	if(msg->image_status == 1 || msg->image_status == 2){
// 		//开始发送图像
// 		cout << "Start send goods_msg" << endl;
// 		send_status = 1;
// 		goods_position[0] = msg->goods_x;
// 		goods_position[1] = msg->goods_y;
// 		goods_position[2] = 0;
// 		cout << "goods_x:" << goods_position[0] << endl;
// 		cout << "goods_y:" << goods_position[1] << endl;
// 	}
// 	else if(msg->image_status ==2){
// 		//开始发送图像
// 		cout << "Start send AprilTag_msg" << endl;
// 		send_status = 1;
// 		goods_position[0] = 0;
// 		goods_position[1] = 0;
// 		goods_position[2] = 0;
// 	}
// }

// void pc_callback(const std_msgs::Int32::ConstPtr& msg){
// 	if(msg->data == 1){
// 		//结束图像发送
// 		send_status = 0;
// 		//图像发送结束标志
// 		std_msgs::Int32 end_msg;
// 		end_msg.data = 1;
// 		end_pub.publish(end_msg);

// 		cout << "image is enough, send end " << endl;
// 	}
// 	else{
// 		cout << "pc_msg is error" << endl;
// 	}
// }


		// if(frmdata)
		// {
		// 	int64_t t = now();

		// 	static int send_num = 0;

		// 	if(t - ts > delay_time && (send_status == 1 || send_status ==2))
		// 	{
		// 		send_num ++;
		// 		cout << "send_num :" <<send_num << endl;

		// 		if(send_num == 5){
		// 			// cout << "image receive error" << endl;

		// 			n.setParam("uav_img_status",0);
		// 			// send_status = 0;
		// 			cout << "img_end_msg is send " << endl;

		// 			// continue;

		// 			send_num = 0;
		// 			// send_status = 0;
		// 		}

		// 		char name[64];
		// 		static int counter = 0;
		// 		printf("Image %d@%p  --  %dx%d -- %d\n", frmsize, frmdata, width, height, pixfmt);
		// 		sprintf(name, "pictures/%dx%d-%d_%d.jpg", width, height, pixfmt, ++counter);
		// 		uint8_t* rgbData = new uint8_t[width * height * 3];

		// 		DjiRtspImageSource::FrameDataToRGB(frmdata, frmsize, width, height, pixfmt, rgbData);
				
		// 		cv::Mat bgr_img(height, width, CV_8UC3, rgbData);
				
		// 		//resize插补
		// 		cv::Mat resize_img;
		// 		cv::resize(bgr_img, resize_img, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);//缩放倍数：2.8
		// 		sensor_msgs::ImagePtr img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resize_img).toImageMsg();

		// 		// sensor_msgs::Image::SharedPtr img_shared = std::make_shared<sensor_msgs::Image>(img);

		// 		// detection_msgs::Room room ;
		// 		// room.Img = img_shared;
		// 		// room.Img = img.get();
		// 		// room.Row = goods_position[0];
		// 		// room.Num = goods_position[1];

		// 		// room_pub.publish(room);
		// 		img_pub.publish(img);

		// 		// cout << "img is send to pc: "<< goods_position[0] << "\t" << goods_position[1] << endl;

				
				
		// 		// std::vector<uchar> buf;
		// 		// try {
		// 		// 	//printf(".jpg start encode\n");
		// 		// 	if (!bgr_img.empty()) {
		// 		// 		cv::imencode(".jpg", resize_img, buf);

		// 		// 		int64_t t1=now();
		// 		// 		size_t size_of_goods = sizeof(goods_position);
		// 		// 		// 创建新的 vector 来存储时间戳和图像数据
		// 		// 		std::vector<uchar> send_buf;
		// 		// 		send_buf.resize(size_of_goods + buf.size());
		// 		// 		// 将时间戳复制到新的 buffer
		// 		// 		memcpy(send_buf.data(), &goods_position, size_of_goods);	
		// 		// 		// 将图像数据复制到新的 buffer
		// 		// 		memcpy(send_buf.data() + size_of_goods, buf.data(), buf.size());
		// 		// 		ts = now();
		// 		// 		ssize_t sent = sendto(sockfd, send_buf.data(), send_buf.size(), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
		// 		// 		std::cout << "image is send " << std::endl;
		// 		// 		cout << "position is :" << goods_position[0] << "\t" << goods_position[1]  << "\t" << goods_position[2] << endl; 

		// 		// 		// printf(".jpg is encode\n");
		// 		// 		cout<<buf.size()<<endl;
		// 		// 		//std::cout << bgr_img << std::endl;
		// 		// 	}
		// 		// } catch (cv::Exception &e) {
		// 		// 	cerr << "Error encoding image: " << e.what() << endl;
		// 		// }
				
		// 		delete[] rgbData;
		// 		ts = now();
			
		// 	}
		// }

	// string ip = "192.168.199.242"; // 目标IP地址
	// string ip;
	// cout << "输入电脑端的IP地址" << endl;
	// cin >>  ip ;
	// cout << "your IP is :" << ip << endl;
	// int port = 9090; // 目标端口号
// //
// 	//UDP初始化
// 	int sockfd = socket(AF_INET, SOCK_DGRAM, 0); // 创建UDP套接字
//     // if (sockfd < 0) {
//     //     cerr << "Could not create socket" << endl;
//     //     continue;
//     // }

//     struct sockaddr_in serverAddr;
//     memset(&serverAddr, 0, sizeof(serverAddr));
//     serverAddr.sin_family = AF_INET;
//     serverAddr.sin_port = htons(port);
//     inet_pton(AF_INET, ip.c_str(), &serverAddr.sin_addr);

// 	static int counter = 0;



// std::vector<uchar> buf;
// try {
// 	//printf(".jpg start encode\n");
// 	if (!bgr_img.empty()) {
// 		cv::imencode(".jpg", resize_img, buf);

// 		int64_t t1=now();
// 		size_t size_of_goods = sizeof(goods_position);
// 		// 创建新的 vector 来存储时间戳和图像数据
// 		std::vector<uchar> send_buf;
// 		send_buf.resize(size_of_goods + buf.size());
// 		// 将时间戳复制到新的 buffer
// 		memcpy(send_buf.data(), &goods_position, size_of_goods);	
// 		// 将图像数据复制到新的 buffer
// 		memcpy(send_buf.data() + size_of_goods, buf.data(), buf.size());
// 		ts = now();
// 		ssize_t sent = sendto(sockfd, send_buf.data(), send_buf.size(), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
// 		std::cout << "image is send " << std::endl;
// 		cout << "position is :" << goods_position[0] << "\t" << goods_position[1] << endl; 

// 		// printf(".jpg is encode\n");
// 		cout<<buf.size()<<endl;
// 		//std::cout << bgr_img << std::endl;
// 	}
// } catch (cv::Exception &e) {
// 	cerr << "Error encoding image: " << e.what() << endl;
// }


// // 图像解码与发送（可用）
// void DecodeandsendThread(const string &ip, int port,int64_t delay_time) {

// 	//UDP初始化
// 	int sockfd = socket(AF_INET, SOCK_DGRAM, 0); // 创建UDP套接字
//     if (sockfd < 0) {
//         cerr << "Could not create socket" << endl;
//         return;
//     }

//     struct sockaddr_in serverAddr;
//     memset(&serverAddr, 0, sizeof(serverAddr));
//     serverAddr.sin_family = AF_INET;
//     serverAddr.sin_port = htons(port);
//     inet_pton(AF_INET, ip.c_str(), &serverAddr.sin_addr);

// 	static int counter = 0;


//     // int64_t ts = now();
// 	int64_t ts = 0;
//     DjiRtspImageSource service(rtsp_url);
// 	service.setImageCallback(nullptr, [&ts,sockfd,serverAddr,delay_time](void* handler, uint8_t* frmdata, int frmsize, int width, int height, int pixfmt) -> void {
// 		printf("Image %d@%p  --  %dx%d -- %d\n", frmsize, frmdata, width, height, pixfmt);
// 		if(frmdata)
// 		{
// 			int64_t t = now();
			
// 			if(t - ts > delay_time)
// 			{
// 				// ts = t;
// 				char name[64];
// 				static int counter = 0;
// 				sprintf(name, "pictures/%dx%d-%d_%d.jpg", width, height, pixfmt, ++counter);
// 				//if(pixfmt == 5) stbi_write_jpg(name, width, height, 3, frmdata, 80);
// 				uint8_t* rgbData = new uint8_t[width * height * 3];

// 				// std::cout << "解码前时间：\t" << '0' << std::endl;

// 				// int64_t t1=now();

// 				DjiRtspImageSource::FrameDataToRGB(frmdata, frmsize, width, height, pixfmt, rgbData);
// 				// printf("rtsp image source result is %d \r\n");

// 				// int64_t t1 = now();
// 				// std::cout << "解码后时间：\t" << t1-t <<std::endl;

// 				cv::Mat bgr_img(height, width, CV_8UC3, rgbData);
// 				// std::cout << "before_size:\t" << bgr_img.total() * 3 << std::endl;

// 				//resize插补
// 				cv::Mat resize_img;
// 				cv::resize(bgr_img, resize_img, cv::Size(450, 340), 0, 0, cv::INTER_LINEAR);//缩放倍数：2.8
// 				// std::cout << "before_size:\t" << resize_img.total() * 3 << std::endl;

// 				// int64_t t2 = now();
// 				// std::cout << "插补后时间：\t" << t2-t <<std::endl;

// 				std::vector<uchar> buf;
// 				try {
// 					//printf(".jpg start encode\n");
// 					if (!bgr_img.empty()) {
// 						cv::imencode(".jpg", resize_img, buf);

// 						int64_t t1=now();
// 						size_t size_of_t = sizeof(t1);
// 						// 创建新的 vector 来存储时间戳和图像数据
// 						std::vector<uchar> send_buf;
// 						send_buf.resize(size_of_t + buf.size());
// 						// 将时间戳复制到新的 buffer
//         				memcpy(send_buf.data(), &t1, size_of_t);	
// 						 // 将图像数据复制到新的 buffer
//         				memcpy(send_buf.data() + size_of_t, buf.data(), buf.size());
// 						ts = now();
// 						ssize_t sent = sendto(sockfd, send_buf.data(), send_buf.size(), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
// 						std::cout << "image is send " << std::endl;

// 						// printf(".jpg is encode\n");
// 						cout<<buf.size()<<endl;
// 						//std::cout << bgr_img << std::endl;
// 					}
// 				} catch (cv::Exception &e) {
// 					cerr << "Error encoding image: " << e.what() << endl;
// 					// continue; // 跳过发送，继续循环
// 				}

// 				// 发送图像数据
// 				// ssize_t sent = sendto(sockfd, buf.data(), buf.size(), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
				

// 				// else if(sent <=0){
// 				//