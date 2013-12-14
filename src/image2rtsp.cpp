#include <string>
#include <stdio.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include <image2rtsp.h>


using namespace std;
using namespace image2rtsp;

void Image2RTSPNodelet::onInit() {
	NODELET_DEBUG("Initializing image2rtsp nodelet...");

	num_rgb = 0;
	appsrc_rgb = NULL;
	num_ir = 0;
	appsrc_ir = NULL;

	video_mainloop_start();
	rtsp_server = rtsp_server_create();

	rtsp_server_add_url((char*)"/rgb", (char*)"( appsrc name=imagesrc is-live=true do-timestamp=true caps=video/x-raw-rgb,bpp=24,depth=24,endianness=4321,red_mask=0x00ff0000,green_mask=0x0000ff00,blue_mask=0x000000ff,width=320,height=240,framerate=10/1 ! ffmpegcolorspace ! ducatih264enc rate-preset=low-delay level=20 ! h264parse ! rtph264pay pt=96 name=pay0 config-interval=1 )", (GstElement **)&appsrc_rgb);

	rtsp_server_add_url((char*)"/ir", (char*)"( appsrc name=imagesrc is-live=true do-timestamp=true caps=video/x-raw-bayer,format=(string)grbg,width=320,height=240,framerate=10/1 ! bayer2rgb ! ffmpegcolorspace ! x264enc tune=fastdecode speed-preset=slow bitrate=4096 ! h264parse ! rtph264pay pt=96 name=pay0 config-interval=1 )", (GstElement **)&appsrc_ir);
}


void Image2RTSPNodelet::rgbCallback(const sensor_msgs::Image::ConstPtr& msg) {
	GstBuffer *buf;

	if (appsrc_rgb != NULL) {
		buf = gst_buffer_new_and_alloc(msg->step*msg->height);
		memcpy(buf->data, &msg->data[0], msg->step*msg->height);
		gst_app_src_push_buffer(appsrc_rgb, buf);
	}
}

void Image2RTSPNodelet::irCallback(const sensor_msgs::Image::ConstPtr& msg) {
	GstBuffer *buf;

	if (appsrc_ir != NULL) {
		buf = gst_buffer_new_and_alloc(msg->step*msg->height);
		memcpy(buf->data, &msg->data[0], msg->step*msg->height);
		gst_app_src_push_buffer(appsrc_ir, buf);
	}
}

void Image2RTSPNodelet::url_connected(string url) {
	printf("Client connected: %s\n", url.c_str());

	if (url == "/rgb") {
		if (num_rgb == 0) {
			ros::NodeHandle& node = getPrivateNodeHandle();
			sub_rgb = node.subscribe("/camera/rgb/image", 10, &Image2RTSPNodelet::rgbCallback, this);
		}
		num_rgb++;
	} else if (url == "/ir") {
		if (num_ir == 0) {
			ros::NodeHandle& node = getPrivateNodeHandle();
			sub_ir = node.subscribe("/camera/ir/image", 10, &Image2RTSPNodelet::irCallback, this);
		}
		num_ir++;
	}
}

void Image2RTSPNodelet::url_disconnected(string url) {
	printf("Client disconnected: %s\n", url.c_str());

	if (url == "/rgb") {
		if (num_rgb > 0) num_rgb--;
		if (num_rgb == 0) {
			sub_rgb.shutdown();
			appsrc_rgb = NULL;
		}
	} else if (url == "/ir") {
		if (num_ir > 0) num_ir--;
		if (num_ir == 0) {
			sub_ir.shutdown();
			appsrc_ir = NULL;
		}
	}
}

PLUGINLIB_EXPORT_CLASS(image2rtsp::Image2RTSPNodelet, nodelet::Nodelet)
