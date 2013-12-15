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
	string mountpoint_1, mountpoint_2;
	string pipeline_1, pipeline_2;

	NODELET_DEBUG("Initializing image2rtsp nodelet...");

	num_rgb = 0;
	appsrc_rgb = NULL;
	num_ir = 0;
	appsrc_ir = NULL;
	ros::NodeHandle& node = getPrivateNodeHandle();

	video_mainloop_start();
	rtsp_server = rtsp_server_create();

	node.getParam("mountpoint_1", mountpoint_1);
	node.getParam("mountpoint_2", mountpoint_2);
	node.getParam("pipeline_1", pipeline_1);
	node.getParam("pipeline_2", pipeline_2);

	rtsp_server_add_url(mountpoint_1.c_str(), pipeline_1.c_str(), (GstElement **)&appsrc_rgb);
	rtsp_server_add_url(mountpoint_2.c_str(), pipeline_2.c_str(), (GstElement **)&appsrc_ir);
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
	string topic;

	ROS_INFO("Client connected: %s\n", url.c_str());

	if (url == "/rgb") {
		if (num_rgb == 0) {
			ros::NodeHandle& node = getPrivateNodeHandle();
			node.getParam("topic_1", topic);
			sub_rgb = node.subscribe(topic, 10, &Image2RTSPNodelet::rgbCallback, this);
		}
		num_rgb++;
	} else if (url == "/ir") {
		if (num_ir == 0) {
			ros::NodeHandle& node = getPrivateNodeHandle();
			node.getParam("topic_2", topic);
			sub_ir = node.subscribe(topic, 10, &Image2RTSPNodelet::irCallback, this);
		}
		num_ir++;
	}
}

void Image2RTSPNodelet::url_disconnected(string url) {
	ROS_INFO("Client disconnected: %s\n", url.c_str());

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
