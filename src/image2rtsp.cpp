#include <string>
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

	init_count = 0;
	num_rgb = 0;
	appsrc_rgb = NULL;
	num_ir = 0;
	appsrc_ir = NULL;

	video_mainloop_start();
	rtsp_server = rtsp_server_create();

	url_connected("/rgb");
	pipeline_rgb = video_start((char*)"appsrc is-live=true do-timestamp=true ! ffmpegcolorspace ! ducatih264enc rate-preset=low-delay level=20 ! h264parse ! rtph264pay config-interval=1 ! udpsink host=127.0.0.1 port=5001", &appsrc_rgb, (char*)"/rgb", (char*)"video/x-raw-rgb,bpp=24,depth=24,endianness=4321,red_mask=0x00ff0000,green_mask=0x0000ff00,blue_mask=0x000000ff,width=320,height=240,framerate=10/1", 5001);

	while(init_count == 0) {
		sleep(1);
	}

	url_connected("/ir");
	pipeline_ir = video_start((char*)"appsrc is-live=true do-timestamp=true ! bayer2rgb ! ffmpegcolorspace ! ducatih264enc rate-preset=low-delay level=20 ! h264parse ! rtph264pay config-interval=1 ! udpsink host=127.0.0.1 port=5002", &appsrc_ir, (char*)"/ir", (char*)"video/x-raw-bayer,format=(string)grbg,width=320,height=240,framerate=10/1", 5002);
}

Image2RTSPNodelet::~Image2RTSPNodelet() {
	video_stop(pipeline_rgb, appsrc_rgb);
	video_stop(pipeline_ir, appsrc_ir);
}


void Image2RTSPNodelet::rgbCallback(const sensor_msgs::Image::ConstPtr& msg) {
	GstBuffer *buf;

	if (appsrc_rgb != NULL) {
		buf = gst_buffer_new();
		buf->size = msg->step * msg->height;
		buf->data = (guint8*)&msg->data[0];
		gst_app_src_push_buffer(appsrc_rgb, buf);
	}
}

void Image2RTSPNodelet::irCallback(const sensor_msgs::Image::ConstPtr& msg) {
	GstBuffer *buf;

	if (appsrc_ir != NULL) {
		buf = gst_buffer_new();
		buf->size = msg->step * msg->height;
		buf->data = (guint8*)&msg->data[0];
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
		}
	} else if (url == "/ir") {
		if (num_ir > 0) num_ir--;
		if (num_ir == 0) {
			sub_ir.shutdown();
		}
	}
}

PLUGINLIB_EXPORT_CLASS(image2rtsp::Image2RTSPNodelet, nodelet::Nodelet)
