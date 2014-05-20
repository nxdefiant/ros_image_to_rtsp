#ifndef IMAGE_TO_RTSP_H
#define IMAGE_TO_RTSP_H

namespace image2rtsp {
	class Image2RTSPNodelet : public nodelet::Nodelet {
		public:
			GstRTSPServer *rtsp_server;
			void onInit();
			void url_connected(std::string url);
			void url_disconnected(std::string url);
			void print_info(char *s);

		private:
			ros::Subscriber sub_rgb, sub_ir;
			GstAppSrc *appsrc_rgb, *appsrc_ir;
			int num_rgb, num_ir;

			void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
			void irCallback(const sensor_msgs::Image::ConstPtr& msg);
			void video_mainloop_start();
			void rtsp_server_add_url(const char *url, const char *sPipeline, GstElement **appsrc);
			GstRTSPServer *rtsp_server_create();
			void set_appsrc_rgb(GstAppSrc *appsrc);
			void set_appsrc_ir(GstAppSrc *appsrc);
	};
}

#endif
