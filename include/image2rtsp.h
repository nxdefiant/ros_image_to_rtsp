#ifndef IMAGE_TO_RTSP_H
#define IMAGE_TO_RTSP_H

namespace image2rtsp {
	class Image2RTSPNodelet : public nodelet::Nodelet {
		public:
			void onInit();
			~Image2RTSPNodelet();
			void rtsp_server_add_url(char *url, int port, char *caps);
			void url_connected(std::string url);
			void url_disconnected(std::string url);
			int init_count;

		private:
			ros::Subscriber sub_rgb, sub_ir;
			GstElement *pipeline_rgb, *pipeline_ir;
			GstAppSrc *appsrc_rgb, *appsrc_ir;
			GstRTSPServer *rtsp_server;
			int num_rgb, num_ir;

			void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
			void irCallback(const sensor_msgs::Image::ConstPtr& msg);
			void video_mainloop_start();
			GstElement *video_start(char *spipeline, GstAppSrc **appsrc, char *url, char *caps, int port);
			void video_stop(GstElement *pipeline, GstAppSrc *appsrc);
			GstRTSPServer *rtsp_server_create();
	};
}

#endif
