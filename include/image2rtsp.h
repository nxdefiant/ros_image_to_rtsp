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
			void print_error(char *s);

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

// From rtsp-client.c
// There is currently no clean way to get the path
struct _GstRTSPClientPrivate
{
	GMutex lock;                  /* protects everything else */
	GMutex send_lock;
	GMutex watch_lock;
	GstRTSPConnection *connection;
	GstRTSPWatch *watch;
	GMainContext *watch_context;
	guint close_seq;
	gchar *server_ip;
	gboolean is_ipv6;

	GstRTSPClientSendFunc send_func;      /* protected by send_lock */
	gpointer send_data;           /* protected by send_lock */
	GDestroyNotify send_notify;   /* protected by send_lock */

	GstRTSPSessionPool *session_pool;
	gulong session_removed_id;
	GstRTSPMountPoints *mount_points;
	GstRTSPAuth *auth;
	GstRTSPThreadPool *thread_pool;

	/* used to cache the media in the last requested DESCRIBE so that
	 * we can pick it up in the next SETUP immediately */
	gchar *path;
	GstRTSPMedia *media;

	GHashTable *transports;
	GList *sessions;
	guint sessions_cookie;

	gboolean drop_backlog;
};

#endif
