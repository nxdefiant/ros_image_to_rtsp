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

// From rtsp-media.c
// There is currently no clean way to get the pipeline
struct _GstRTSPMediaPrivate
{
	GMutex lock;
	GCond cond;

	/* protected by lock */
	GstRTSPPermissions *permissions;
	gboolean shared;
	gboolean suspend_mode;
	gboolean reusable;
	GstRTSPProfile profiles;
	GstRTSPLowerTrans protocols;
	gboolean reused;
	gboolean eos_shutdown;
	guint buffer_size;
	GstRTSPAddressPool *pool;
	gchar *multicast_iface;
	gboolean blocked;
	GstRTSPTransportMode transport_mode;
	gboolean stop_on_disconnect;

	GstElement *element;
	GRecMutex state_lock;         /* locking order: state lock, lock */
	GPtrArray *streams;           /* protected by lock */
	GList *dynamic;               /* protected by lock */
	GstRTSPMediaStatus status;    /* protected by lock */
	gint prepare_count;
	gint n_active;
	gboolean adding;

	/* the pipeline for the media */
	GstElement *pipeline;
	GstElement *fakesink;         /* protected by lock */
	GSource *source;
	guint id;
	GstRTSPThread *thread;

	gboolean time_provider;
	GstNetTimeProvider *nettime;

	gboolean is_live;
	gboolean seekable;
	gboolean buffering;
	GstState target_state;

	/* RTP session manager */
	GstElement *rtpbin;

	/* the range of media */
	GstRTSPTimeRange range;       /* protected by lock */
	GstClockTime range_start;
	GstClockTime range_stop;

	GList *payloads;              /* protected by lock */
	GstClockTime rtx_time;        /* protected by lock */
	guint latency;                /* protected by lock */
	GstClock *clock;              /* protected by lock */
	GstRTSPPublishClockMode publish_clock_mode;
};

#endif
