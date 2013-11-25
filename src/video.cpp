#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "sensor_msgs/Image.h"
#include <image2rtsp.h>

using namespace std;
using namespace image2rtsp;

/**
 * Receives asynchronous messages type GST_MESSAGE_ERROR in the main loop
 * and prints them on stderr.
 */
static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data) {
	gchar *debug;
	GError *error;

	switch (GST_MESSAGE_TYPE(msg)) {
		case GST_MESSAGE_ERROR:
			gst_message_parse_error(msg, &error, &debug);
			g_free(debug);

			printf("Error: %s\n", error->message);
			g_error_free(error);
			break;
                case GST_MESSAGE_WARNING:
                        gst_message_parse_warning(msg, &error, &debug);
                        g_free(debug);
                        printf("Warning: %s\n", error->message);
                        g_error_free(error);
                        break;
		default:
			break;
	}

	return TRUE;
}


static void *mainloop(void *arg) {
	GMainLoop *loop = g_main_loop_new(NULL, FALSE);

	g_main_loop_run(loop);

	g_main_destroy(loop);
	return NULL;
}


void Image2RTSPNodelet::video_mainloop_start() {
	pthread_t tloop;

	gst_init(NULL, NULL);
	pthread_create(&tloop, NULL, &mainloop, NULL);
}


static void client_closed(GstRTSPClient *client, Image2RTSPNodelet *nodelet) {
	nodelet->url_disconnected(client->uri->abspath);
}

static void client_options(GstRTSPClient *client, GstRTSPClientState *state, Image2RTSPNodelet *nodelet) {
	nodelet->url_connected(state->uri->abspath);
}


static void new_client(GstRTSPServer *server, GstRTSPClient *client, Image2RTSPNodelet *nodelet) {
	printf("New client\n");
	g_signal_connect(client, "options-request", G_CALLBACK(client_options), nodelet);
	g_signal_connect(client, "closed", G_CALLBACK(client_closed), nodelet);
}


GstRTSPServer *Image2RTSPNodelet::rtsp_server_create() {
	GstRTSPServer *server;

	/* create a server instance */
	server = gst_rtsp_server_new();

	/* attach the server to the default maincontext */
	gst_rtsp_server_attach(server, NULL);

	g_signal_connect(server, "client-connected", G_CALLBACK(new_client), this);

	return server;
}


void Image2RTSPNodelet::rtsp_server_add_url(char *url, int port, char *caps) {
	GstRTSPMediaMapping *mapping;
	GstRTSPMediaFactory *factory;
	char sPipeline[512];

	snprintf(sPipeline, 512, "( udpsrc port=%d caps=\"%s\" ! rtph264depay ! rtph264pay pt=96 name=pay0 config-interval=1 )", port, caps);

	/* get the mapping for this server, every server has a default mapper object
	 * that be used to map uri mount points to media factories */
	mapping = gst_rtsp_server_get_media_mapping(rtsp_server);

	/* make a media factory for a test stream. The default media factory can use
	 * gst-launch syntax to create pipelines. 
	 * any launch line works as long as it contains elements named pay%d. Each
	 * element with pay%d names will be a stream */
	factory = gst_rtsp_media_factory_new();
	gst_rtsp_media_factory_set_launch(factory, sPipeline);

	gst_rtsp_media_factory_set_shared(factory, TRUE);

	/* attach the test factory to the /test url */
	gst_rtsp_media_mapping_add_factory(mapping, url, factory);

	/* don't need the ref to the mapper anymore */
	g_object_unref(mapping);
}


static void notify_caps(GstPad *pad, GParamSpec *pspec, Image2RTSPNodelet *nodelet) {
	GstCaps *caps;
	char *pad_caps;
	char escaped_caps[320];
	char *c_src;
	char *c_dest;
	char *url = (char *)g_object_get_data(G_OBJECT(pad), "param_url");
	int port = (int)g_object_get_data(G_OBJECT(pad), "param_port");

#if GST_VERSION_MAJOR > 0
	caps = gst_pad_get_current_caps(pad);
#else
	caps = gst_pad_get_negotiated_caps(pad);
#endif
	pad_caps = gst_caps_to_string(caps);

	for(c_src=pad_caps, c_dest=escaped_caps;; c_src++) {
		// Escape some characters
		if (*c_src == '\\') {
			*c_dest = '\\';
			c_dest++;
		} else if (*c_src == '"') {
			*c_dest = '\\';
			c_dest++;
		}
		
		*c_dest = *c_src;
		c_dest++;

		// EOS
		if (*c_src == '\0') break;
		else if(c_dest-escaped_caps >= 320) {
			fprintf(stderr, "Fixme: Caps string to short: Need %d bytes\n", strlen(pad_caps));
			exit(1);
		}
	}

	printf("Got caps %s\n", pad_caps);
	nodelet->rtsp_server_add_url(url, port, escaped_caps);
	nodelet->url_disconnected(url);
	nodelet->init_count++;
}


GstElement *Image2RTSPNodelet::video_start(char *spipeline, GstAppSrc **appsrc, char *url, char *caps, int port) {
	GError *error=NULL;
	GstBus *bus;
	GstElement *udpsink;
	GstStateChangeReturn ret;
	GstElement *pipeline;

	pipeline = gst_parse_launch(spipeline, &error);

	if (!pipeline) {
		printf("Pipeline could not be created. Exiting.\n");
		return NULL;
	} else if (error != NULL) {
		printf("Error: %s\n", error->message);
		g_error_free(error);
		return NULL;
	}

	udpsink = gst_bin_get_by_name(GST_BIN(pipeline), "udpsink0");
	if (udpsink) {
		GstPad *pad = gst_element_get_static_pad(udpsink, "sink");
		g_object_set_data(G_OBJECT(pad), "param_url", url);
		g_object_set_data(G_OBJECT(pad), "param_port", (void *)port);

		g_signal_connect(pad, "notify::caps", G_CALLBACK(notify_caps), this);
		gst_object_unref(udpsink);
	}

	// add error handler	
	bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
	gst_bus_add_watch(bus, bus_call, NULL);
	gst_object_unref(bus);

        *appsrc = (GstAppSrc *)gst_bin_get_by_name(GST_BIN(pipeline), "appsrc0");
        if (*appsrc != NULL) {
                g_object_set(G_OBJECT(*appsrc), "caps", gst_caps_from_string(caps), NULL);
        }


	ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
	if (ret == GST_STATE_CHANGE_FAILURE) {
		printf("Unable to set the pipeline to the playing state.\n");
		gst_element_set_state(pipeline, GST_STATE_NULL);
	}

	return pipeline;
}


void Image2RTSPNodelet::video_stop(GstElement *pipeline, GstAppSrc *appsrc) {
	gst_element_set_state(pipeline, GST_STATE_NULL);
	gst_object_unref(GST_OBJECT(pipeline));
	gst_object_unref(GST_OBJECT(appsrc));
}
