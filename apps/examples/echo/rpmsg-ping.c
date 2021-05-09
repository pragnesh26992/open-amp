/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* This is a sample demonstration application that showcases usage of rpmsg 
This application is meant to run on the remote CPU running baremetal code. 
This application echoes back data that was sent to it by the master core. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <openamp/open_amp.h>
#include <metal/alloc.h>
#include "platform_info.h"
#include "rpmsg-echo.h"

#define LPRINTF(format, ...) printf(format, ##__VA_ARGS__)
#define LPERROR(format, ...) LPRINTF("ERROR: " format, ##__VA_ARGS__)

struct _payload {
	unsigned long num;
	unsigned long size;
	unsigned char data[];
};

static int err_cnt;

#define PAYLOAD_MIN_SIZE	1

/* Globals */
static struct rpmsg_endpoint lept;
static struct _payload *i_payload;
int rnum = 0;
int no_try = 0;
static int err_cnt = 0;
static int ept_deleted = 0;

/*-----------------------------------------------------------------------------*
 *  RPMSG endpoint callbacks
 *-----------------------------------------------------------------------------*/
static int rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
			     uint32_t src, void *priv)
{
	int i;
	struct _payload *r_payload = (struct _payload *)data;

	(void)ept;
	(void)src;
	(void)priv;
	printf(" received payload number %lu of size %lu \r\n",
		r_payload->num, (unsigned long)len);

	if (r_payload->size == 0) {
		LPERROR(" Invalid size of package is received.\r\n");
		err_cnt++;
		return RPMSG_SUCCESS;
	}
	/* Validate data buffer integrity. */
	for (i = 0; i < (int)r_payload->size; i++) {
		if (r_payload->data[i] != 0xA5) {
			printf("Data corruption at index %d\r\n", i);
			err_cnt++;
			break;
		}
	}
	//rnum = r_payload->num + 1;
	rnum = 0x2;
	return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept)
{
	(void)ept;
	rpmsg_destroy_ept(&lept);
	printf("echo test: service is destroyed\r\n");
	ept_deleted = 1;
}

static void rpmsg_name_service_bind_cb(struct rpmsg_device *rdev,
				       const char *name, uint32_t dest)
{
	printf("new endpoint notification is received.\r\n");
	if (strcmp(name, RPMSG_SERVICE_NAME))
		LPERROR("Unexpected name service %s.\r\n", name);
	else
		(void)rpmsg_create_ept(&lept, rdev, RPMSG_SERVICE_NAME,
				       RPMSG_ADDR_ANY, dest,
				       rpmsg_endpoint_cb,
				       rpmsg_service_unbind);

}

/*-----------------------------------------------------------------------------*
 *  Application
 *-----------------------------------------------------------------------------*/
int app (struct rpmsg_device *rdev, void *priv)
{
	int ret;
	int i;
	int size, max_size, num_payloads;
	int expect_rnum = 0;

	printf(" 1 - Send data to remote core, retrieve the echo");
	printf(" and validate its integrity ..\r\n");

	max_size = rpmsg_virtio_get_buffer_size(rdev);
	if (max_size < 0) {
		LPERROR("No avaiable buffer size.\r\n");
		return -1;
	}
	max_size -= sizeof(struct _payload);
	num_payloads = max_size - PAYLOAD_MIN_SIZE + 1;
	printf(" num_payloads ======== %d\r\n", num_payloads);
	i_payload =
	    (struct _payload *)metal_allocate_memory(2 * sizeof(unsigned long) +
				      max_size);

	if (!i_payload) {
		LPERROR("memory allocation failed.\r\n");
		return -1;
	}

	/* Create RPMsg endpoint */
	ret = rpmsg_create_ept(&lept, rdev, RPMSG_SERVICE_NAME,
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_endpoint_cb, rpmsg_service_unbind);

	if (ret) {
		LPERROR("Failed to create RPMsg endpoint.\r\n");
		metal_free_memory(i_payload);
		return ret;
	}

	while (!is_rpmsg_ept_ready(&lept))
		platform_poll(priv);

	usleep(1000000);

#if 0
	        i_payload->num = 0;
                i_payload->size = PAYLOAD_MIN_SIZE;

		memset(&(i_payload->data[0]), 0xA5, PAYLOAD_MIN_SIZE);

              ret = rpmsg_send(&lept, i_payload,
                                 (2 * sizeof(unsigned long)) + PAYLOAD_MIN_SIZE);

		platform_poll(priv);
#endif
	printf("RPMSG endpoint is binded with remote.\r\n");
	for (i = 0, size = PAYLOAD_MIN_SIZE; i < 100; i++, size++) {
		i_payload->num = i;
		i_payload->size = size;

		/* Mark the data buffer. */
		memset(&(i_payload->data[0]), 0xA5, size);

		printf("sending payload number %lu of size %lu\r\n",
			i_payload->num,
			(unsigned long)(2 * sizeof(unsigned long)) + size);

		ret = rpmsg_send(&lept, i_payload,
				 (2 * sizeof(unsigned long)) + size);

		if (ret < 0) {
			LPERROR("Failed to send data...\r\n");
			break;
		}
		printf("echo test: sent : %lu\r\n",
			(unsigned long)(2 * sizeof(unsigned long)) + size);

		expect_rnum++;
		rnum = 0x1;
		no_try = 0x0;
		do {
			platform_poll(priv);
			no_try++;

			if (rnum == 0x2)
				break;
		} while (no_try <= 3 && !ept_deleted);

	}

	printf("**********************************\r\n");
	printf(" Test Results: Error count = %d \r\n", err_cnt);
	printf("**********************************\r\n");
	/* Destroy the RPMsg endpoint */
	rpmsg_destroy_ept(&lept);
	printf("Quitting application .. Echo test end\r\n");

	metal_free_memory(i_payload);
	return 0;
}

int main(int argc, char *argv[])
{
	void *platform;
	struct rpmsg_device *rpdev;
	int ret;

	/* Initialize platform */
	ret = platform_init(argc, argv, &platform);
	if (ret) {
		LPERROR("Failed to initialize platform.\r\n");
		ret = -1;
	} else {
		rpdev = platform_create_rpmsg_vdev(platform, 0,
						  VIRTIO_DEV_MASTER,
						  NULL,
						  rpmsg_name_service_bind_cb);
		if (!rpdev) {
			LPERROR("Failed to create rpmsg virtio device.\r\n");
			ret = -1;
		} else {
			app(rpdev, platform);
			platform_release_rpmsg_vdev(rpdev, platform);
			ret = 0;
		}
	}

	printf("Stopping application...\r\n");
	platform_cleanup(platform);

	return ret;
}

