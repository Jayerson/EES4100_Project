#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <modbus-tcp.h>

#include "file_ops.h"

#define LISTEN_BACKLOG 1

/* Hack:
 * libmodbus doesn't expose the context structure which is a problem for a
 * multithreaded server, as read/write operations rely on a file descriptor that
 * is written to by modbus_tcp_accept(). If multiple connections are created,
 * the file descriptor will be written over. Define a structure here to allow
 * sizeof() to determine how large modbus_t is */
struct ctx_data {
    int slave;
    int s; /* Modified by modbus_tcp_accept() */
    int debug;
    int error_recovery;
    struct timeval response_timeout;
    struct timeval byte_timeout;
    const void *backend;
    void *backend_data;
};

static modbus_mapping_t *mb_mapping;

static void *connection(void *arg) {
    modbus_t *ctx = (modbus_t *) arg;
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    int bytes;
    uint16_t device;

    while (1) {
	bytes = modbus_receive(ctx, query);
	if (bytes > 0) {
	    /* More hacking: manually extract the modbus query address so that
	     * we can use it to update the holding registers. The project
	     * requires that the modus address match the BACnet device no */
	    device = (query[8] << 8) + query[9];
	    printf("Request for device %i\n", device);
	    file_update_regs(mb_mapping->tab_registers, device);
	    modbus_reply(ctx, query, bytes, mb_mapping);
	}

	/* Other end has disconnected */
	if (bytes == -1) break;
    }

    free(ctx);
    return arg;
}

int main(int argc, char *arg[]) {
    modbus_t *ctx, *dup_ctx;
    int server_fd;
    pthread_t tcp_thread;

    file_read_random_data(RANDOM_DATA_POOL);

    if ((ctx = modbus_new_tcp("SERVER", MODBUS_TCP_DEFAULT_PORT)) == NULL) {
	printf("Failied to initialise modbus\n");
	return -1;
    }

    mb_mapping = modbus_mapping_new(0, 0, file_get_highest_channel(), 0);

listen:
    if ((server_fd = modbus_tcp_listen(ctx, LISTEN_BACKLOG)) < 0) {
	printf("Failed to initiate modbus_tcp server, %i\n", server_fd);
	printf("Root permissions are needed to open ports below 1024\n");
	printf("Currently configured to open port %i\n",
			MODBUS_TCP_DEFAULT_PORT);
	modbus_free(ctx);
	return -1;
    }

    while (1) {
	modbus_set_debug(ctx, TRUE);
	if (modbus_tcp_accept(ctx, &server_fd) < 0) {
	    /* libmodbus closes the supplied socket if accept fails - this may
	     * be making new connections impossible */
	    printf("Accept failed \n");
	    goto listen;
	}
	modbus_set_debug(ctx, FALSE);
	dup_ctx = malloc(sizeof(struct ctx_data));
	memcpy(dup_ctx, ctx, sizeof(struct ctx_data));
	pthread_create(&tcp_thread, 0, connection, dup_ctx);
    }

    modbus_mapping_free(mb_mapping);
    close(server_fd);
    modbus_free(ctx);

    return 0;
}
