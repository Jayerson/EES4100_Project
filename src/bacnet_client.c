#include <stdio.h>

#include <libbacnet/address.h>
#include <libbacnet/device.h>
#include <libbacnet/handlers.h>
#include <libbacnet/datalink.h>
#include <libbacnet/bvlc.h>
#include <libbacnet/client.h>
#include <libbacnet/txbuf.h>
#include <libbacnet/tsm.h>
#include <libbacnet/bactext.h>
#include "bacnet_namespace.h"

#include "list.h"
#include "file_ops.h"

#define BACNET_PORT		    0xBAC0
#define BACNET_INTERFACE	    "lo"
#define BACNET_DATALINK_TYPE	    "bvlc"
#define BACNET_SELECT_TIMEOUT_MS    1	    /* ms */

#define RUN_AS_BBMD_CLIENT	    0

#if RUN_AS_BBMD_CLIENT
#define BACNET_BBMD_PORT	    0xBAC0
#define BACNET_BBMD_ADDRESS	    "127.0.0.1"
#define BACNET_BBMD_TTL		    90
#endif

#define debug 0

static pthread_mutex_t timer_lock = PTHREAD_MUTEX_INITIALIZER;
static int instance_no;
static LIST_HEAD(devices);

typedef struct device_obj_s device_obj;
struct device_obj_s {
    int			found;
    uint32_t		device_id;
    BACNET_ADDRESS	bacnet_address;
    struct list_head	instances;

    list_entry		devices;
};

typedef struct instance_obj_s instance_obj;
struct instance_obj_s {
    device_obj		*parent;

    int			instance_no;
    BACNET_OBJECT_TYPE	object_type;
    BACNET_PROPERTY_ID	object_property;
    uint32_t		array_index;
    int			invoke_id;

    /* Retrieve data as integers (actually floats) and convert them to uint16_t
     * for comparison with modbus data */
    uint16_t		*needle;
    uint16_t		*haystack;
    size_t		num_words;
    int			match;
    uint16_t		last_value;

    list_entry		instances;
};

static bacnet_object_functions_t client_objects[] = {
    {bacnet_OBJECT_DEVICE,
	    NULL,
	    bacnet_Device_Count,
	    bacnet_Device_Index_To_Instance,
	    bacnet_Device_Valid_Object_Instance_Number,
	    bacnet_Device_Object_Name,
	    bacnet_Device_Read_Property_Local,
	    bacnet_Device_Write_Property_Local,
	    bacnet_Device_Property_Lists,
	    bacnet_DeviceGetRRInfo,
	    NULL, /* Iterator */
	    NULL, /* Value_Lists */
	    NULL, /* COV */
	    NULL, /* COV Clear */
	    NULL  /* Intrinsic Reporting */
    },
    {MAX_BACNET_OBJECT_TYPE}
};

static void register_with_bbmd(void) {
#if RUN_AS_BBMD_CLIENT
    bacnet_bvlc_register_with_bbmd(
	    bacnet_bip_getaddrbyname(BACNET_BBMD_ADDRESS), 
	    htons(BACNET_BBMD_PORT),
	    BACNET_BBMD_TTL);
#endif
}

static void *minute_tick(void *arg) {
    while (1) {
	pthread_mutex_lock(&timer_lock);

	/* Expire addresses once the TTL has expired */
	bacnet_address_cache_timer(60);

	/* Re-register with BBMD once BBMD TTL has expired */
	register_with_bbmd();

	/* Update addresses for notification class recipient list 
	 * Requred for INTRINSIC_REPORTING
	 * bacnet_Notification_Class_find_recipient(); */

	/* Sleep for 1 minute */
	pthread_mutex_unlock(&timer_lock);
	sleep(60);
    }
    return arg;
}

static void ping_servers(void) {
    device_obj *device;
    list_for_each_entry(device, &devices, devices) {
	if (!device->found)
	    bacnet_Send_WhoIs(device->device_id, device->device_id);
    }
}

static void *second_tick(void *arg) {
    while (1) {
	pthread_mutex_lock(&timer_lock);

	/* Keep searching for server */
	ping_servers();

	/* Invalidates stale BBMD foreign device table entries */
	bacnet_bvlc_maintenance_timer(1);

	/* Transaction state machine: Responsible for retransmissions and ack
	 * checking for confirmed services */
	bacnet_tsm_timer_milliseconds(1000);

	/* Re-enables communications after DCC_Time_Duration_Seconds
	 * Required for SERVICE_CONFIRMED_DEVICE_COMMUNICATION_CONTROL
	 * bacnet_dcc_timer_seconds(1); */

	/* State machine for load control object
	 * Required for OBJECT_LOAD_CONTROL
	 * bacnet_Load_Control_State_Machine_Handler(); */

	/* Expires any COV subscribers that have finite lifetimes
	 * Required for SERVICE_CONFIRMED_SUBSCRIBE_COV
	 * bacnet_handler_cov_timer_seconds(1); */

	/* Monitor Trend Log uLogIntervals and fetch properties
	 * Required for OBJECT_TRENDLOG
	 * bacnet_trend_log_timer(1); */
	
	/* Run [Object_Type]_Intrinsic_Reporting() for all objects in device
	 * Required for INTRINSIC_REPORTING
	 * bacnet_Device_local_reporting(); */

	/* Sleep for 1 second */
	pthread_mutex_unlock(&timer_lock);
	sleep(1);
    }
    return arg;
}

static void ack_servers(void) {
    device_obj *device;
    unsigned max_apdu;

    list_for_each_entry(device, &devices, devices) {

	if (!device->found)
	    device->found =
		bacnet_address_bind_request(
		    device->device_id,
		    &max_apdu,
		    &device->bacnet_address);
    }

}

static void ms_tick(void) {
    /* Updates change of value COV subscribers.
     * Required for SERVICE_CONFIRMED_SUBSCRIBE_COV
     * bacnet_handler_cov_task(); */

    ack_servers();
}

#define BN_UNC(service, handler) \
    bacnet_apdu_set_unconfirmed_handler(		\
		    SERVICE_UNCONFIRMED_##service,	\
		    bacnet_handler_##handler)
#define BN_CON(service, handler) \
    bacnet_apdu_set_confirmed_handler(			\
		    SERVICE_CONFIRMED_##service,	\
		    bacnet_handler_##handler)
#define BN_CON_ACK(service, handler) \
    bacnet_apdu_set_confirmed_ack_handler(		\
		    SERVICE_CONFIRMED_##service,	\
		    handler)
#define BN_ERR(service, handler) \
    bacnet_apdu_set_error_handler(			\
		    SERVICE_CONFIRMED_##service,	\
		    handler)

static device_obj *match_server(BACNET_ADDRESS *src,
			uint8_t invoke_id,
			instance_obj **instance_out) {
    device_obj *device;
    instance_obj *instance;
    list_for_each_entry(device, &devices, devices) {
	if (bacnet_address_match(&device->bacnet_address, src)) {
	    list_for_each_entry(instance, &device->instances, instances) {
		if (instance->invoke_id == invoke_id) {
		    if (instance_out)
			*instance_out = instance;
		    return device;
		}
	    }
	}
    }
    return NULL;
}

static void abort_handler(
		BACNET_ADDRESS *src,
		uint8_t invoke_id,
		uint8_t abort_reason,
		bool server) {

    device_obj *device;
    
    if (!(device = match_server(src, invoke_id, NULL))) return;

    fprintf(stderr, "BACnet Abort from server %i: %s\n",
	    device->device_id,
	    bactext_abort_reason_name(abort_reason));
    device->found = 0;
}

static void reject_handler(
		BACNET_ADDRESS *src,
		uint8_t invoke_id,
		uint8_t reject_reason) {

    device_obj *device;
    
    if (!(device = match_server(src, invoke_id, NULL))) return;

    fprintf(stderr, "BACnet Reject from server %i: %s\n",
	    device->device_id,
	    bactext_reject_reason_name(reject_reason));
    device->found = 0;
}

static void read_property_err(
		BACNET_ADDRESS *src,
		uint8_t invoke_id,
		BACNET_ERROR_CLASS error_class,
		BACNET_ERROR_CODE error_code) {

    device_obj *device;
    
    if (!(device = match_server(src, invoke_id, NULL))) return;

    fprintf(stderr, "BACnet Error from server %i: %s: %s\n",
	    device->device_id,
	    bactext_error_class_name(error_class),
	    bactext_error_code_name(error_code));
    device->found = 0;
}

static void array_tail(uint16_t data, instance_obj *instance) {
    int i;

    /* If we are making requests too often, it is possible to get ahead of the
     * bacnet_server. If so, just drop the data */
    if (data == instance->last_value) return;
    instance->last_value = data;

    /* Shift the array */
    for (i = 0; i < instance->num_words - 1; i++) {
	instance->haystack[i] = instance->haystack[i+1];
    }

    /* Place new data in the last entry of the array */
    instance->haystack[instance->num_words - 1] = data;

#if debug
    printf("Haystack (%i:%i): -------\n", instance->parent->device_id, instance->instance_no);
    for (i = 0; i < instance->num_words; i++) {
	printf("%04X ", instance->haystack[i]);
	if ((i % 8) == 7) printf("\n");
    }
    printf("\n");
    fflush(stdout);

    printf("Needle (%i:%i): -------\n", instance->parent->device_id, instance->instance_no);
    for (i = 0; i < instance->num_words; i++) {
	printf("%04X ", instance->needle[i]);
	if ((i % 8) == 7) printf("\n");
    }
    printf("\n");
    fflush(stdout);
#endif

    if (!memcmp(instance->needle, 
		instance->haystack,
		instance->num_words * sizeof(uint16_t))) {
	instance->match = 1;
	printf("Successful match for device %i, instance %i\n",
			instance->parent->device_id, instance->instance_no);
	fflush(stdout);
    }
}

static void read_property_ack(
		uint8_t *service_request,
		uint16_t service_len,
		BACNET_ADDRESS *src,
		BACNET_CONFIRMED_SERVICE_ACK_DATA *service_data) {
    int len;
    device_obj *device;
    instance_obj *instance;
    BACNET_READ_PROPERTY_DATA data;
    BACNET_APPLICATION_DATA_VALUE value;
    uint16_t data_16;
    
    if (!(device = match_server(src,
		    service_data->invoke_id, &instance))) return;

    len = bacnet_rp_ack_decode_service_request(
			service_request, service_len, &data);
    if (len < 0) {
	fprintf(stderr, "Read Property ACK service request decode failed\n");
    } else {
	bacapp_decode_application_data(
			data.application_data,
			data.application_data_len,
			&value);
	data_16 = value.type.Real;

	array_tail(data_16, instance);
#if debug
	fprintf(stderr, "%04X from server %i, instance %i\n", data_16,
	    device->device_id, instance->instance_no);
#endif
    }
}

static void send_rp_request(device_obj *device, instance_obj *instance) {
    if (!device->found) return;

    if (!instance->invoke_id)
	instance->invoke_id = 
		bacnet_Send_Read_Property_Request(
				    device->device_id,
				    instance->object_type,
				    instance->instance_no,
				    instance->object_property,
				    instance->array_index);

    else if (bacnet_tsm_invoke_id_free(instance->invoke_id))

	/* Transaction is finished */
	instance->invoke_id = 0;

    else if (bacnet_tsm_invoke_id_failed(instance->invoke_id)) {

	fprintf(stderr, "Error: TSM Timeout for device %i\n",
			device->device_id);

	bacnet_tsm_free_invoke_id(instance->invoke_id);
	instance->invoke_id = 0;
	device->found = 0;
    }
}

static void *read_prop_thread(void *arg) {
    device_obj *device;
    instance_obj *instance;
    while (1) {

	usleep(100000);

	pthread_mutex_lock(&timer_lock);

	list_for_each_entry(device, &devices, devices)
	    list_for_each_entry(instance, &device->instances, instances)
		send_rp_request(device, instance);
	    
	pthread_mutex_unlock(&timer_lock);
    }

    return arg;
}

void add_instance(size_t num_words, uint16_t *data, void *arg) {
    int bytes;
    instance_obj *instance;
    device_obj *device = (device_obj *) arg;

    instance = malloc(sizeof(instance_obj));
    memset(instance, 0, sizeof(instance_obj));

    instance->parent = device;
    instance->object_type = bacnet_OBJECT_ANALOG_INPUT;
    instance->object_property = bacnet_PROP_PRESENT_VALUE;
    instance->array_index = BACNET_ARRAY_ALL;

    bytes = num_words * sizeof(uint16_t);
    instance->num_words = num_words;
    instance->instance_no = instance_no++;

    instance->needle = malloc(bytes);
    instance->haystack = malloc(bytes);
    memcpy(instance->needle, data, num_words * sizeof(uint16_t));

    list_add_tail(&instance->instances, &device->instances);
}

void add_device(int device_id) {
    device_obj *device;

    device = malloc(sizeof(device_obj));
    memset(device, 0, sizeof(device_obj));

    device->device_id = device_id;
    INIT_LIST_HEAD(&device->instances);
    instance_no = 0;

    list_add_tail(&device->devices, &devices);

    file_channel_enumerate(add_instance, device);
}

void free_devices(void) {
    device_obj *device;
    instance_obj *instance;

    list_for_each_entry(device, &devices, devices) {
	list_for_each_entry(instance, &device->instances, instances) {
	    free(instance->needle);
	    free(instance->haystack);
	    free(instance);
	}
	free(device);
    }
    INIT_LIST_HEAD(&devices);
}

int main(int argc, char **argv) {
    uint8_t rx_buf[bacnet_MAX_MPDU];
    uint16_t pdu_len;
    BACNET_ADDRESS src;
    pthread_t read_prop_thread_id, minute_tick_id, second_tick_id;

    bacnet_Device_Set_Object_Instance_Number(BACNET_MAX_INSTANCE);
    bacnet_address_init();

    /* Setup device objects */
    bacnet_Device_Init(client_objects);
    BN_UNC(I_AM, i_am_bind);
    BN_CON_ACK(READ_PROPERTY, read_property_ack);
    BN_ERR(READ_PROPERTY, read_property_err);
    bacnet_apdu_set_abort_handler(abort_handler);
    bacnet_apdu_set_reject_handler(reject_handler);

    bacnet_BIP_Debug = true;
    bacnet_bip_set_port(htons(BACNET_PORT));
    bacnet_datalink_set(BACNET_DATALINK_TYPE);
    bacnet_datalink_init(BACNET_INTERFACE);
    atexit(bacnet_datalink_cleanup);
    memset(&src, 0, sizeof(src));

    register_with_bbmd();

    pthread_create(&read_prop_thread_id, 0, read_prop_thread, NULL);
    pthread_create(&minute_tick_id, 0, minute_tick, NULL);
    pthread_create(&second_tick_id, 0, second_tick, NULL);

    file_read_random_data(RANDOM_DATA_POOL);
    file_device_enumerate(add_device);
    file_free_random_data();

    while (1) {
	pdu_len = bacnet_datalink_receive(
		    &src, rx_buf, bacnet_MAX_MPDU, BACNET_SELECT_TIMEOUT_MS);

	if (pdu_len) {
	    /* May call any registered handler.
	     * Thread safety: May block, however we still need to guarantee
	     * atomicity with the timers, so hold the lock anyway */
	    pthread_mutex_lock(&timer_lock);
	    bacnet_npdu_handler(&src, rx_buf, pdu_len);
	    pthread_mutex_unlock(&timer_lock);
	}

	ms_tick();
    }

    free_devices();

    return 0;
}
