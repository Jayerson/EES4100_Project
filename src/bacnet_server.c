/* major functions
pthread_mutex_t t*second_tick()
Update_Analog_Input_Read_Property()
index
bacnet_object_functions_t server_objects[]
register_with_bbmd()
*minute_tick()
*second_tick()
ms_tick()
*server_connect() - mine
add_to_list()
*list_get_first()
list_flush

int main
*/

#include <modbus-tcp.h>
#include <stdio.h>
#include <errno.h>

#include <libbacnet/address.h>
#include <libbacnet/device.h>
#include <libbacnet/handlers.h>
#include <libbacnet/datalink.h>
#include <libbacnet/bvlc.h>
#include <libbacnet/client.h>
#include <libbacnet/txbuf.h>
#include <libbacnet/tsm.h>
#include <libbacnet/ai.h>
#include "bacnet_namespace.h"

#define BACNET_INSTANCE_NO	    12
#define BACNET_PORT		    0xBAC1
#define BACNET_INTERFACE	    "lo"
#define BACNET_DATALINK_TYPE	    "bvlc"
#define BACNET_SELECT_TIMEOUT_MS    1	    /* ms */

#define RUN_AS_BBMD_CLIENT	    1

#if RUN_AS_BBMD_CLIENT
#define BACNET_BBMD_PORT	    0xBAC0
#define BACNET_BBMD_ADDRESS	    "127.0.0.1"
#define BACNET_BBMD_TTL		    90
#endif

struct list_object_s {
    char *string;                   /* 8 bytes */
    int strlen;                     /* 4 bytes */
    struct list_object_s *next;     /* 8 bytes */
};

// change address here: linked list setup below

/* If you are trying out the test suite from home, this data matches the data
 * stored in RANDOM_DATA_POOL for device number 12
 * BACnet client will print "Successful match" whenever it is able to receive
 * this set of data. Note that you will not have access to the RANDOM_DATA_POOL
 * for your final submitted application. */
static uint16_t test_data[] = {
    0xA4EC, 0x6E39, 0x8740, 0x1065, 0x9134, 0xFC8C };
#define NUM_TEST_DATA (sizeof(test_data)/sizeof(test_data[0]))

static pthread_mutex_t list_lock = PTHREAD_MUTEX_INITIALIZER;		// tells if list is locked
static pthread_cond_t list_ready = PTHREAD_COND_INITIALIZER;			// is list ready
static pthread_cond_t list_data_flush = PTHREAD_COND_INITIALIZER;		// is list flushed
static pthread_mutex_t server = PTHREAD_MUTEX_INITIALIZER;		// tells if server is active

static struct list_object_s *list_head;		// pointer to start of list

// DO WORK HERE
// adds to end of list
// included from Kim's example wholesale
static void add_to_list(char *input) {
    /* Allocate memory */
    struct list_object_s *last_item;
    struct list_object_s *new_item = malloc(sizeof(struct list_object_s));
    if (!new_item) {
        fprintf(stderr, "Malloc failed\n");
        exit(1);
    }

    /* Set up the object */
    new_item->string = strdup(input);
    new_item->strlen = strlen(input);
    new_item->next = NULL;

    /* list_head is shared between threads, need to lock before access */
    pthread_mutex_lock(&list_lock);

    if (list_head == NULL) {
        /* Adding the first object */
        list_head = new_item;
    } else {
        /* Adding the nth object */
        last_item = list_head;
        while (last_item->next) last_item = last_item->next;
        last_item->next = new_item;
    }

    /* Inform other functions that data is available */
    pthread_cond_signal(&list_ready);
    /* Release shared data lock */
    pthread_mutex_unlock(&list_lock);
}

// gets from head of list and moves list on
// from example
static struct list_object_s *list_get_first(void) {
    struct list_object_s *first_item;

    first_item = list_head;
    list_head = list_head->next;

    return first_item;
}

//clears the list
static void list_flush(void) {
    pthread_mutex_lock(&list_lock);

    while (list_head) {
        pthread_cond_signal(&list_data_flush);
        pthread_cond_wait(&list_data_flush, &list_lock);
    }
    pthread_mutex_unlock(&list_lock);
}

// this server sends to BACnet
static int Update_Analog_Input_Read_Property(
		BACNET_READ_PROPERTY_DATA *rpdata) {

    static int index;
    int instance_no = bacnet_Analog_Input_Instance_To_Index(
			rpdata->object_instance);

    if (rpdata->object_property != bacnet_PROP_PRESENT_VALUE) goto not_pv;

    printf("AI_Present_Value request for instance %i\n", instance_no);

// do work here
// *** call: list_get_first

    /* Update the values to be sent to the BACnet client here.
     * The data should be read from the head of a linked list. You are required
     * to implement this list functionality.
     *
     * bacnet_Analog_Input_Present_Value_Set() 
     *     First argument: Instance No
     *     Second argument: data to be sent
     *
     * Without reconfiguring libbacnet, a maximum of 4 values may be sent */
    bacnet_Analog_Input_Present_Value_Set(0, test_data[index++]);
    bacnet_Analog_Input_Present_Value_Set(1, test_data[index++]);
    bacnet_Analog_Input_Present_Value_Set(2, test_data[index++]);
    bacnet_Analog_Input_Present_Value_Set(3, test_data[index++]);
// 4 instances
    
    if (index == NUM_TEST_DATA) index = 0;

not_pv:
    return bacnet_Analog_Input_Read_Property(rpdata);
}
//end of server-to-bacnet

static bacnet_object_functions_t server_objects[] = {
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

    {bacnet_OBJECT_ANALOG_INPUT,
            bacnet_Analog_Input_Init,
            bacnet_Analog_Input_Count,
            bacnet_Analog_Input_Index_To_Instance,
            bacnet_Analog_Input_Valid_Instance,
            bacnet_Analog_Input_Object_Name,
            Update_Analog_Input_Read_Property,
            bacnet_Analog_Input_Write_Property,
            bacnet_Analog_Input_Property_Lists,
            NULL /* ReadRangeInfo */ ,
            NULL /* Iterator */ ,
            bacnet_Analog_Input_Encode_Value_List,
            bacnet_Analog_Input_Change_Of_Value,
            bacnet_Analog_Input_Change_Of_Value_Clear,
            bacnet_Analog_Input_Intrinsic_Reporting},
    {MAX_BACNET_OBJECT_TYPE}
};

static void register_with_bbmd(void) {
#if RUN_AS_BBMD_CLIENT
    /* Thread safety: Shares data with datalink_send_pdu */
    bacnet_bvlc_register_with_bbmd(
	    bacnet_bip_getaddrbyname(BACNET_BBMD_ADDRESS), 
	    htons(BACNET_BBMD_PORT),
	    BACNET_BBMD_TTL);
#endif
}

static void *minute_tick(void *arg) {
    while (1) {
	pthread_mutex_lock(&server);

	/* Expire addresses once the TTL has expired */
	bacnet_address_cache_timer(60);

	/* Re-register with BBMD once BBMD TTL has expired */
	register_with_bbmd();

	/* Update addresses for notification class recipient list 
	 * Requred for INTRINSIC_REPORTING
	 * bacnet_Notification_Class_find_recipient(); */
	
	/* Sleep for 1 minute */
	pthread_mutex_unlock(&server);
	sleep(60);
    }
    return arg;
}

static void *second_tick(void *arg) {
    while (1) {
	pthread_mutex_lock(&server);

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
	pthread_mutex_unlock(&server);
	sleep(1);
    }
    return arg;
}

static void ms_tick(void) {
    /* Updates change of value COV subscribers.
     * Required for SERVICE_CONFIRMED_SUBSCRIBE_COV
     * bacnet_handler_cov_task(); */
}

#define BN_UNC(service, handler) \
    bacnet_apdu_set_unconfirmed_handler(		\
		    SERVICE_UNCONFIRMED_##service,	\
		    bacnet_handler_##handler)
#define BN_CON(service, handler) \
    bacnet_apdu_set_confirmed_handler(			\
		    SERVICE_CONFIRMED_##service,	\
		    bacnet_handler_##handler)


// client to modbus
int *server_connect (void) {
  modbus_t *mb;
  uint16_t tab_reg[32];

  mb = modbus_new_tcp("127.0.0.1", 502);
  modbus_connect(mb);

//check connection
  if (modbus_connect(mb) == -1) {
     fprintf(stderr, "Connection failed: %s\n", modbus_strerror (errno));
     modbus_free(mb);
     return -1;
     }

// Read 5 registers from the address 0
  modbus_read_registers(mb, 0, 5, tab_reg);

  modbus_close(mb);
  modbus_free(mb);

// loop here: linked list
// do work here
    
// *** call: add_to_list
    /* Loop:
     *	    Read the required number of registers from the modbus server
     *	    Store the register data into the tail of a linked list 
     */
return 0;
}
//end of client

int main(int argc, char **argv) {
    uint8_t rx_buf[bacnet_MAX_MPDU];
    uint16_t pdu_len;
    BACNET_ADDRESS src;
    pthread_t minute_tick_id, second_tick_id, server_connect;

    list_flush();		//clear before starting

    bacnet_Device_Set_Object_Instance_Number(BACNET_INSTANCE_NO);
    bacnet_address_init();

    /* Setup device objects */
    bacnet_Device_Init(server_objects);
    BN_UNC(WHO_IS, who_is);
    BN_CON(READ_PROPERTY, read_property);

    bacnet_BIP_Debug = true;
    bacnet_bip_set_port(htons(BACNET_PORT));
    bacnet_datalink_set(BACNET_DATALINK_TYPE);
    bacnet_datalink_init(BACNET_INTERFACE);
    atexit(bacnet_datalink_cleanup);
    memset(&src, 0, sizeof(src));

    register_with_bbmd();

    bacnet_Send_I_Am(bacnet_Handler_Transmit_Buffer);

    pthread_create(&minute_tick_id, 0, minute_tick, NULL);
    pthread_create(&second_tick_id, 0, second_tick, NULL);

//start server and loop
    pthread_create(&server_connect, 0, server_connect, NULL);

    while (1) {
	pdu_len = bacnet_datalink_receive(
		    &src, rx_buf, bacnet_MAX_MPDU, BACNET_SELECT_TIMEOUT_MS);

	if (pdu_len) {
	    /* May call any registered handler.
	     * Thread safety: May block, however we still need to guarantee
	     * atomicity with the timers, so hold the lock anyway */
	    pthread_mutex_lock(&server);
	    bacnet_npdu_handler(&src, rx_buf, pdu_len);
	    pthread_mutex_unlock(&server);
	}

	ms_tick();
    }

    return 0;
}
