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

#define BACNET_DEV		    48			//bacnet device number/port
#define BACNET_PORT		    0xBAC1
#define BACNET_INTERFACE	    "lo"
#define BACNET_DATALINK_TYPE	    "bvlc"
#define BACNET_SELECT_TIMEOUT_MS    1	    /* ms */

#define INST_NO		10

#define RUN_AS_BBMD_CLIENT	    1
#define SERVER_IP		    "140.159.153.159"	//modbus server location (VU)
#define SLEEP_TIME		    100000

#define BACNET_BBMD_PORT	    0xBAC0
#define BACNET_BBMD_ADDRESS	    "140.159.160.7"	//bacnet client location (vu)
#define BACNET_BBMD_TTL		    90

struct list_object_s {
    uint16_t mdata;                 /* 8 bytes */
    struct list_object_s *next;     /* 8 bytes */
};

/* If you are trying out the test suite from home, this data matches the data
 * stored in RANDOM_DATA_POOL for device number 12
 * BACnet client will print "Successful match" whenever it is able to receive
 * this set of data. Note that you will not have access to the RANDOM_DATA_POOL
 * for your final submitted application. */
//static uint16_t test_data[] = {
//    0xA4EC, 0x6E39, 0x8740, 0x1065, 0x9134, 0xFC8C };
//#define NUM_TEST_DATA (sizeof(test_data)/sizeof(test_data[0]))

static pthread_mutex_t list_lock = PTHREAD_MUTEX_INITIALIZER;		// tells if list is locked
static pthread_cond_t list_ready = PTHREAD_COND_INITIALIZER;			// is list ready
static pthread_mutex_t timer = PTHREAD_MUTEX_INITIALIZER;		// tells if timer is active

static struct list_object_s *list_head[INST_NO];		// pointer to start of list

// DO WORK HERE
// adds to end of list
// included from Kim's example wholesale
static void add_to_list(uint16_t input,struct list_object_s **list_head) {
    /* Allocate memory */
    struct list_object_s *last_item;
    struct list_object_s *new_item = malloc(sizeof(struct list_object_s));
    if (!new_item) {
        fprintf(stderr, "Malloc failed\n");
        exit(1);
    }

    /* Set up the object */
    new_item->mdata = input;		//gets modbus data
    new_item->next = NULL;

    /* list_head is shared between threads, need to lock before access */
    pthread_mutex_lock(&list_lock);

    if (*list_head == NULL) {					//changing to array type variable makes threading compile nicely
        /* Adding the first object */
        *list_head = new_item;
    } else {
        /* Adding the nth object */
        last_item = *list_head;
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
static struct list_object_s *list_get_first(struct list_object_s **list_head) {
    struct list_object_s *first_item;

    first_item = *list_head;
    *list_head = (*list_head)->next;

    return first_item;
}

// this server sends to BACnet
static int Update_Analog_Input_Read_Property(
		BACNET_READ_PROPERTY_DATA *rpdata) {

    struct list_object_s *cur_obj;				//structure: current data object

    int insta_num = bacnet_Analog_Input_Instance_To_Index(
			rpdata->object_instance);		//current instance

    if (rpdata->object_property != bacnet_PROP_PRESENT_VALUE) goto not_pv;	//bailout
    if (list_head[insta_num] == NULL) goto not_pv;

	cur_obj = list_get_first(&list_head[insta_num]);			//calls list first item
	printf("AI_Present_Value request for instance %i\n", insta_num);
    bacnet_Analog_Input_Present_Value_Set(insta_num, cur_obj->mdata);		//sends current data to current instance

    //static int index;
    //bacnet_Analog_Input_Present_Value_Set(0, test_data[index++]);		//test data
    //if (index == NUM_TEST_DATA) index = 0;

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
	pthread_mutex_lock(&timer);

	/* Expire addresses once the TTL has expired */
	bacnet_address_cache_timer(60);

	/* Re-register with BBMD once BBMD TTL has expired */
	register_with_bbmd();
	
	/* Sleep for 1 minute */
	pthread_mutex_unlock(&timer);
	sleep(60);
    }
    return arg;
}

static void *second_tick(void *arg) {
    while (1) {
	pthread_mutex_lock(&timer);

	/* Invalidates stale BBMD foreign device table entries */
	bacnet_bvlc_maintenance_timer(1);

	/* Transaction state machine: Responsible for retransmissions and ack
	 * checking for confirmed services */
	bacnet_tsm_timer_milliseconds(1000);

	pthread_mutex_unlock(&timer);
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
static void *server_connect (void *check) {
  modbus_t *mb;
  uint16_t tab_reg[32];
  int cnt;

  mb = modbus_new_tcp(SERVER_IP, 502);								// change ip
  modbus_connect(mb);

	//check connection
  if (modbus_connect(mb) == -1) {
     fprintf(stderr, "Connection failed: %s\n", modbus_strerror (errno));
     modbus_free(mb);
     return (check);
     }
	// Read INST_NO registers from the address BACNET_DEV
while (1)
  { usleep(SLEEP_TIME);
    modbus_read_registers(mb, BACNET_DEV, INST_NO, tab_reg);

      for (cnt=0 ; cnt< INST_NO ; cnt++)
	add_to_list(tab_reg[cnt],&list_head[cnt]);	// loop here: linked list
        printf("got value %x\n", tab_reg[cnt]);
      }
  modbus_close(mb);
  modbus_free(mb);

return 0;
}
//end of client

int main(int argc, char **argv) {
    uint8_t rx_buf[bacnet_MAX_MPDU];
    uint16_t pdu_len;
    BACNET_ADDRESS src;
    pthread_t minute_tick_id, second_tick_id, server_id;

    bacnet_Device_Set_Object_Instance_Number(BACNET_DEV);
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
    pthread_create(&server_id, 0, server_connect, NULL);

    while (1) {
	pdu_len = bacnet_datalink_receive(
		    &src, rx_buf, bacnet_MAX_MPDU, BACNET_SELECT_TIMEOUT_MS);

	if (pdu_len) {
	    /* May call any registered handler.
	     * Thread safety: May block, however we still need to guarantee
	     * atomicity with the timers, so hold the lock anyway */
	    pthread_mutex_lock(&timer);
	    bacnet_npdu_handler(&src, rx_buf, pdu_len);
	    pthread_mutex_unlock(&timer);
	}

	ms_tick();
    }

    return 0;
}
