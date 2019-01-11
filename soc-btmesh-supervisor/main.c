/***********************************************************************************************//**
 * \file   main.c
 * \brief  Supervisor Main Code
 *
 * 11/27/18 Make it do something
 *
 **************************************************************************************************/

/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "retargetserial.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>
#include <em_rtcc.h>
#include <gpiointerrupt.h>

/* Device initialization header */
#include "hal-config.h"

/* Display Interface header */
#include "display_interface.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/*************************************** BT SETUP ***************************************/

// Maximum number of simultaneous Bluetooth connections
#define MAX_CONNECTIONS 2

// heap for Bluetooth stack
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

// set link layer priorities to  default
static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

// Bluetooth stack configuration
const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, 					// Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, 	// Configure PA input to VBAT
#endif 										// (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .max_timers = 16,
};

/*************************************** GLOBAL VAR ***************************************/

static uint16 _elem_index = 0xffff; // for indexing elements of the node
static uint16 _my_address = 0;    	// Address of the Primary Element of the Node
uint16_t sample_data = 2305;		// test data
static uint8 trid = 0;        		// transaction identifier
static uint8 num_connections = 0;   // number of active Bluetooth connections
static uint8 conn_handle = 0xFF;    // handle of the last opened LE connection

/*************************************** PROTOTYPES ******************************************/

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

/*************************************** BT FUNCTIONS ***************************************/


// block status request - request for block to update the block managers data about specific block
static void block_status_request(uint16_t model_id,
                              	 uint16_t element_index,
								 uint16_t client_addr,
								 uint16_t server_addr,
								 uint16_t appkey_index,
								 const struct mesh_generic_request *request,
								 uint32_t transition_ms,
								 uint16_t delay_ms,
								 uint8_t request_flags){
	char buf[30];
	sprintf(buf, "%x", client_addr);
	DI_Print(buf, 1);
	sprintf(buf, "ob_ntc1: %x", request->block_temp.temp_ob_ntc1);
	DI_Print(buf, 2);
}

static void block_status_change(uint16_t model_id,
                                uint16_t element_index,
								const struct mesh_generic_state *current,
								const struct mesh_generic_state *target,
								uint32_t remaining_ms){
}

/*************************************** BT INIT ***************************************/

/**
 * node initialization. This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 */
void supervisor_init(void)
{
  mesh_lib_init(malloc, free, 8);
}

void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "Pack Supervisor %x:%x", pAddr->addr[1], pAddr->addr[0]);

  printf("Device name: '%s'\r\n", name);

  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res) {
    printf("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }

  // show device name on the LCD
  DI_Print(name, DI_ROW_NAME);
}

// Initialization of the models supported by this node. This function registers callbacks for
// each of the supported models.
static void init_models(void){
  mesh_lib_generic_server_register_handler(MESH_DK_BLOCK_STATUS_SERVER_MODEL_ID,
                                           0,
                                           block_status_request,
                                           block_status_change);
}

/*************************************** MAIN ***************************************/


int main()
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Minimize advertisement latency by allowing the advertiser to always
  // interrupt the scanner.
  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  gecko_cmd_mesh_prov_init();		//Initializes the Mesh stack in Provisioner role

  gecko_stack_init(&config);
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  //gecko_bgapi_class_gatt_init();
  gecko_bgapi_class_gatt_server_init();
  gecko_bgapi_class_endpoint_init();
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
  //gecko_bgapi_class_sm_init();
  //mesh_native_bgapi_init();
  gecko_bgapi_class_mesh_node_init();
  //gecko_bgapi_class_mesh_prov_init();
  gecko_bgapi_class_mesh_proxy_init();
  gecko_bgapi_class_mesh_proxy_server_init();
  //gecko_bgapi_class_mesh_proxy_client_init();
  gecko_bgapi_class_mesh_generic_client_init();
  //gecko_bgapi_class_mesh_generic_server_init();
  //gecko_bgapi_class_mesh_vendor_model_init();
  //gecko_bgapi_class_mesh_health_client_init();
  //gecko_bgapi_class_mesh_health_server_init();
  //gecko_bgapi_class_mesh_test_init();
  //gecko_bgapi_class_mesh_lpn_init();
  //gecko_bgapi_class_mesh_friend_init();

  gecko_initCoexHAL();			//wifi coexistence
  RETARGET_SerialInit();		//retarget printf()
  DI_Init();					//setup display

  //MAIN LOOPs
  while (1) {
      struct gecko_cmd_packet *evt = gecko_wait_event();
      bool pass = mesh_bgapi_listener(evt);
      if (pass) {
          handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
      }
  }
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	uint16 result;
	char buf[30];

	struct gecko_msg_mesh_node_provisioning_failed_evt_t  *prov_fail_evt;

	if (NULL == evt) {
		return;
	}

	switch (evt_id) {

	/************** BT PROVISIONER EVENTS **************/

	case gecko_evt_mesh_prov_initialized_id: 		// Provisioner initialized and operational
		printf("provisioner initialized\r\n");
		printf("scanning for nodes\r\n");
		gecko_cmd_mesh_prov_scan_unprov_beacons(); 	//Find them nodes
		break;

	case gecko_evt_mesh_prov_unprov_beacon_id:		//Unprovisioned beacon seen
		printf("::Node Found::\r\n");
		struct gecko_msg_mesh_prov_unprov_beacon_evt_t *beaconData = (struct gecko_msg_mesh_prov_unprov_beacon_evt_t *)&(evt->data);
		printf("address:%x, type:%d\r\n", beaconData->address, beaconData->address_type);
		printf("uuid:%x\r\n", beaconData->uuid);
		break;

  	/************** BT EVENTS **************/

    case gecko_evt_dfu_boot_id:				//device firmware upgrade signal
        //gecko_cmd_le_gap_set_advertising_timing(0, 1000*adv_interval_ms/625, 1000*adv_interval_ms/625, 0, 0);
        gecko_cmd_le_gap_set_mode(2, 2);
      	break;

    case gecko_evt_mesh_node_initialized_id:  	// bt stack is ready
    	printf("node initialized\r\n");

    	gecko_cmd_mesh_generic_client_init();

    	struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

    	if (pData->provisioned) {
    		printf("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

    	    _my_address = pData->address;
    	    _elem_index = 0;   					// index of primary element is zero

    	    supervisor_init();

    	    DI_Print("provisioned", DI_ROW_STATUS);
    	} else {
    		printf("node is unprovisioned\r\n");
    	    DI_Print("unprovisioned", DI_ROW_STATUS);
    	    printf("starting unprovisioned beaconing...\r\n");
    	    // The Node is now initialized, start unprovisioned Beaconing using PB-Adv Bearer
    	    gecko_cmd_mesh_node_start_unprov_beaconing(0x1);
    	    //gecko_cmd_mesh_node_start_unprov_beaconing(0x2);
    	}
    	break;

    case gecko_evt_mesh_node_provisioning_started_id:
    	printf("Started provisioning\r\n");
        DI_Print("provisioning...", DI_ROW_STATUS);
        // start timer for blinking LEDs to indicate which node is being provisioned
        // gecko_cmd_hardware_set_soft_timer(32768 / 4, TIMER_ID_PROVISIONING, 0);
        break;

    case gecko_evt_mesh_node_provisioned_id:
        _elem_index = 0;   // index of primary element is zero.
        supervisor_init();
        printf("node provisioned, got address=%x\r\n", evt->data.evt_mesh_node_provisioned.address);
        // stop LED blinking when provisioning complete
        // gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_PROVISIONING, 0);
        // LED_set_state(LED_STATE_OFF);
        DI_Print("provisioned", DI_ROW_STATUS);
        break;

    case gecko_evt_mesh_node_provisioning_failed_id:
        prov_fail_evt = (struct gecko_msg_mesh_node_provisioning_failed_evt_t  *)&(evt->data);
        printf("provisioning failed, code %x\r\n", prov_fail_evt->result);
        DI_Print("prov failed", DI_ROW_STATUS);
        // start a one-shot timer that will trigger soft reset after small delay
        // gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
        break;

    // This event is received when a Configuration Client has deployed a new network or application key to the node.
    case gecko_evt_mesh_node_key_added_id:
        printf("got new %s key with index %x\r\n",
        		evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
                evt->data.evt_mesh_node_key_added.index);
        break;

    // Informative. This event notifies that a remote Configuration Client has changed the configuration of a local model.
    case gecko_evt_mesh_node_model_config_changed_id:
        printf("model config changed\r\n");
        break;

    case gecko_evt_le_connection_closed_id:
    	/* Check if need to boot to dfu mode */
    	if (boot_to_dfu) {
    		/* Enter to DFU OTA mode */
    		gecko_cmd_system_reset(2);
    	}
    	printf("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
    	conn_handle = 0xFF;
    	if (num_connections > 0) {
    		if (--num_connections == 0) {
    			DI_Print("", DI_ROW_CONNECTION);
    			//lpn_init();
    		}
    	}
    	break;

    case gecko_evt_gatt_server_user_write_request_id:
        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
            /* Set flag to enter to OTA mode */
        	boot_to_dfu = 1;
        	/* Send response to Write Request */
        	gecko_cmd_gatt_server_send_user_write_response(
        			evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control,
					bg_err_success);

        	/* Close connection to enter to DFU OTA mode */
        	gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

    	/************** SYS EVENTS **************/

    case gecko_evt_system_boot_id:  // device started, radio ready
    	;
        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
        set_device_name(&pAddr->address);
        // Initialize Mesh stack in Node operation mode, wait for initialized event
        result = gecko_cmd_mesh_node_init()->result;
        if (result) {
        	sprintf(buf, "init failed (0x%x)", result);
            DI_Print(buf, DI_ROW_STATUS);
        }
        break;

    default:
    	//printf("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
    	break;
  }
}
