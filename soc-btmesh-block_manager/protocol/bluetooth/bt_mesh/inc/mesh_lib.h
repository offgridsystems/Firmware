#ifndef MESH_LIB_H
#define MESH_LIB_H

typedef enum {
  MESH_REQUEST_FLAG_NONRELAYED = 0x01,
  MESH_REQUEST_FLAG_RESPONSE_REQUIRED = 0x02,
} mesh_request_flags_t;

typedef enum {
  MESH_RESPONSE_FLAG_NONRELAYED = 0x01,
} mesh_response_flags_t;

uint32_t mesh_lib_transition_time_to_ms(uint8_t transition_time);

/***
 *** Library initialization
 ***/

errorcode_t mesh_lib_init(void *(*malloc_fn)(size_t),
                          void (*free_fn)(void *),
                          size_t generic_models);

void mesh_lib_deinit(void);

/***
 *** Generic event handlers
 ***/

void mesh_lib_generic_server_event_handler(struct gecko_cmd_packet *evt);

void mesh_lib_generic_client_event_handler(struct gecko_cmd_packet *evt);

/***
 *** Generic Server
 ***/

typedef void
(*mesh_lib_generic_server_client_request_cb)(uint16_t model_id,
                                             uint16_t element_index,
                                             uint16_t client_addr,
                                             uint16_t server_addr,
                                             uint16_t appkey_index,
                                             const struct mesh_generic_request *req,
                                             uint32_t transition_ms,
                                             uint16_t delay_ms,
                                             uint8_t request_flags);

typedef void
(*mesh_lib_generic_server_change_cb)(uint16_t model_id,
                                     uint16_t element_index,
                                     const struct mesh_generic_state *current,
                                     const struct mesh_generic_state *target,
                                     uint32_t remaining_ms);

errorcode_t
mesh_lib_generic_server_response(uint16_t model_id,
                                 uint16_t element_index,
                                 uint16_t client_addr,
                                 uint16_t appkey_index,
                                 const struct mesh_generic_state *current,
                                 const struct mesh_generic_state *target,
                                 uint32_t remaining_ms,
                                 uint8_t response_flags);

errorcode_t
mesh_lib_generic_server_update(uint16_t model_id,
                               uint16_t element_index,
                               const struct mesh_generic_state *current,
                               const struct mesh_generic_state *target,
                               uint32_t remaining_ms);

errorcode_t
mesh_lib_generic_server_publish(uint16_t model_id,
                                uint16_t element_index,
                                mesh_generic_state_t kind);

errorcode_t
mesh_lib_generic_server_register_handler(uint16_t model_id,
                                         uint16_t element_index,
                                         mesh_lib_generic_server_client_request_cb cb,
                                         mesh_lib_generic_server_change_cb ch);

/***
 *** Generic Client
 ***/

typedef void
(*mesh_lib_generic_client_server_response_cb)(uint16_t model_id,
                                              uint16_t element_index,
                                              uint16_t client_addr,
                                              uint16_t server_addr,
                                              const struct mesh_generic_state *current,
                                              const struct mesh_generic_state *target,
                                              uint32_t remaining_ms,
                                              uint8_t response_flags);

errorcode_t
mesh_lib_generic_client_get(uint16_t model_id,
                            uint16_t element_index,
                            uint16_t server_addr,
                            uint16_t appkey_index,
                            mesh_generic_state_t kind);

errorcode_t
mesh_lib_generic_client_set(uint16_t model_id,
                            uint16_t element_index,
                            uint16_t server_addr,
                            uint16_t appkey_index,
                            uint8_t transaction_id,
                            const struct mesh_generic_request *req,
                            uint32_t transition_ms,
                            uint16_t delay_ms,
                            uint8_t request_flags);

errorcode_t
mesh_lib_generic_client_publish(uint16_t model_id,
                                uint16_t element_index,
                                uint16_t appkey_index,
                                uint8_t transaction_id,
                                const struct mesh_generic_request *req,
                                uint32_t transition_ms,
                                uint16_t delay_ms,
                                uint8_t request_flags);

errorcode_t
mesh_lib_generic_client_register_handler(uint16_t model_id,
                                         uint16_t element_index,
                                         mesh_lib_generic_client_server_response_cb cb);

#endif
