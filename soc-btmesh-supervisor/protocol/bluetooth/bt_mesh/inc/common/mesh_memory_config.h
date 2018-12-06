/******************************************************************************/
/**
 * @file   mesh_memory_config.h
 * @brief  Silicon Labs Bluetooth Mesh Memory Configuration API
 *
 *******************************************************************************
 * <b> (C) Copyright 2018 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 ******************************************************************************/

#ifndef _MESH_MEMORY_CONFIG_H
#define _MESH_MEMORY_CONFIG_H
#include <stdint.h>

/**
 * Structure defining the heap memory reserved for the BT Mesh stack.
 * Everything is allocated when the stack is initialized, in order
 * to prevent the heap from becoming fragmented.
 *
 * The maximum values for items defined in the DCD (elements, models,
 * replay protection list size, GATT proxy connections) should be set
 * according to what is actually needed. That is, max_elements should
 * be equal to the number of elements present in the DCD, and so on.
 *
 * The memory is wasted if the maximum value is higher than needed, but
 * causes no other side effects. Setting the value to a lower number
 * than required will cause undefined behavior.
 */
typedef struct {
  /** Local sequence number write interval; must be a power of two */
  uint32_t pstore_write_interval_elem_seq;
  /** Maximum number of elements on the device */
  uint8_t max_elements;
  /** Maximum number of models on the device */
  uint8_t max_models;
  /** Maximum number of network keys on the device */
  uint8_t max_net_keys;
  /** Maximum number of application keys on the device */
  uint8_t max_appkeys;
  /** Maximum number of device keys on the device; set to 1 for a node */
  uint8_t max_devkeys;
  /** Maximum number of friendships for a friend/LPN device */
  uint8_t max_friendships;
  /** Maximum number of model-application key bindings per model;
      should not be larger than the maximum number of application keys */
  uint8_t max_app_binds;
  /** Maximum number of subscription addresses per model */
  uint8_t max_subscriptions;
  /** Maximum number of concurrently ongoing foundation model commands */
  uint8_t max_foundation_model_commands;
  /** Network cache entry size */
  uint8_t net_cache_size;
  /** Replay protection list entry size; must be equal to or larger than
      the number of nodes the device communicates with */
  uint8_t replay_size;
  /** Maximum number of concurrent transport layer segmented messages
      being sent */
  uint8_t max_send_segs;
  /** Maximum number of concurrent transport layer segmented messages
      being received */
  uint8_t max_recv_segs;
  /** Maximum number of virtual addresses on the node */
  uint8_t max_virtual_addresses;
  /** Maximum number of provisioning sessions */
  uint8_t max_provision_sessions;
  /** Maximum number of provisioning bearers */
  uint8_t max_provision_bearers;
  /** Maximum number of concurrent GATT connections (for
      provisioning and proxying) */
  uint8_t max_gatt_connections;
  /** GATT transport queue size */
  uint8_t gatt_txqueue_size;
  /** Maximum number of provisioner database entries; ignored for node */
  uint8_t provisioner_max_ddb_entries;
  /** Maximum number of keys to store per node on provisioner;
      ignored for node */
  uint8_t provisioner_max_node_net_keys;
  uint8_t friend_max_total_cache;
  uint8_t friend_max_single_cache;
  uint8_t friend_max_subs_list;
} mesh_memory_config_t;

#endif
