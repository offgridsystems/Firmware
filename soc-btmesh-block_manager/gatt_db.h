// Copyright 2018 Silicon Laboratories, Inc.
//
//

/********************************************************************
 * Autogenerated file, do not edit.
 *******************************************************************/

#ifndef __GATT_DB_H
#define __GATT_DB_H

#include "bg_gattdb_def.h"

extern const struct bg_gattdb_def bg_gattdb_data;

#define gattdb_service_changed_char             3
#define gattdb_device_name                      7
#define gattdb_ota_control                     27

typedef enum
{
    mesh_provisioning_service      = 0x0001,
    mesh_proxy_service             = 0x0002,
    bg_gattdb_data_all_caps = 0x0003
} bg_gattdb_data_cap_t;

#endif
