/******************************************************************************/
/**
 * @file   mesh_lighting_model_capi_types.h
 * @brief  Silicon Labs Bluetooth Mesh Lighting Model API
 *
 * Please see the @ref mesh_generic "generic model API"
 *
 *******************************************************************************
 * <b> (C) Copyright 2018 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 ******************************************************************************/

/*
 * C API for lighting models
 */

#ifndef MESH_LIGHTING_MODEL_CAPI_TYPES_H
#define MESH_LIGHTING_MODEL_CAPI_TYPES_H

/*
 * Lighting model IDs
 */
#define MESH_LIGHTING_LIGHTNESS_SERVER_MODEL_ID 0x1300
#define MESH_LIGHTING_LIGHTNESS_SETUP_SERVER_MODEL_ID 0x1301
#define MESH_LIGHTING_LIGHTNESS_CLIENT_MODEL_ID 0x1302

#define MESH_LIGHTING_CTL_SERVER_MODEL_ID 0x1303
#define MESH_LIGHTING_CTL_SETUP_SERVER_MODEL_ID 0x1304
#define MESH_LIGHTING_CTL_CLIENT_MODEL_ID 0x1305
#define MESH_LIGHTING_CTL_TEMPERATURE_SERVER_MODEL_ID 0x1306

/** Light CTL Temperature Minimum */
#define MESH_LIGHTING_CTL_TEMPERATURE_MIN 0x0320

/** Light CTL Temperature Maximum */
#define MESH_LIGHTING_CTL_TEMPERATURE_MAX 0x4e20

#endif /* MESH_LIGHTING_MODEL_CAPI_TYPES_H */
