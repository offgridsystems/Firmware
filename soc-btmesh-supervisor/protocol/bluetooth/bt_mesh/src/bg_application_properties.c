/***************************************************************************//**
 * @file ble_application_properties.c
 * @brief Template for Application Properties
 * @author Silicon Labs
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>Copyright 2017 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include <application_properties.h>

#if defined(__IAR_SYSTEMS_ICC__)
/* IAR ICC */
  #define KEEP_SYMBOL           _Pragma("location=\".application_properties\"") __root
#elif defined(__GNUC__)
/* GNU GCC */
  #define KEEP_SYMBOL           __attribute__ ((section(".application_properties")))
#else
  #define KEEP_SYMBOL
#endif

/// Version number for this application (uint32_t)
#define BG_APP_PROPERTIES_VERSION 1
/// Capabilities of this application (uint32_t)
#define BG_APP_PROPERTIES_CAPABILITIES 0
/// Unique ID (e.g. UUID or GUID) for the product this application is built for (uint8_t[16])
#define BG_APP_PROPERTIES_ID { 0 }

KEEP_SYMBOL const ApplicationProperties_t bg_application_properties
  =
  {
  /// @brief Magic value indicating that this is an ApplicationProperties_t struct.
  /// Must equal @ref APPLICATION_PROPERTIES_MAGIC
  .magic = APPLICATION_PROPERTIES_MAGIC,
  /// Version number of this struct
  .structVersion = APPLICATION_PROPERTIES_VERSION,
  /// Type of signature this application is signed with
  .signatureType = APPLICATION_SIGNATURE_NONE,
  /// Location of the signature. Typically a pointer to the end of the application
  .signatureLocation = 0,
  /// Information about the application
  .app = {
    /// Bitfield representing type of application, e.g. @ref APPLICATION_TYPE_BLUETOOTH_APP
    .type = APPLICATION_TYPE_BLUETOOTH_APP,
    /// Version number for this application
    .version = BG_APP_PROPERTIES_VERSION,
    /// Capabilities of this application
    .capabilities = BG_APP_PROPERTIES_CAPABILITIES,
    /// Unique ID (e.g. UUID or GUID) for the product this application is built for
    .productId = BG_APP_PROPERTIES_ID,
  },
  };
