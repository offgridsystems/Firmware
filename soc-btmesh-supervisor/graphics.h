/***********************************************************************************************//**
 * \file graphics.h
 * \brief Displays text on the LCD
 ***************************************************************************************************
 * <b> (C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

#ifndef GRAPHICS_H
#define GRAPHICS_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************************************
   Public Function Declarations
***************************************************************************************************/

/***********************************************************************************************//**
 *  \brief  Initializes the graphics stack
 *  \param[in]  header  Header Text on display
 **************************************************************************************************/
void graphInit(char* header);

/***********************************************************************************************//**
 *  \brief  display a string on the LCD center aligned
 *  \param[in]  string  String to be displayed
 **************************************************************************************************/
void graphWriteString(char *string);

#ifdef __cplusplus
}
#endif

#endif /* GRAHPHICS_H */
