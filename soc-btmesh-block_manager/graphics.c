/***********************************************************************************************//**
 * \file   graphics.c
 * \brief  Draws the graphics on the display
 ***************************************************************************************************
 * <b> (C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* standard headers */
#include <string.h>
#include <stdio.h>

#include "bg_types.h"
#include "em_types.h"
#include "glib.h"
#include "dmd.h"
#include "display.h"

/* Own header */
#include "graphics.h"

/***************************************************************************************************
   Local Variables
 **************************************************************************************************/

/* Global glib context */
static GLIB_Context_t glibContext;
/* Current line number stored for printing text */
static uint8_t graphLineNum = 0;
/* Device name string */
static char *deviceHeader = NULL;

/***************************************************************************************************
   Static Function Declarations
 **************************************************************************************************/
static void graphPrintCenter(GLIB_Context_t *pContext, char *pString);

/***************************************************************************************************
   Function Definitions
 **************************************************************************************************/
void graphInit(char *header)
{
  EMSTATUS status;

  /* Initialize the display module. */
  status = DISPLAY_Init();
  if (DISPLAY_EMSTATUS_OK != status) {
    while (1)
      ;
  }

  /* Initialize the DMD module for the DISPLAY device driver. */
  status = DMD_init(0);
  if (DMD_OK != status) {
    while (1)
      ;
  }

  status = GLIB_contextInit(&glibContext);
  if (GLIB_OK != status) {
    while (1)
      ;
  }

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Use Narrow font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *)&GLIB_FontNarrow6x8);

  deviceHeader = header;
}

void graphWriteString(char *string)
{
  GLIB_clear(&glibContext);

  /* Reset line number, print header and device name */
  graphLineNum = 0;
  graphPrintCenter(&glibContext, deviceHeader);

  /* Print the string below the header center aligned */
  graphPrintCenter(&glibContext, string);

  DMD_updateDisplay();
}

/***************************************************************************************************
   Static Function Definitions
 **************************************************************************************************/

/***********************************************************************************************//**
 *  \brief  Print the given string center aligned
 *  \note   The string may contain several lines separated by new line
 *          characters ('\n'). Each line will be printed center aligned.
 *  \param[in]  pContext  Context
 *  \param[in]  pString  String to be displayed
 **************************************************************************************************/
static void graphPrintCenter(GLIB_Context_t *pContext, char *pString)
{
  do {
    char* nextToken;
    uint8_t len;

    /* Search for the next important token (new line or terminating NULL) */
    for (nextToken = pString; ((*nextToken != '\n') && (*nextToken != '\0')); nextToken++) {
      ;
    }

    len = nextToken - pString;
    /* Print the line if it is not null length */
    if (len) {
      uint8_t strWidth = len * pContext->font.fontWidth;
      uint8_t posX = (pContext->pDisplayGeometry->xSize - strWidth) >> 1;
      uint8_t posY = ((pContext->font.lineSpacing + pContext->font.fontHeight) * graphLineNum)
                     + pContext->font.lineSpacing;
      GLIB_drawString(pContext, pString, len, posX, posY, 0);
    }
    pString = nextToken;
    /* If the token at the end of the line is new line character, then increase line number */
    if (*nextToken == '\n') {
      graphLineNum++;
      pString++;
    }
  } while (*pString); /* while terminating NULL is not reached */
}
