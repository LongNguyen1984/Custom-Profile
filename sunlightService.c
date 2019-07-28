/**********************************************************************************************
 * Filename:       sunlightService.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include <icall.h>

#include "sunlightService.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// sunlightService Service UUID
CONST uint8_t sunlightServiceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SUNLIGHTSERVICE_SERV_UUID), HI_UINT16(SUNLIGHTSERVICE_SERV_UUID)
};

// myData UUID
CONST uint8_t sunlightService_MyDataUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SUNLIGHTSERVICE_MYDATA_UUID), HI_UINT16(SUNLIGHTSERVICE_MYDATA_UUID)
};

// UpdatePeriod UUID
CONST uint8_t sunlightService_UpdatePeriodUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SUNLIGHTSERVICE_UPDATEPERIOD_UUID)
};

// Characteristic "UpdatePeriod" Properties (for declaration)
static uint8_t sunlightService_UpdatePeriodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "UpdatePeriod" Value variable
static uint8_t sunlightService_UpdatePeriodVal[SUNLIGHTSERVICE_UPDATEPERIOD_LEN] = {0};

//===========================================
// Characteristic "MyData" Properties (for declaration)
static uint8_t sunlightService_MyDataProps = GATT_PROP_READ|GATT_PROP_NOTIFY;

// Characteristic "MyData" Value
static uint8_t sunlightService_MyDataVal[SUNLIGHTSERVICE_MYDATA_LEN] = {0xFF};

// Characteristic "MyData" CCCD
static gattCharCfg_t *sunlightService_MyDataConfig;
/*********************************************************************
 * LOCAL VARIABLES
 */

static sunlightServiceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t sunlightServiceDecl = { ATT_BT_UUID_SIZE, sunlightServiceUUID };


/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t sunlightServiceAttrTbl[] =
{
  // sunlightService Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&sunlightServiceDecl
  },
  // MyData Characteristic Declaration
      {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &sunlightService_MyDataProps
      },
        // MyData Characteristic Value
        {
          { ATT_BT_UUID_SIZE, sunlightService_MyDataUUID },
          GATT_PERMIT_READ,
          0,
          sunlightService_MyDataVal
        },
        // MyData CCCD
        {
          { ATT_BT_UUID_SIZE, clientCharCfgUUID },
          GATT_PERMIT_READ | GATT_PERMIT_WRITE,
          0,
          (uint8 *)&sunlightService_MyDataConfig
        },
        // UpdatePeriod Characteristic Declaration
           {
             { ATT_BT_UUID_SIZE, characterUUID },
             GATT_PERMIT_READ,
             0,
             &sunlightService_UpdatePeriodProps
           },
           // UpdatePeriod Characteristic Value
           {
            { ATT_UUID_SIZE, sunlightService_UpdatePeriodUUID },
              GATT_PERMIT_READ | GATT_PERMIT_WRITE,
              0,
              sunlightService_UpdatePeriodVal
            },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t sunlightService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t sunlightService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t sunlightServiceCBs =
{
  sunlightService_ReadAttrCB,  // Read callback function pointer
  sunlightService_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * SunlightService_AddService- Initializes the SunlightService service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t SunlightService_AddService( void )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  sunlightService_MyDataConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
   if ( sunlightService_MyDataConfig == NULL )
   {
     return ( bleMemAllocError );
   }
   // Initialize Client Characteristic Configuration attributes
   GATTServApp_InitCharCfg( INVALID_CONNHANDLE, sunlightService_MyDataConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( sunlightServiceAttrTbl,
                                        GATT_NUM_ATTRS( sunlightServiceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &sunlightServiceCBs );

  return ( status );
}

/*
 * SunlightService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t SunlightService_RegisterAppCBs( sunlightServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * SunlightService_SetParameter - Set a SunlightService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t SunlightService_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {

  case SUNLIGHTSERVICE_MYDATA://0
        if ( len == SUNLIGHTSERVICE_MYDATA_LEN )
        {
          memcpy(sunlightService_MyDataVal, value, len);

          // Try to send notification.
          GATTServApp_ProcessCharCfg( sunlightService_MyDataConfig, (uint8_t *)&sunlightService_MyDataVal, FALSE,
                                      sunlightServiceAttrTbl, GATT_NUM_ATTRS( sunlightServiceAttrTbl ),
                                      INVALID_TASK_ID,  sunlightService_ReadAttrCB);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;

  case SUNLIGHTSERVICE_UPDATEPERIOD://1
        if ( len == SUNLIGHTSERVICE_UPDATEPERIOD_LEN )
        {
          memcpy(sunlightService_UpdatePeriodVal, value, len);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;

  default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * SunlightService_GetParameter - Get a SunlightService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t SunlightService_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  case SUNLIGHTSERVICE_UPDATEPERIOD:
        memcpy(value, sunlightService_UpdatePeriodVal, SUNLIGHTSERVICE_UPDATEPERIOD_LEN);
        break;


  default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          sunlightService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t sunlightService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the MyData Characteristic Value
  if ( ! memcmp(pAttr->type.uuid, sunlightService_MyDataUUID, pAttr->type.len) )
    {
      if ( offset > SUNLIGHTSERVICE_MYDATA_LEN )  // Prevent malicious ATT ReadBlob offsets.
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        *pLen = MIN(maxLen, SUNLIGHTSERVICE_MYDATA_LEN - offset);  // Transmit as much as possible
        memcpy(pValue, pAttr->pValue + offset, *pLen);
      }
    }
  /*
  if( ! memcmp(pAttr->type.uuid, sunlightService_MyDataUUID, pAttr->type.len) )
  {
      *pLen = 2;
      memcpy(pValue, pAttr->pValue, *pLen);
  }
  */
  // See if request is regarding the UpdatePeriod Characteristic Value
 else if ( ! memcmp(pAttr->type.uuid, sunlightService_UpdatePeriodUUID, pAttr->type.len) )
   {
     if ( offset > SUNLIGHTSERVICE_UPDATEPERIOD_LEN )  // Prevent malicious ATT ReadBlob offsets.
     {
       status = ATT_ERR_INVALID_OFFSET;
     }
     else
     {
       *pLen = MIN(maxLen, SUNLIGHTSERVICE_UPDATEPERIOD_LEN - offset);  // Transmit as much as possible
       memcpy(pValue, pAttr->pValue + offset, *pLen);
     }
   }
    else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      sunlightService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t sunlightService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                        uint8 *pValue, uint16 len, uint16 offset,
                                        uint8 method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  // See if request is regarding the UpdatePeriod Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, sunlightService_UpdatePeriodUUID, pAttr->type.len) )
    {
      if ( offset + len > SUNLIGHTSERVICE_UPDATEPERIOD_LEN )
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application if entire expected value is written
        if ( offset + len == SUNLIGHTSERVICE_UPDATEPERIOD_LEN)
          paramID = SUNLIGHTSERVICE_UPDATEPERIOD;
      }
    }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb( paramID ); // Call app function from stack task context.

  return status;
}
