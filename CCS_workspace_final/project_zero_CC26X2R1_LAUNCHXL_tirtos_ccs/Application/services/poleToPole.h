/**********************************************************************************************
 * Filename:       poleToPole.h
 *
 * Description:    This file contains the poleToPole service definitions and
 *                 prototypes.
 *
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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


#ifndef _POLETOPOLE_H_
#define _POLETOPOLE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
* CONSTANTS
*/

// Service UUID
#define POLETOPOLE_SERV_UUID 0xD2FF

//  Characteristic defines
#define POLETOPOLE_ALLDEVICES_ID   0
#define POLETOPOLE_ALLDEVICES_UUID 0xA00E
#define POLETOPOLE_ALLDEVICES_LEN  251

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*poleToPoleChange_t)(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

typedef struct
{
  poleToPoleChange_t        pfnChangeCb;  // Called when characteristic value changes
  poleToPoleChange_t        pfnCfgChangeCb;
} poleToPoleCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * PoleToPole_AddService- Initializes the PoleToPole service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t PoleToPole_AddService( uint8_t rspTaskId);

/*
 * PoleToPole_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t PoleToPole_RegisterAppCBs( poleToPoleCBs_t *appCallbacks );

/*
 * PoleToPole_SetParameter - Set a PoleToPole parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t PoleToPole_SetParameter(uint8_t param, uint16_t len, void *value);

/*
 * PoleToPole_GetParameter - Get a PoleToPole parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t PoleToPole_GetParameter(uint8_t param, uint16_t *len, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _POLETOPOLE_H_ */
