//###########################################################################
//
// FILE:    flashapi_ex2_erase.c
//
// TITLE:   Erase Source Code
//
// Functions:
//
//     void sharedErase(uint16_t command, uint32_t sectors)
//
//###########################################################################
// $TI Release: F28004x Support Library v1.10.00.00 $
// $Release Date: Tue May 26 17:06:03 IST 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Includes
//
#include "flashapi_ex2_bootrom.h"
#include "flashapi_ex2_commands.h"
#include "F021_F28004x_C28x.h"
#include "flashapi_ex2_flash_programming_c28.h"
#include "flashapi_ex2_erase.h"
#include "Types.h"

//
// Globals
//
typedef struct
{
    uint16_t status;
    uint32_t address;
} StatusCode;
extern StatusCode statusCode;

//
//************ 32(16x2) sectors Bzero/one_Sector0~15_start **********
//
const uint32_t sectAddress[32] = { Bzero_Sector0_start, Bzero_Sector1_start,
                                 Bzero_Sector2_start, Bzero_Sector3_start,
                                 Bzero_Sector4_start, Bzero_Sector5_start,
                                 Bzero_Sector6_start, Bzero_Sector7_start,
                                 Bzero_Sector8_start, Bzero_Sector9_start,
                                 Bzero_Sector10_start, Bzero_Sector11_start,
                                 Bzero_Sector12_start, Bzero_Sector13_start,
                                 Bzero_Sector14_start, Bzero_Sector15_start,
                                 Bone_Sector0_start, Bone_Sector1_start,
                                 Bone_Sector2_start, Bone_Sector3_start,
                                 Bone_Sector4_start, Bone_Sector5_start,
                                 Bone_Sector6_start, Bone_Sector7_start,
                                 Bone_Sector8_start, Bone_Sector9_start,
                                 Bone_Sector10_start, Bone_Sector11_start,
                                 Bone_Sector12_start, Bone_Sector13_start,
                                 Bone_Sector14_start, Bone_Sector15_start };

//
// Function Prototypes
//
void sharedErase(uint32_t sectors);
extern void exampleError(Fapi_StatusType status);
extern void sendACK(void);
extern void sendNAK(void);

//
// void sharedErase(uint32_t sectors) - This routine takes the 32-bit sectors
//                                      variable as a parameter.  Each bit
//                                      corresponds to a sector, starting with
//                                      bit 0 and sector A.  This routine
//                                      attempts to erase the sectors specified.
//
void sharedErase(uint32_t sectors)
{
    statusCode.status = NO_ERROR;
    statusCode.address = 0x12345678;
    int i = 0;
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusWordType oFlashStatusWord;
    int fail = 0;

    EALLOW;
    for (i = 0; i < 32; i++)
    {
        if ((sectors & 0x00000001) == 0x00000001)
        {
            oReturnCheck = Fapi_issueAsyncCommandWithAddress(
                    Fapi_EraseSector, (uint32_t *) (sectAddress[i]));

            //
            // wait until AsyncCommand is done.
            //
            while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
            {
            }

            //
            // All 8K sector size
            //
            oReturnCheck = Fapi_doBlankCheck((uint32_t *) (sectAddress[i]),
            B_8KSector_u32length,
                                             &oFlashStatusWord);
            if (oReturnCheck != Fapi_Status_Success)
            {
                //
                // first fail
                //
                if (fail == 0) 
                {
                    statusCode.status = BLANK_ERROR;
                    statusCode.address = oFlashStatusWord.au32StatusWord[0];
                }
                fail++;
            }

            while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
            {
            }
        }
        sectors = sectors >> 1;
    }
    EDIS;
    return;
}

//
// End of file
//
