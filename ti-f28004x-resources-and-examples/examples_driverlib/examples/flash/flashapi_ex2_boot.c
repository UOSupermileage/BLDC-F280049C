//###########################################################################
//
// FILE:    flashapi_ex2_boot.c
//
// TITLE:   Boot Source Code
//
// Functions:
//
//     void   copyData(void)
//     uint32_t getLongData(void)
//     void readReservedFn(void)
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
#include "assert.h"

//
// Globals
//

// getWordData is a pointer to the function that interfaces to the peripheral.
// Each loader assigns this pointer to it's particular getWordData function.
//
uint16fptr getWordData;

typedef struct
{
    uint16_t status;
    uint32_t address;
} StatusCode;
extern StatusCode statusCode;

//
// Because of ECC, must be multiple of 64 bits, or 4 words, BUFFER_SIZE % 4 == 0
//
#define BUFFER_SIZE                 0x80 

//
// Function Prototypes
//
extern void sciSendChecksum(void);
uint32_t getLongData(void);
void copyData(void);
void readReservedFn(void);
uint32_t findSector(uint32_t address);
uint16_t findSize(uint32_t address);
void exampleError(Fapi_StatusType status);

//
// 32(16x2) sectors.
//
unsigned char erasedAlready[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0, 0 };

//
// void CopyData(void) - This routine copies multiple blocks of data from the host
//                       to the specified RAM locations.  There is no error
//                       checking on any of the destination addresses.
//                       That is it is assumed all addresses and block size
//                       values are correct.
//
//                       Multiple blocks of data are copied until a block
//                       size of 00 00 is encountered.
//
void copyData()
{
    struct HEADER
    {
        uint16_t BlockSize;
        uint32_t DestAddr;
    } BlockHeader;

    uint16_t Buffer[BUFFER_SIZE];
    
    //
    // useful for 4-word access to flash with
    //
    uint16_t miniBuffer[4]; 
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t k = 0;
    uint32_t sectorAddress;
    uint16_t sectorSize;
    uint16_t wordData;
    int fail = 0;

    assert(BUFFER_SIZE % 4 == 0);

    if (statusCode.status != NO_ERROR)
    {
        fail++;
    }

    //
    // Reset erase status of all flash sectors
    // 36 sectors.
    //
    for (i = 0; i < 32; i++)
    {
        erasedAlready[i] = 0;
    }

    //
    // Send checksum to satisfy before we begin
    //
#if checksum_enable
    sciSendChecksum();
#endif

    //
    // Get the size in words of the first block
    //
    BlockHeader.BlockSize = (*getWordData)();

    //
    // While the block size is > 0 copy the data
    // to the DestAddr.  There is no error checking
    // as it is assumed the DestAddr is a valid
    // memory location
    //

    EALLOW;
    while (BlockHeader.BlockSize != (uint16_t) 0x0000)
    {
        Fapi_StatusType oReturnCheck;
        Fapi_FlashStatusWordType oFlashStatusWord;
       
        BlockHeader.DestAddr = getLongData();
        for (i = 0; i < BlockHeader.BlockSize; i += 0)
        {
            if (BlockHeader.BlockSize < BUFFER_SIZE)
            {
                for (j = 0; j < BlockHeader.BlockSize; j++)
                {
                    wordData = (*getWordData)();
                    Buffer[j] = wordData;
                    i++;
                }
                for (j = BlockHeader.BlockSize; j < BUFFER_SIZE; j++)
                {
                    Buffer[j] = 0xFFFF;
                }
            }
            
            //
            // BlockHeader.BlockSize >= BUFFER_SIZE
            //
            else 
            {
                //
                // less than one BUFFER_SIZE left
                //
                if ((BlockHeader.BlockSize - i) < BUFFER_SIZE) 
                {
                    //
                    // fill Buffer with rest of data
                    //
                    for (j = 0; j < BlockHeader.BlockSize - i; j++) 
                    {
                        wordData = (*getWordData)();
                        Buffer[j] = wordData;
                    }
                    
                    //
                    // increment i outside here so it doesn't affect loop above
                    //
                    i += j; 
                    
                    //
                    // fill the rest with 0xFFFF
                    //
                    for (; j < BUFFER_SIZE; j++) 
                    {
                        Buffer[j] = 0xFFFF;
                    }
                }
                else
                {
                    //
                    // fill up like normal, up to BUFFER_SIZE
                    //
                    for (j = 0; j < BUFFER_SIZE; j++) 
                    {
                        wordData = (*getWordData)();
                        Buffer[j] = wordData;
                        i++;
                    }
                }
            }
            for (k = 0; k < (BUFFER_SIZE / 4); k++)
            {
                miniBuffer[0] = Buffer[k * 4 + 0];
                miniBuffer[1] = Buffer[k * 4 + 1];
                miniBuffer[2] = Buffer[k * 4 + 2];
                miniBuffer[3] = Buffer[k * 4 + 3];
                
                //
                // check that miniBuffer is not already all erased data
                //
                if (!((miniBuffer[0] == 0xFFFF) && (miniBuffer[1] == 0xFFFF)
                        && (miniBuffer[2] == 0xFFFF)
                        && (miniBuffer[3] == 0xFFFF)))
                {
                    //
                    // clean out flash banks if needed
                    //
                    sectorAddress = findSector(BlockHeader.DestAddr);
                    if (sectorAddress != 0xdeadbeef)
                    {
                        //
                        // FindSize just returns 0x4000. 
                        //
                        sectorSize = findSize(sectorAddress);
                        oReturnCheck = Fapi_issueAsyncCommandWithAddress(
                                Fapi_EraseSector, (uint32_t *) sectorAddress);

                        while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

                        oReturnCheck = Fapi_doBlankCheck(
                                (uint32_t *) sectorAddress, sectorSize,
                                &oFlashStatusWord);
                        if (oReturnCheck != Fapi_Status_Success)
                        {
                            //
                            //first fail
                            //
                            if (fail == 0) 
                            {
                                statusCode.status = BLANK_ERROR;
                                statusCode.address =
                                        oFlashStatusWord.au32StatusWord[0];
                            }
                            fail++;
                        }
                    }
                    
                    //
                    //program 4 words at once, 64-bits
                    //
                    oReturnCheck = Fapi_issueProgrammingCommand(
                            (uint32_t *) BlockHeader.DestAddr, miniBuffer,
                            sizeof(miniBuffer), 0, 0, Fapi_AutoEccGeneration);
                    while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
                        ;
                    if (oReturnCheck != Fapi_Status_Success)
                    {
                        //
                        // first fail
                        //
                        if (fail == 0) 
                        {
                            statusCode.status = PROGRAM_ERROR;
                            statusCode.address =
                                    oFlashStatusWord.au32StatusWord[0];
                        }
                        fail++;
                    }

                    for (j = 0; j < 4; j += 2)
                    {
                        uint32_t toVerify = miniBuffer[j + 1];
                        toVerify = toVerify << 16;
                        toVerify |= miniBuffer[j];
                        oReturnCheck = Fapi_doVerify(
                                (uint32_t *) (BlockHeader.DestAddr + j), 1,
                                (uint32_t *) (&toVerify), &oFlashStatusWord);
                        if (oReturnCheck != Fapi_Status_Success)
                        {
                            //
                            // first fail
                            //
                            if (fail == 0) 
                            {
                                statusCode.status = VERIFY_ERROR;
                                statusCode.address =
                                        oFlashStatusWord.au32StatusWord[0];
                            }
                            fail++;
                        }
                    } //for j; for Fapi_doVerify
                } //check if miniBuffer does not contain all already erased data
                BlockHeader.DestAddr += 0x4;
            } //for(int k); loads miniBuffer with Buffer elements
#if checksum_enable
            sciSendChecksum();
#endif
        }
        //
        // get the size of the next block
        //
        BlockHeader.BlockSize = (*getWordData)();
    }
    EDIS;
    return;
}

//
// uint32_t findSector(uint32_t address) - This routine finds what sector the mentioned address
//                                         is a part of.
//
uint32_t findSector(uint32_t address)
{
    //
    //******************** Bzero_sector0~15_start~End *********************
    //
    if ((address >= Bzero_Sector0_start) && (address <= Bzero_Sector0_End)
            && (erasedAlready[0] == 0))
    {
        erasedAlready[0] = 1;
        return (uint32_t) Bzero_Sector0_start;
    }
    else if ((address >= Bzero_Sector1_start) && (address <= Bzero_Sector1_End)
            && (erasedAlready[1] == 0))
    {
        erasedAlready[1] = 1;
        return (uint32_t) Bzero_Sector1_start;
    }
    else if ((address >= Bzero_Sector2_start) && (address <= Bzero_Sector2_End)
            && (erasedAlready[2] == 0))
    {
        erasedAlready[2] = 1;
        return (uint32_t) Bzero_Sector2_start;
    }
    else if ((address >= Bzero_Sector3_start) && (address <= Bzero_Sector3_End)
            && (erasedAlready[3] == 0))
    {
        erasedAlready[3] = 1;
        return (uint32_t) Bzero_Sector3_start;
    }
    else if ((address >= Bzero_Sector4_start) && (address <= Bzero_Sector4_End)
            && (erasedAlready[4] == 0))
    {
        erasedAlready[4] = 1;
        return (uint32_t) Bzero_Sector4_start;
    }
    else if ((address >= Bzero_Sector5_start) && (address <= Bzero_Sector5_End)
            && (erasedAlready[5] == 0))
    {
        erasedAlready[5] = 1;
        return (uint32_t) Bzero_Sector5_start;
    }
    else if ((address >= Bzero_Sector6_start) && (address <= Bzero_Sector6_End)
            && (erasedAlready[6] == 0))
    {
        erasedAlready[6] = 1;
        return (uint32_t) Bzero_Sector6_start;
    }
    else if ((address >= Bzero_Sector7_start) && (address <= Bzero_Sector7_End)
            && (erasedAlready[7] == 0))
    {
        erasedAlready[7] = 1;
        return (uint32_t) Bzero_Sector7_start;
    }
    else if ((address >= Bzero_Sector8_start) && (address <= Bzero_Sector8_End)
            && (erasedAlready[8] == 0))
    {
        erasedAlready[8] = 1;
        return (uint32_t) Bzero_Sector8_start;
    }
    else if ((address >= Bzero_Sector9_start) && (address <= Bzero_Sector9_End)
            && (erasedAlready[9] == 0))
    {
        erasedAlready[9] = 1;
        return (uint32_t) Bzero_Sector9_start;
    }
    else if ((address >= Bzero_Sector10_start)
            && (address <= Bzero_Sector10_End) && (erasedAlready[10] == 0))
    {
        erasedAlready[10] = 1;
        return (uint32_t) Bzero_Sector10_start;
    }
    else if ((address >= Bzero_Sector11_start)
            && (address <= Bzero_Sector11_End) && (erasedAlready[11] == 0))
    {
        erasedAlready[11] = 1;
        return (uint32_t) Bzero_Sector11_start;
    }
    else if ((address >= Bzero_Sector12_start)
            && (address <= Bzero_Sector12_End) && (erasedAlready[12] == 0))
    {
        erasedAlready[12] = 1;
        return (uint32_t) Bzero_Sector12_start;
    }
    else if ((address >= Bzero_Sector13_start)
            && (address <= Bzero_Sector13_End) && (erasedAlready[13] == 0))
    {
        erasedAlready[13] = 1;
        return (uint32_t) Bzero_Sector13_start;
    }
    else if ((address >= Bzero_Sector14_start)
            && (address <= Bzero_Sector14_End) && (erasedAlready[14] == 0))
    {
        erasedAlready[14] = 1;
        return (uint32_t) Bzero_Sector14_start;
    }
    else if ((address >= Bzero_Sector15_start)
            && (address <= Bzero_Sector15_End) && (erasedAlready[15] == 0))
    {
        erasedAlready[15] = 1;
        return (uint32_t) Bzero_Sector15_start;
    }

    //
    //***************** Bone_Sector0~15_start~End *************************
    //
    else if ((address >= Bone_Sector0_start) && (address <= Bone_Sector0_End)
            && (erasedAlready[16] == 0))
    {
        erasedAlready[16] = 1;
        return (uint32_t) Bone_Sector0_start;
    }
    else if ((address >= Bone_Sector1_start) && (address <= Bone_Sector1_End)
            && (erasedAlready[17] == 0))
    {
        erasedAlready[17] = 1;
        return (uint32_t) Bone_Sector1_start;
    }
    else if ((address >= Bone_Sector2_start) && (address <= Bone_Sector2_End)
            && (erasedAlready[18] == 0))
    {
        erasedAlready[18] = 1;
        return (uint32_t) Bone_Sector2_start;
    }
    else if ((address >= Bone_Sector3_start) && (address <= Bone_Sector3_End)
            && (erasedAlready[19] == 0))
    {
        erasedAlready[19] = 1;
        return (uint32_t) Bone_Sector3_start;
    }
    else if ((address >= Bone_Sector4_start) && (address <= Bone_Sector4_End)
            && (erasedAlready[20] == 0))
    {
        erasedAlready[20] = 1;
        return (uint32_t) Bone_Sector4_start;
    }
    else if ((address >= Bone_Sector5_start) && (address <= Bone_Sector5_End)
            && (erasedAlready[21] == 0))
    {
        erasedAlready[21] = 1;
        return (uint32_t) Bone_Sector5_start;
    }
    else if ((address >= Bone_Sector6_start) && (address <= Bone_Sector6_End)
            && (erasedAlready[22] == 0))
    {
        erasedAlready[22] = 1;
        return (uint32_t) Bone_Sector6_start;
    }
    else if ((address >= Bone_Sector7_start) && (address <= Bone_Sector7_End)
            && (erasedAlready[23] == 0))
    {
        erasedAlready[23] = 1;
        return (uint32_t) Bone_Sector7_start;
    }
    else if ((address >= Bone_Sector8_start) && (address <= Bone_Sector8_End)
            && (erasedAlready[24] == 0))
    {
        erasedAlready[24] = 1;
        return (uint32_t) Bone_Sector8_start;
    }
    else if ((address >= Bone_Sector9_start) && (address <= Bone_Sector9_End)
            && (erasedAlready[25] == 0))
    {
        erasedAlready[25] = 1;
        return (uint32_t) Bone_Sector9_start;
    }
    else if ((address >= Bone_Sector10_start) && (address <= Bone_Sector10_End)
            && (erasedAlready[26] == 0))
    {
        erasedAlready[26] = 1;
        return (uint32_t) Bone_Sector10_start;
    }
    else if ((address >= Bone_Sector11_start) && (address <= Bone_Sector11_End)
            && (erasedAlready[27] == 0))
    {
        erasedAlready[27] = 1;
        return (uint32_t) Bone_Sector11_start;
    }
    else if ((address >= Bone_Sector12_start) && (address <= Bone_Sector12_End)
            && (erasedAlready[28] == 0))
    {
        erasedAlready[28] = 1;
        return (uint32_t) Bone_Sector12_start;
    }
    else if ((address >= Bone_Sector13_start) && (address <= Bone_Sector13_End)
            && (erasedAlready[29] == 0))
    {
        erasedAlready[29] = 1;
        return (uint32_t) Bone_Sector13_start;
    }
    else if ((address >= Bone_Sector14_start) && (address <= Bone_Sector14_End)
            && (erasedAlready[30] == 0))
    {
        erasedAlready[30] = 1;
        return (uint32_t) Bone_Sector14_start;
    }
    else if ((address >= Bone_Sector15_start) && (address <= Bone_Sector15_End)
            && (erasedAlready[31] == 0))
    {
        erasedAlready[31] = 1;
        return (uint32_t) Bone_Sector15_start;
    }
    else
    {
        //
        // a proxy address to signify that it is not a flash sector
        //
        return 0xdeadbeef;
    }
}

//
// uint32_t findSize(uint32_t address) - This routine finds the size of the sector under use.
//
uint16_t findSize(uint32_t address)
{
    //
    // set erasedAlready, all sectors are 8K in 32bits in size.
    //
    return B_8KSector_u32length;  
}

//
// uint32_t getLongData(void) - This routine fetches a 32-bit value from the peripheral
//                              input stream.
//
uint32_t getLongData()
{
    uint32_t longData;

    //
    // Fetch the upper 1/2 of the 32-bit value
    //
    longData = ((uint32_t) (*getWordData)() << 16);

    //
    // Fetch the lower 1/2 of the 32-bit value
    //
    longData |= (uint32_t) (*getWordData)();

    return longData;
}

//
//
// void readReservedFn(void) - This function reads 8 reserved words in the header.
//                             None of these reserved words are used by the
//                             this boot loader at this time, they may be used in
//                             future devices for enhancements.  Loaders that use
//                             these words use their own read function.
//
void readReservedFn()
{
    uint16_t i;

    //
    // Read and discard the 8 reserved words.
    //
    for (i = 1; i <= 8; i++)
    {
        getWordData();
    }
    return;
}
