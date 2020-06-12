//###########################################################################
//
// FILE:    flashapi_ex2_commands.h
//
// TITLE:   SCI Kernel Commands
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

#ifndef __FLASHAPI_EX2_COMMANDS_H__
#define __FLASHAPI_EX2_COMMANDS_H__

#define DFU_CPU1                    0x0100
#define DFU_CPU2                    0x0200
#define ERASE_CPU1                  0x0300
#define ERASE_CPU2                  0x0400
#define VERIFY_CPU1                 0x0500
#define VERIFY_CPU2                 0x0600
#define LIVE_DFU_CPU1               0x0700
#define CPU1_UNLOCK_Z1              0x000A
#define CPU1_UNLOCK_Z2              0x000B
#define CPU2_UNLOCK_Z1              0x000C
#define CPU2_UNLOCK_Z2              0x000D
#define RUN_CPU1                    0x000E
#define RESET_CPU1                  0x000F
#define RUN_CPU1_BOOT_CPU2          0x0004
#define RESET_CPU1_BOOT_CPU2        0x0007
#define RUN_CPU2                    0x0010
#define RESET_CPU2                  0x0020

//
// undefine from bootrom.h
//
#undef  NO_ERROR 
#define NO_ERROR                    0x1000
#define BLANK_ERROR                 0x2000
#define VERIFY_ERROR                0x3000
#define PROGRAM_ERROR               0x4000
#define COMMAND_ERROR               0x5000
#define UNLOCK_ERROR                0x6000
#define ACK                         0x2D
#define NAK                         0xA5
#define DEFAULT_BAUD                0x2580
#define checksum_enable             1

#endif // __FLASHAPI_EX2_COMMANDS_H__

//
// End of File
//
