/*-
 * Copyright (c) 2003-2012 Broadcom Corporation
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BROADCOM ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL BROADCOM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * #BRCM_2# */


#ifndef _PHONIEX_SEC_H
#define _NETLOGIC_SEC_H

#include <linux/types.h>
#include <asm/netlogic/msgring.h>

enum sec_pipe_config {

  SEC_PIPE_CIPHER_KEY0_L0            = 0x00,
  SEC_PIPE_CIPHER_KEY0_HI,
  SEC_PIPE_CIPHER_KEY1_LO,
  SEC_PIPE_CIPHER_KEY1_HI,
  SEC_PIPE_CIPHER_KEY2_LO,
  SEC_PIPE_CIPHER_KEY2_HI,
  SEC_PIPE_CIPHER_KEY3_LO,
  SEC_PIPE_CIPHER_KEY3_HI,
  SEC_PIPE_HMAC_KEY0_LO,
  SEC_PIPE_HMAC_KEY0_HI,
  SEC_PIPE_HMAC_KEY1_LO,
  SEC_PIPE_HMAC_KEY1_HI,
  SEC_PIPE_HMAC_KEY2_LO,
  SEC_PIPE_HMAC_KEY2_HI,
  SEC_PIPE_HMAC_KEY3_LO,
  SEC_PIPE_HMAC_KEY3_HI,
  SEC_PIPE_HMAC_KEY4_LO,
  SEC_PIPE_HMAC_KEY4_HI,
  SEC_PIPE_HMAC_KEY5_LO,
  SEC_PIPE_HMAC_KEY5_HI,
  SEC_PIPE_HMAC_KEY6_LO,
  SEC_PIPE_HMAC_KEY6_HI,
  SEC_PIPE_HMAC_KEY7_LO,
  SEC_PIPE_HMAC_KEY7_HI,
  SEC_PIPE_NCFBM_LO,
  SEC_PIPE_NCFBM_HI,
  SEC_PIPE_INSTR_LO,
  SEC_PIPE_INSTR_HI,
  SEC_PIPE_RSVD0,
  SEC_PIPE_RSVD1,
  SEC_PIPE_RSVD2,
  SEC_PIPE_RSVD3,

  SEC_PIPE_DF_PTRS0,
  SEC_PIPE_DF_PTRS1,
  SEC_PIPE_DF_PTRS2,
  SEC_PIPE_DF_PTRS3,
  SEC_PIPE_DF_PTRS4,
  SEC_PIPE_DF_PTRS5,
  SEC_PIPE_DF_PTRS6,
  SEC_PIPE_DF_PTRS7,

  SEC_PIPE_DU_DATA_IN_LO,
  SEC_PIPE_DU_DATA_IN_HI,
  SEC_PIPE_DU_DATA_IN_CTRL,
  SEC_PIPE_DU_DATA_OUT_LO,
  SEC_PIPE_DU_DATA_OUT_HI,
  SEC_PIPE_DU_DATA_OUT_CTRL,

  SEC_PIPE_STATE0,
  SEC_PIPE_STATE1,
  SEC_PIPE_STATE2,
  SEC_PIPE_STATE3,
  SEC_PIPE_STATE4,
  SEC_PIPE_INCLUDE_MASK0,
  SEC_PIPE_INCLUDE_MASK1,
  SEC_PIPE_INCLUDE_MASK2,
  SEC_PIPE_INCLUDE_MASK3,
  SEC_PIPE_INCLUDE_MASK4,
  SEC_PIPE_EXCLUDE_MASK0,
  SEC_PIPE_EXCLUDE_MASK1,
  SEC_PIPE_EXCLUDE_MASK2,
  SEC_PIPE_EXCLUDE_MASK3,
  SEC_PIPE_EXCLUDE_MASK4,
};

enum sec_pipe_base_config {
  
  SEC_PIPE0_BASE = 0x00,
  SEC_PIPE1_BASE = 0x40,
  SEC_PIPE2_BASE = 0x80,
  SEC_PIPE3_BASE = 0xc0

};

enum sec_rsa_config {
  
  SEC_RSA_PIPE0_DU_DATA_IN_LO = 0x100,
  SEC_RSA_PIPE0_DU_DATA_IN_HI,
  SEC_RSA_PIPE0_DU_DATA_IN_CTRL,
  SEC_RSA_PIPE0_DU_DATA_OUT_LO,
  SEC_RSA_PIPE0_DU_DATA_OUT_HI,
  SEC_RSA_PIPE0_DU_DATA_OUT_CTRL,
  SEC_RSA_RSVD0,
  SEC_RSA_RSVD1,

  SEC_RSA_PIPE0_STATE0,
  SEC_RSA_PIPE0_STATE1,
  SEC_RSA_PIPE0_STATE2,
  SEC_RSA_PIPE0_INCLUDE_MASK0,
  SEC_RSA_PIPE0_INCLUDE_MASK1,
  SEC_RSA_PIPE0_INCLUDE_MASK2,
  SEC_RSA_PIPE0_EXCLUDE_MASK0,
  SEC_RSA_PIPE0_EXCLUDE_MASK1,
  SEC_RSA_PIPE0_EXCLUDE_MASK2,
  SEC_RSA_PIPE0_EVENT_CTR

};

enum sec_config {
  
  SEC_DMA_CREDIT = 0x140,
  SEC_CONFIG1,
  SEC_CONFIG2,
  SEC_CONFIG3,  

};

enum sec_debug_config {
  
  SEC_DW0_DESCRIPTOR0_LO  = 0x180,
  SEC_DW0_DESCRIPTOR0_HI,
  SEC_DW0_DESCRIPTOR1_LO,
  SEC_DW0_DESCRIPTOR1_HI,
  SEC_DW1_DESCRIPTOR0_LO,
  SEC_DW1_DESCRIPTOR0_HI,
  SEC_DW1_DESCRIPTOR1_LO,
  SEC_DW1_DESCRIPTOR1_HI,
  SEC_DW2_DESCRIPTOR0_LO,
  SEC_DW2_DESCRIPTOR0_HI,
  SEC_DW2_DESCRIPTOR1_LO,
  SEC_DW2_DESCRIPTOR1_HI,
  SEC_DW3_DESCRIPTOR0_LO,
  SEC_DW3_DESCRIPTOR0_HI,
  SEC_DW3_DESCRIPTOR1_LO,
  SEC_DW3_DESCRIPTOR1_HI,

  SEC_STATE0,
  SEC_STATE1,  
  SEC_STATE2,
  SEC_INCLUDE_MASK0,
  SEC_INCLUDE_MASK1,
  SEC_INCLUDE_MASK2,
  SEC_EXCLUDE_MASK0,
  SEC_EXCLUDE_MASK1,
  SEC_EXCLUDE_MASK2,
  SEC_EVENT_CTR

};

//enum sec_perf_config {
  
//  SEC_PERF0  = 0x1c0

//};

enum sec_msgring_bucket_config {

  SEC_BIU_CREDITS = 0x308,
  
  SEC_MSG_BUCKET0_SIZE = 0x320,
  SEC_MSG_BUCKET1_SIZE,
  SEC_MSG_BUCKET2_SIZE,
  SEC_MSG_BUCKET3_SIZE,
  SEC_MSG_BUCKET4_SIZE,
  SEC_MSG_BUCKET5_SIZE,
  SEC_MSG_BUCKET6_SIZE,
  SEC_MSG_BUCKET7_SIZE,
};

enum sec_msgring_credit_config {

  SEC_CC_CPU0_0                        = 0x380,
  SEC_CC_CPU1_0                        = 0x388,
  SEC_CC_CPU2_0                        = 0x390,
  SEC_CC_CPU3_0                        = 0x398,
  SEC_CC_CPU4_0                        = 0x3a0,
  SEC_CC_CPU5_0                        = 0x3a8,
  SEC_CC_CPU6_0                        = 0x3b0,
  SEC_CC_CPU7_0                        = 0x3b8

};

enum sec_engine_id {
  SEC_PIPE0,
  SEC_PIPE1,
  SEC_PIPE2,
  SEC_PIPE3,
  SEC_RSA
};

enum sec_cipher {
  SEC_AES256_MODE_HMAC,
  SEC_AES256_MODE,
  SEC_AES256_HMAC,
  SEC_AES256,
  SEC_AES192_MODE_HMAC,
  SEC_AES192_MODE,
  SEC_AES192_HMAC,
  SEC_AES192,
  SEC_AES128_MODE_HMAC,
  SEC_AES128_MODE,
  SEC_AES128_HMAC,
  SEC_AES128,
  SEC_DES_HMAC,
  SEC_DES,
  SEC_3DES,
  SEC_3DES_HMAC,
  SEC_HMAC
};

enum sec_msgrng_msg_ctrl_config {
  SEC_EOP=5,
  SEC_SOP=6,
};

/*
 * Security block data and control exchange
 *
 * A 2-word message ring descriptor is used to pass a pointer to the control descriptor data structure
 * and a pointer to the packet descriptor data structure:
 *
 *  63  61 60    54      53      52    48 47            45 44    40 39                                                     5 4      0
 *  ---------------------------------------------------------------------------------------------------------------------------------
 * | Ctrl | FREEBACKID | IF_L2ALLOC | UNUSED | Control Length | UNUSED | 35 MSB of address of control descriptor data structure | UNUSED |
 *  ---------------------------------------------------------------------------------------------------------------------------------
 *    3       7          1          5             3           5                              35                                 5
 *
 *  63  61 60    54     53          52             51        50    46      45       44    40 39                                                    5 4      0
 *  ---------------------------------------------------------------------------------------------------------------------------------------------------------
 * | Ctrl | FREEBACKID | WRB_COH | WRB_L2ALLOC | DF_PTR_L2ALLOC | UNUSED | Data Length | UNUSED | 35 MSB of address of packet descriptor data structure | UNUSED |
 *  ---------------------------------------------------------------------------------------------------------------------------------------------------------
 *    3       7         1          1               1            5           1          5                                35                              5
 *
 * Addresses assumed to be cache-line aligned, i.e., Address[4:0] ignored (using 5'h00 instead)
 *
 * Control length is the number of control cachelines to be read so user needs to round up
 * the control length to closest integer multiple of 32 bytes. Note that at present (03/18/04)
 * the longest (sensical) ctrl structure is <= 128 bytes.
 *
 * The packet descriptor data structure size is fixed at 1 cacheline (32 bytes).
 * This effectively makes "Data Length" a Load/NoLoad bit. NoLoad causes an abort.
 *
 *
 * Upon completion of operation, the security block returns a 2-word free descriptor
 * in the following format:
 *
 *  63  61 60            54 53   52 51       49   48   47               40 39                                                  0
 *  ----------------------------------------------------------------------------------------------------------------------------
 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl | 1'b0 | Instruction Error |    Address of control descriptor data structure     |
 *  ----------------------------------------------------------------------------------------------------------------------------
 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl | 1'b0 |     Data Error    |    Address of packet descriptor data structure      |
 *  ----------------------------------------------------------------------------------------------------------------------------
 *
 * The Instruction and Data Error codes are enumerated in the 
 * ControlDescriptor and PacketDescriptor sections below
 *
 *
 *                                              /----------------------------------------\
 *                                              |                                        |
 *                                              |   ControlDescriptor_s datastructure    |
 *                                              |                                        |
 *                                              \----------------------------------------/
 *
 *
 *       ControlDescriptor_t.Instruction
 *       -------------------------------
 *
 *   63        37   36   35   34    32 31  29 28         27 26           24
 *  ------------------------------------------------------------------------
 * ||   UNUSED   || V || E/D | Cipher | Mode | Init_Cipher | Cipher_Offset ||   ... CONT ...
 *  ------------------------------------------------------------------------
 *        28        1     1      3       3          2              3        
 *               <CTRL><--------------------CIPHER----------------------->
 *
 *
 *     23   22  21      20     19         18     17        16    15   14 13           2 1         0
 *  -----------------------------------------------------------------------------------------------
 * || HMAC | Hash | Init_Hash | Hash_Offset | Hash_Src || CkSum |  N/U  | CkSum_Offset | CkSum_Src ||
 *  -----------------------------------------------------------------------------------------------
 *     1      2         1            2            1         1       2         12            2
 *   <-----------------------HASH--------------------->  <-----------CHECKSUM-------------------->
 *
 *
 *
 *      CTRL.V                 =        1'b0       Instruction invalid
 *                                      1'b1       Instruction valid
 *      CIPHER.E/D             =        1'b0       Decrypt
 *                                      1'b1       Encrypt
 *             Cipher          =        3'b000     Bypass
 *                                      3'b001     DES
 *                                      3'b010     3DES
 *                                      3'b011     AES 128-bit key
 *                                      3'b100     AES 192-bit key
 *                                      3'b101     AES 256-bit key
 *                                      Remainder  UNDEFINED
 *             Mode            =        3'b000     ECB
 *                                      3'b001     CBC
 *                                      3'b010     CFB (AES only, otherwise undefined)
 *                                      3'b011     OFB (AES only, otherwise undefined)
 *                                      3'b100     CTR (AES only, otherwise undefined)
 *                                      Remainder  UNDEFINED
 *             Init_Cipher     =        2'b00      Preserve old IV/(Keys,NonceCFBMask)
 *                                      2'b01      Load new IV use old Keys,NonceCFBMask
 *                                      2'b10      Load new Keys,NonceCFBMask use old IV (?)
 *                                      2'b11      Load new IV/(Keys,NonceCFBMask)
 *             Cipher_Offset   =                   Nb of words between the first data segment 
 *                                                 and word on which to start cipher operation
 *                                                 (64 BIT WORDS !!!)
 *        HASH.HMAC            =        1'b0       Hash without HMAC
 *                                      1'b1       Hash with HMAC 
 *             Hash            =        2'b00      Hash NOP
 *                                      2'b01      MD5
 *                                      2'b10      SHA-1
 *                                      2'b11      SHA-256
 *             Init_Hash       =        1'b0       Preserve old key HMAC key stored in ID registers (moot if HASH.HMAC == 0)
 *                                      1'b1       Load new HMAC key from memory ctrl section to ID registers
 *             Hash_Offset     =                   Nb of words between the first data segment
 *                                                 and word on which to start hashing 
 *                                                 (64 bit words)
 *             Hash_Src        =        1'b0       DMA channel
 *                                      1'b1       Cipher if word count exceeded Cipher_Offset; 
 *                                                 DMA channel otherwise
 *    CHECKSUM.CkSum           =        1'b0       CkSum NOP
 *                                      1'b1       INTERNET_CHECKSUM
 *             N/U             =        2'bx       Field not used
 *             CkSum_Offset    =                   Nb of words between the first data segment 
 *                                                 and word on which to start 
 *                                                 checksum calculation (32 BIT WORDS !!!)
 *             CkSum_Src       =        2'b00      DMA channel if word count exceeded CkSum_Offset
 *                                      2'b01      Cipher if word count exceeded CkSum_Offset,
 *                                      2'b10      UNDEFINED
 *                                      2'b11      UNDEFINED
 *
 *
 *             OLD !!!
 *             CkSum_Src       =        2'b00      0
 *                                      2'b01      Cipher if word count exceeded CkSum_Offset,
 *                                                 0 otherwise
 *                                      2'b10      DMA channel if word count exceeded 
 *                                                 CkSum_Offset, 0 otherwise
 *                                      2'b11      UNDEFINED
 *
 *
 *       ControlDescriptor_t.cipherHashInfo.infoAES256ModeHMAC
 *       -----------------------------------------------------
 *  
 *  -----------------------------------------------------------------
 * ||63              AES Key0                                      0||
 *  -----------------------------------------------------------------
 *                   .
 *                   .
 *                   .
 *  -----------------------------------------------------------------
 * ||63              AES Key3                                      0||
 *  -----------------------------------------------------------------
 *                   .
 *                   .
 *                   .
 *  -----------------------------------------------------------------
 * ||63              HMAC Key0                                      0||
 *  -----------------------------------------------------------------
 *                   .
 *                   .
 *                   .
 *  -----------------------------------------------------------------
 * ||63              HMAC Key7                                      0||
 *  -----------------------------------------------------------------
 *
 *   63        40  39                   8  7                       0
 *  -----------------------------------------------------------------
 * ||   UNUSED   || Nonce (AES/CTR only) || CFB_Mask (AES/CFB only) ||
 *  -----------------------------------------------------------------
 *        24                 32                       8
 *
 */


/*                                              /--------------------------------------------\
 *                                              |                                            |
 *                                              |    New PacketDescriptor_s datastructure    |
 *                                              |                                            |
 *                                              \--------------------------------------------/
 *
 *
 *       PacketDescriptor_t.srcLengthIVOffUseIVNext
 *       ------------------------------------------
 *
 *           63           62      61             59    58        57    56       54  53           43  42    40  39                  5  4      3  2                      0
 *  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * || Load HMAC key || Pad Hash || Hash Byte Count || Next || Use IV || IV Offset || Packet length || UNUSED || Segment src address || UNUSED || Global src data offset || 
 *  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *           1            1           3                1        1          3              11            3             35                 2                 3
 *
 *
 *             Load HMAC key           =        1'b0       Preserve old HMAC key stored in Auth engine (moot if HASH.HMAC == 0)
 *                                              1'b1       Load HMAC key from ID registers at beginning of op
 *             Pad Hash                =        1'b0       HASH will assume the data was padded to be a multiple
 *                                                         of 512 bits in length and that the last 64 bit word
 *                                                         expresses the total datalength in bits seen by HASH engine
 *                                              1'b1       The data was not padded to be a multiple of 512 bits in length;
 *                                                         The Hash engine will do its own padding to generate the correct digest.
 *             Hash Byte Count                             Number of BYTES on last 64-bit data word to use in digest calculation RELEVANT ONLY IF Pad Hash IS SET
 *                                              3'b000     Use all 8
 *                                              3'b001     Use first (MS) byte only (0-out rest), i.e., 0xddXXXXXXXXXXXXXX
 *                                              3'b010     Use first 2 bytes only (0-out rest), i.e., 0xddddXXXXXXXXXXXX     ... etc
 *             Next                    =        1'b0       Finish (return msg descriptor) at end of operation
 *                                              1'b1       Grab the next PacketDescriptor (i.e. next cache-line) - NOT YET IMPLEMENTED !!!
 *             Use IV                  =        1'b0       Use old IV
 *                                              1'b1       Use data @ Segment_address + IV_Offset as IV
 *             IV Offset               =                   Offset IN NB OF 8 BYTE WORDS from beginning of packet
 *                                                         (i.e. (Potentially shifted) Segment address) to cipher IV
 *             Packet length           =                   Nb double words to stream in (Including Segment address->CP/IV/Auth/CkSum offsets)
 *                                                         This is the total amount of data (x8 in bytes) read    (+1 dword if "Global src data offset" != 0)
 *                                                         This is the total amount of data (x8 in bytes) written (+1 dword if "Global dst data offset" != 0)
 *                                                         (0-7); allows realignment of byte-aligned, non-double-word aligned data
 *                                                         If Packet length == 11'h7ff and (Global src data offset != 0 or Global dst data offset != 0)
 *                                                         the operation is aborted (no mem writes occur)
 *             Segment src address     =                   35 MSB of pointer to src data (i.e., cache-line aligned)
 *             Global src data offset  =                   Nb BYTES to right-shift data by before presenting it to engines
 *                                                         (0-7); allows realignment of byte-aligned, non-double-word aligned data
 *
 *       PacketDescriptor_t.dstLLWMask
 *       -----------------------------
 *
 *   63    60  59                 58  57    40  39                 5  4      3  2                      0
 *  -----------------------------------------------------------------------------------------------------
 * || UNUSED || Last long word mask || UNUSED || Cipher dst address || UNUSED || Global dst data offset ||
 *  -----------------------------------------------------------------------------------------------------
 *      4                2               18              35              2                 3
 *
 *
 *             Last long word mask     =   2'b00      Give last 128 bit word to AES/HAMC/CKSUM engines as is
 *                                         2'b11      Mask (zero-out) 32 least significant bits
 *                                         2'b10      Mask 64 LSBs
 *                                         2'b01      Mask 96 LSBs
 *             Cipher dst address      =              35 MSB of pointer to dst location (i.e., cache-line aligned)
 *             Global dst data offset  =              Nb BYTES to left-shift (double-word boundary aligned) data by before writing it to memory
 *
 * Upon completion of operation, the Sec block returns a 2-word free descriptor
 * in the following format:
 *
 *  63  61 60            54 53   52 51       49  48          40 39             0
 *  ----------------------------------------------------------------------------
 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl | Control Error | Source Address |
 *  ----------------------------------------------------------------------------
 * | Ctrl | Destination Id | 2'b00 | Desc Ctrl |   Data Error  | Dest Address   |
 *  ----------------------------------------------------------------------------
 *
 * The Control and Data Error codes are enumerated below
 *

*
 * Component strcts and unions defining CipherHashInfo_u
 */

/* All AES256 possibilities */
 /* AES256, (CTR or CFB),    HMAC (MD5, SHA-1, SHA-256)      - 104 bytes */
typedef struct AES256ModeHMAC_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
  uint64_t             cipherKey3;
  uint64_t             hmacKey0;
  uint64_t             hmacKey1;
  uint64_t             hmacKey2;
  uint64_t             hmacKey3;
  uint64_t             hmacKey4;
  uint64_t             hmacKey5;
  uint64_t             hmacKey6;
  uint64_t             hmacKey7;
  uint64_t             nonceCFBMask;
} AES256ModeHMAC_t, *AES256ModeHMAC_pt;

/* AES256, (CTR or CFB),    Non-HMAC (MD5, SHA-1, SHA-256)  - 40  bytes */
typedef struct AES256Mode_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
  uint64_t             cipherKey3;
  uint64_t             nonceCFBMask;
} AES256Mode_t, *AES256Mode_pt;

/* AES256, (ECB, CBC, OFB), HMAC (MD5, SHA-1, SHA-256)      - 96  bytes */
typedef struct AES256HMAC_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
  uint64_t             cipherKey3;
  uint64_t             hmacKey0;
  uint64_t             hmacKey1;
  uint64_t             hmacKey2;
  uint64_t             hmacKey3;
  uint64_t             hmacKey4;
  uint64_t             hmacKey5;
  uint64_t             hmacKey6;
  uint64_t             hmacKey7;
} AES256HMAC_t, *AES256HMAC_pt;

/* AES256, (ECB, CBC, OFB), Non-HMAC (MD5, SHA-1, SHA-256)  - 32  bytes */
typedef struct AES256_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
  uint64_t             cipherKey3;
} AES256_t, *AES256_pt;


/* All AES192 possibilities */

/* AES192, (CTR or CFB),    HMAC (MD5, SHA-1, SHA-192)      - 96  bytes */
typedef struct AES192ModeHMAC_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
  uint64_t             hmacKey0;
  uint64_t             hmacKey1;
  uint64_t             hmacKey2;
  uint64_t             hmacKey3;
  uint64_t             hmacKey4;
  uint64_t             hmacKey5;
  uint64_t             hmacKey6;
  uint64_t             hmacKey7;
  uint64_t             nonceCFBMask;
} AES192ModeHMAC_t, *AES192ModeHMAC_pt;

/* AES192, (CTR or CFB),    Non-HMAC (MD5, SHA-1, SHA-192)  - 32  bytes */
typedef struct AES192Mode_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
  uint64_t             nonceCFBMask;
} AES192Mode_t, *AES192Mode_pt;

/* AES192, (ECB, CBC, OFB), HMAC (MD5, SHA-1, SHA-192)      - 88  bytes */
typedef struct AES192HMAC_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
  uint64_t             hmacKey0;
  uint64_t             hmacKey1;
  uint64_t             hmacKey2;
  uint64_t             hmacKey3;
  uint64_t             hmacKey4;
  uint64_t             hmacKey5;
  uint64_t             hmacKey6;
  uint64_t             hmacKey7;
} AES192HMAC_t, *AES192HMAC_pt;

/* AES192, (ECB, CBC, OFB), Non-HMAC (MD5, SHA-1, SHA-192)  - 24  bytes */
typedef struct AES192_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
} AES192_t, *AES192_pt;


/* All AES128 possibilities */

/* AES128, (CTR or CFB),    HMAC (MD5, SHA-1, SHA-128)      - 88  bytes */
typedef struct AES128ModeHMAC_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             hmacKey0;
  uint64_t             hmacKey1;
  uint64_t             hmacKey2;
  uint64_t             hmacKey3;
  uint64_t             hmacKey4;
  uint64_t             hmacKey5;
  uint64_t             hmacKey6;
  uint64_t             hmacKey7;
  uint64_t             nonceCFBMask;
} AES128ModeHMAC_t, *AES128ModeHMAC_pt;

/* AES128, (CTR or CFB),    Non-HMAC (MD5, SHA-1, SHA-128)  - 24  bytes */
typedef struct AES128Mode_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             nonceCFBMask;
} AES128Mode_t, *AES128Mode_pt;

/* AES128, (ECB, CBC, OFB), HMAC (MD5, SHA-1, SHA-128)      - 80  bytes */
typedef struct AES128HMAC_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             hmacKey0;
  uint64_t             hmacKey1;
  uint64_t             hmacKey2;
  uint64_t             hmacKey3;
  uint64_t             hmacKey4;
  uint64_t             hmacKey5;
  uint64_t             hmacKey6;
  uint64_t             hmacKey7;
} AES128HMAC_t, *AES128HMAC_pt;

/* AES128, (ECB, CBC, OFB), Non-HMAC (MD5, SHA-1, SHA-128)  - 16  bytes */
typedef struct AES128_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
} AES128_t, *AES128_pt;


/* All DES possibilities */

/* DES, (ECB, CBC), HMAC (MD5, SHA-1, SHA-128)              - 72  bytes */
typedef struct DESHMAC_s {
  uint64_t             cipherKey0;
  uint64_t             hmacKey0;
  uint64_t             hmacKey1;
  uint64_t             hmacKey2;
  uint64_t             hmacKey3;
  uint64_t             hmacKey4;
  uint64_t             hmacKey5;
  uint64_t             hmacKey6;
  uint64_t             hmacKey7;
} DESHMAC_t, *DESHMAC_pt;

/* DES, (ECB, CBC), Non-HMAC (MD5, SHA-1, SHA-128)          - 9   bytes */
typedef struct DES_s {
  uint64_t             cipherKey0;
} DES_t, *DES_pt;


/* All 3DES possibilities */

/* 3DES, (ECB, CBC), HMAC (MD5, SHA-1, SHA-128)             - 88  bytes */
typedef struct TriDESHMAC_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
  uint64_t             hmacKey0;
  uint64_t             hmacKey1;
  uint64_t             hmacKey2;
  uint64_t             hmacKey3;
  uint64_t             hmacKey4;
  uint64_t             hmacKey5;
  uint64_t             hmacKey6;
  uint64_t             hmacKey7;
} TriDESHMAC_t, *TriDESHMAC_pt;

/* 3DES, (ECB, CBC), Non-HMAC (MD5, SHA-1, SHA-128)         - 24  bytes */
typedef struct TriDES_s {
  uint64_t             cipherKey0;
  uint64_t             cipherKey1;
  uint64_t             cipherKey2;
} TriDES_t, *TriDES_pt;


/* HMAC only - no cipher */

/* HMAC (MD5, SHA-1, SHA-128)                               - 64  bytes */
typedef struct HMAC_s {
  uint64_t             hmacKey0;
  uint64_t             hmacKey1;
  uint64_t             hmacKey2;
  uint64_t             hmacKey3;
  uint64_t             hmacKey4;
  uint64_t             hmacKey5;
  uint64_t             hmacKey6;
  uint64_t             hmacKey7;
} HMAC_t, *HMAC_pt;

typedef union CipherHashInfo_u {
  AES256ModeHMAC_t     infoAES256ModeHMAC;
  AES256Mode_t         infoAES256Mode;
  AES256HMAC_t         infoAES256HMAC;
  AES256_t             infoAES256;
  AES192ModeHMAC_t     infoAES192ModeHMAC;
  AES192Mode_t         infoAES192Mode;
  AES192HMAC_t         infoAES192HMAC;
  AES192_t             infoAES192;
  AES128ModeHMAC_t     infoAES128ModeHMAC;
  AES128Mode_t         infoAES128Mode;
  AES128HMAC_t         infoAES128HMAC;
  AES128_t             infoAES128;
  DESHMAC_t            infoDESHMAC;
  DES_t                infoDES;
  TriDESHMAC_t         info3DESHMAC;
  TriDES_t             info3DES;
  HMAC_t               infoHMAC;
  uint64_t             infoDwords[12];
} CipherHashInfo_t, *CipherHashInfo_pt;

/*
 * This defines the security control descriptor format
 */
typedef struct ControlDescriptor_s {
  uint64_t            instruction;
  CipherHashInfo_t    cipherHashInfo;
} ControlDescriptor_t, *ControlDescriptor_pt;

/*
 * This defines the security data descriptor format
 */
typedef struct PacketDescriptor_s {
  uint64_t             srcLengthIVOffUseIVNext;
  uint64_t             dstLLWMask;
  uint64_t             authDst;
  uint64_t             ckSumDst;
} PacketDescriptor_t, *PacketDescriptor_pt;

#define SEC_CTRL_ERR_NONE                    0x000
#define SEC_CTRL_ERR_UNKNOWN_CIPHER_OP       0x001
#define SEC_CTRL_ERR_ILLEGAL_MODE            0x002
#define SEC_CTRL_ERR_UNSUPP_CKSUM_SRC        0x004
#define SEC_CTRL_ERR_FORBIDDEN_CFB_MASK      0x008
#define SEC_CTRL_ERR_UNKNOWN_CTRL_OP         0x010
#define SEC_CTRL_ERR_DATA_READ               0x080
#define SEC_CTRL_ERR_DESC_CTRL_FIELD         0x100

#define SEC_PKT_ERR_NONE                     0x000
#define SEC_PKT_ERR_INSUFF_DATA_TO_CIPHER    0x001
#define SEC_PKT_ERR_ILLEGAL_IV_LOCATION      0x002
#define SEC_PKT_ERR_ILLEGAL_WORDCOUNT_AES    0x004
#define SEC_PKT_ERR_ILLEGAL_PAD_BYTECOUNT_SPEC 00x8
#define SEC_PKT_ERR_INSUFF_DATA_TO_CKSUM       0x010
#define SEC_PKT_ERR_UNKNOWN_DATA_OP            0x020
#define SEC_PKT_ERR_INSUFF_DATA_TO_AUTH        0x040
#define SEC_PKT_ERR_DATA_READ                  0x080

#define HASH_NOP    0
#define HASH_MD5    1
#define HASH_SHA1   2
#define HASH_SHA256 3

static __inline__ int make_sec_desc(struct msgrng_msg *msg, void *ctrl_addr, int ctrl_len, void *data_addr)
{
  int stid = MSGRNG_STNID_SEC0;
  msg->msg0 = ( ((uint64_t)SEC_SOP<<61)  | 
		((uint64_t)4<<54) |
		((uint64_t)ctrl_len<<45) |
		((uint64_t)virt_to_phys(ctrl_addr) & 0xffffffffe0ULL)
		);
  msg->msg1 = ( ((uint64_t)SEC_EOP<<61)  | 
		((uint64_t)4<<54) |
		((uint64_t)1<<45) |
		((uint64_t)virt_to_phys(data_addr) & 0xffffffffe0ULL)
		);
  msg->msg2 = msg->msg3 = 0;

  return stid;
}

#endif
