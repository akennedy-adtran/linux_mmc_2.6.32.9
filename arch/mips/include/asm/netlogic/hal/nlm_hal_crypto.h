/*-
 * Copyright (c) 2003-2013 Broadcom Corporation
 * All Rights Reserved
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * http://www.gnu.org/licenses/gpl-2.0.txt
 * or the Broadcom license below:

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
 * #BRCM_4# */

#ifndef _NLM_HAL_CRYPTO_H_
#define _NLM_HAL_CRYPTO_H_

#define shift_lower_bits(x, bitshift, numofbits) ((unsigned long long)(x) << (bitshift))

#define shift_lower_bits_mask(x, bitshift, numofbits) (((unsigned long long)(x) & ((1ULL << (numofbits)) - 1)) << (bitshift))

/**
* @file_name nlm_hal_crypto.c
*/

/**
* @defgroup crypto  Crypto HAL apis
* @brief Description about the crypto HAL level apis for RSA/ECC and SAE engines
*/

/**
* @brief Generate crypto rsa/ecc fmn message entry 0
* @ingroup crypto
* - l3alloc: 1 casuses source data to transit through l3 cache
* - type: ecc prime/ecc bin/ me/ micorcode load
* - func: point mul/add etc
* - srcaddr : source address
*/
static inline unsigned long long nlm_crypto_form_rsa_ecc_fmn_entry0(unsigned int l3alloc, unsigned int type, 
		unsigned int func, unsigned long long srcaddr)
{
	return shift_lower_bits(l3alloc, 61, 1) | 
		shift_lower_bits(type, 46, 7) |
		shift_lower_bits(func, 40, 6) |
		shift_lower_bits(srcaddr, 0, 40);
}

/**
* @brief Generate crypto rsa/ecc fmn message entry 1
* @ingroup crypto
* - dstclobber: 1 causes data to be written as 64byte cacheline
* - l3alloc: 1 caused data written to the dram is also copied to l3 cache
* - fbvc: freeback message vc
* - dstaddr : destination address
*/
static inline unsigned long long nlm_crypto_form_rsa_ecc_fmn_entry1(unsigned int dstclobber, unsigned int l3alloc, 
		unsigned int fbvc, unsigned long long dstaddr)
{
	return shift_lower_bits(dstclobber, 62, 1) |
		shift_lower_bits(l3alloc, 61, 1) |
		shift_lower_bits(fbvc, 40, 12) |
		shift_lower_bits(dstaddr, 0, 40);
}

/**
* @brief Generate crypto control descriptor
* @ingroup crypto
* - hmac : 1 for hash with hmac 
* - hashalg: see hash_alg enums
* - hashmode: see hash_mode enums
* - cipherhalg: see  cipher_alg enums
* - ciphermode: see  cipher_mode enums
* - arc4_cipherkeylen : length of arc4 cipher key, 0 is interpreted as 32 
* - arc4_keyinit : 
* - cfbmask : cipher text for feedback, 
*           0(1 bit), 1(2 bits), 2(4 bits), 3(8 bits), 4(16bits), 5(32 bits), 6(64 bits), 7(128 bits)
*/
static inline unsigned long long nlm_crypto_form_pkt_ctrl_desc(unsigned int hmac, 
		unsigned int hashalg,  unsigned int hashmode,
		unsigned int cipheralg, unsigned int ciphermode,
		unsigned int arc4_cipherkeylen, unsigned int arc4_keyinit, unsigned int cfbmask)
{
	return shift_lower_bits(hmac, 61, 1) | 
		shift_lower_bits(hashalg, 52, 8) | 
		shift_lower_bits(hashmode, 43, 8) | 
		shift_lower_bits(cipheralg, 34, 8) | 
		shift_lower_bits(ciphermode, 25, 8) | 
		shift_lower_bits(arc4_cipherkeylen, 18, 5) | 
		shift_lower_bits(arc4_keyinit, 17, 1) | 
		shift_lower_bits(cfbmask, 0, 3);
}
/**
* @brief Generate crypto packet descriptor 0
* @ingroup crypto
* - tls : 1(tls enabled) 0(tls disabled)
* - hash_source : 1(encrypted data is sent to the auth engine) 0(plain data is sent to the auth engine)
* - hashout_l3alloc : 1(auth output is transited through l3 cache)
* - encrypt : 1(for encrypt) 0(for decrypt)
* - ivlen : iv length in bytes
* - hashdst_addr : hash out physical address, byte aligned
*/
static inline unsigned long long nlm_crypto_form_pkt_desc0(unsigned int tls, unsigned int hash_source, 
		unsigned int hashout_l3alloc,
		unsigned int encrypt, unsigned int ivlen, unsigned long long hashdst_addr)
{
	return (shift_lower_bits(tls, 63, 1) |
		shift_lower_bits(hash_source, 62, 1) |
		shift_lower_bits(hashout_l3alloc, 60, 1) |
		shift_lower_bits(encrypt, 59, 1) |
		shift_lower_bits_mask((ivlen - 1), 41, 16) |
		shift_lower_bits(hashdst_addr, 0, 40));
}

/**
* @brief Generate crypto packet descriptor 1
* @ingroup crypto
* - cipherlen : cipher length in bytes
* - hashlen : hash length in bytes
*/
static inline unsigned long long nlm_crypto_form_pkt_desc1(unsigned int cipherlen, unsigned int hashlen)
{
	return shift_lower_bits_mask((cipherlen - 1), 32, 32)
		| shift_lower_bits_mask((hashlen - 1), 0, 32);
}	

/**
* @brief Generate crypto packet descriptor 2
* @ingroup crypto
* - ivoff : iv offset, offset from start of src data addr
* - ciperbit_cnt : number of valid bits in the last input byte to the cipher, 0(8 bits), 1(1 bit)..7(7 bits)
* - cipheroff : cipher offset, offset from start of src data addr
* - hashbit_cnt : number of valid bits in the last input byte to the auth, 0(8 bits), 1(1 bit)..7(7 bits)
* - hashclobber : 1(hash output will be written as multiples of cachelines, no read modify write)
* - hashoff : hash offset, offset from start of src data addr
*/

static inline unsigned long long nlm_crypto_form_pkt_desc2(unsigned int ivoff, unsigned int cipherbit_cnt, 
		unsigned int cipheroff, 
		unsigned int hashbit_cnt, unsigned int hashclobber, unsigned int hashoff)
{
	return shift_lower_bits(ivoff , 45, 16)
		| shift_lower_bits(cipherbit_cnt, 42, 3) 
		| shift_lower_bits(cipheroff, 22, 16)
		| shift_lower_bits(hashbit_cnt, 19, 3)
		| shift_lower_bits(hashclobber, 18, 1)
		| shift_lower_bits(hashoff, 0, 16);
}

/**
* @brief Generate crypto packet descriptor 3
* @ingroup crypto
* - designer_vc : designer freeback fmn destination id
* - taglen : length in bits of the tag generated by the auth engine
*          md5(128 bits), sha1(160), sha224(224), sha384(384), sha512(512), Kasumi(32), snow3g(32), gcm(128)
* - hmacpad : 1 if hmac padding is already done 
*/
static  inline unsigned long long nlm_crypto_form_pkt_desc3(unsigned int designer_vc, unsigned int taglen, 
	unsigned int arc4_state_save_l3, unsigned int arc4_save_state, unsigned int hmacpad)
{
	return shift_lower_bits(designer_vc, 48, 16)
		| shift_lower_bits(taglen, 11, 16)
		| shift_lower_bits(arc4_state_save_l3, 8, 1)
		| shift_lower_bits(arc4_save_state, 6, 1)
		| shift_lower_bits(hmacpad, 5, 1);
}

/**
* @brief Generate crypto packet descriptor 4
* @ingroup crypto
* - srcfraglen : length of the source fragment(header + data + tail) in bytes
* - srcfragaddr : physical address of the srouce fragment
*/
static inline unsigned long long nlm_crypto_form_pkt_desc4(unsigned int srcfraglen, unsigned long long srcfragaddr )
{
	return shift_lower_bits_mask((srcfraglen - 1), 48, 16)
		| shift_lower_bits(srcfragaddr, 0, 40);
}

/**
* @brief Generate crypto packet descriptor 5
* @ingroup crypto
* - dstfraglen : length of the dst fragment(header + data + tail) in bytes
* - chipherout_l3alloc : 1(cipher output is transited through l3 cache)
* - cipherclobber : 1(cipher output will be written as multiples of cachelines, no read modify write)
* - chiperdst_addr : physical address of the cipher destination address
*/
static inline unsigned long long nlm_crypto_form_pkt_desc5(unsigned int dstfraglen, unsigned int cipherout_l3alloc, 
	       unsigned int cipherclobber, unsigned long long cipherdst_addr)

{
	return shift_lower_bits_mask((dstfraglen - 1), 48, 16)
		| shift_lower_bits(cipherout_l3alloc, 46, 1) 
		| shift_lower_bits(cipherclobber, 41, 1)
		| shift_lower_bits(cipherdst_addr, 0, 40);
}

/**
  * @brief Generate crypto packet fmn message entry 0
  * @ingroup crypto
  * - freeback_vc: freeback response destination address
  * - designer_fblen : Designer freeback length, 1 - 4
  * - designerdesc_valid : designer desc valid or not
  * - cipher_keylen : cipher key length in bytes
  * - ctrldesc_addr : physicall address of the control descriptor
  */
static inline unsigned long long  nlm_crypto_form_pkt_fmn_entry0(unsigned int freeback_vc, unsigned int designer_fblen,
		unsigned int designerdesc_valid, unsigned int cipher_keylen, unsigned long long cntldesc_addr)
{
	return shift_lower_bits(freeback_vc, 48, 16)
		| shift_lower_bits_mask(designer_fblen - 1, 46, 2)
		| shift_lower_bits(designerdesc_valid, 45, 1)
		| shift_lower_bits_mask(((cipher_keylen + 7) >> 3), 40, 5)
		| shift_lower_bits(cntldesc_addr >> 6, 0, 34);
}

/**
  * @brief Generate crypto packet fmn message entry 1
  * @ingroup crypto
  * - arc4load_state : 1 if load state required 0 otherwise
  * - hash_keylen : hash key length in bytes
  * - pktdesc_size : packet descriptor size in bytes
  * - pktdesc_addr : physicall address of the packet descriptor
  */
static inline unsigned long long nlm_crypto_form_pkt_fmn_entry1(unsigned int arc4load_state, unsigned int hash_keylen,
		unsigned int pktdesc_size, unsigned long long pktdesc_addr)
{
	return shift_lower_bits(arc4load_state, 63, 1)
		| shift_lower_bits_mask(((hash_keylen + 7) >> 3), 56, 5)
		| shift_lower_bits_mask(((pktdesc_size >> 4) - 1), 43, 12)
		| shift_lower_bits(pktdesc_addr >> 6, 0, 34);
}

/* Comment this out if using SDK 2.2.7 libraries/crypto and linux-userspace */
#if 0
enum chip_specific_features { 
	INIT_DONE = 0x1,
	ZUC = 0x2, 
	DES3_KEY_SWAP = 0x4
};
#endif

#endif /* _NLM_HAL_CRYPTO_H_*/

