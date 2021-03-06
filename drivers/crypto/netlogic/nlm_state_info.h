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

struct xlr_crypt_state {

	void *src;
	void *dst;
	void *auth_dst;
	void *auth_src;
	
	u32 cipher;
	u32 hash;

	u32 mode;
	u32 dir;
	u32 flags;
	int len;
	int iv_len;
	int block_size;
	u8 *iv_ptr;
	u8 *data_ptr;
	u8 *out_data_ptr;
	u8 *auth_ptr;
	int hmac;
	int hmac_keylen;
	void *malloc_ptr;
	int nr_data_desc;
	//sandip -> check data_desc ptr...
	uint64_t *cipher_desc;
	uint64_t *cipher_key_hash_info;
	uint64_t *data_desc;
	int key_len;

	u8 key[32]; // max key length
	u8 iv[32]; // max iv length
	u8 hmac_key[128]; // max hmac key length //sandip -> Check this length
	u8 hash_result[128];
	uint32_t nonce;
//	u8 nonce[16];
//	int nonce_len;
	uint64_t data_msg;
	uint64_t ctrl_msg;
};

#ifdef SANDIP

/**
 * SIZES of control data in bytes
 */
#define CTRL_DESC_VECTOR_SIZE 8
#define CTRL_INSTR_SIZE 8
#define MAX_CTRL_DATA_SIZE 472
#define DATA_DESC_VECTOR_SIZE 8
#define DATA_DESC_SIZE 32
#define CEIL_BY_DIV(a,b) (((a)/b) + ((a%b)?1:0))
#define ADDR_MASK 0xffffffffe0ULL
#define DATA_DESC_VECTOR_TEMPLATE         ((5ULL << 61) | (1ULL << 45))
#define MAX_FRAG_TOT_SIZE (((1 << 11) -1) * 8) /* 16 K - 8 */
#define MAX_PER_FRAG_FIELD_SIZE (MAX_FRAG_TOT_SIZE >> 5 << 2)
#define MAX_PER_FRAG_SIZE (MAX_FRAG_TOT_SIZE >> 5 << 5);

#define PTR_OFFSET(a,b) (unsigned long)((((unsigned long)a) + (b)))
#define HAS_REMINDER(x, y)  (((uint64_t)(x) & ((1ULL << (y)) - 1))?1:0)
#define CEIL(n, bits) ((((uint64_t)n)>>(bits)) + HAS_REMINDER(n,bits))
#define CEIL_BYTES(m,n) ((CEIL(m,n))<<(n))
#define NEXT_CACHELINE_ALIGN(m) (unsigned long)(CEIL_BYTES(m, 5))
#define CACHELINE_ALIGN(m) ((unsigned long long)(unsigned long)(m) >> 5 << 5)
#define PTR_DIFF(x, y)   (((unsigned long)x) - ((unsigned long)(y)))

#define APPLY_ADDR_MASK(phys,mask) ((unsigned long long)(phys) & (mask))

typedef enum CipherAlgo {
	CIPHER_BYPASS = 0,
	NLM_DES,
	NLM_DES3,
	NLM_AES128,
	NLM_AES192,
	NLM_AES256,
	NLM_ARC4,
	NLM_KASUMI_F8,
	MAX_CIPHER_ALGO
} CipherAlgo_t;

typedef enum CipherMode {
	NLM_ECB = 0,
	NLM_CBC,
	NLM_CFB,
	NLM_OFB,
	NLM_CTR,
	NLM_F8,
	NLM_CCM,
	MAX_CIPHER_MODE
} CipherMode_t;

typedef enum HashAlgo {
	HASH_BYPASS = 0,
	NLM_MD5,
	NLM_SHA1,
	NLM_SHA256,
	NLM_SHA384,
	NLM_SHA512,
	NLM_GCM,
	NLM_KASUMI_F9,
	NLM_DES3_CMAC,
	NLM_AES_CMAC,
	MAX_HASH_ALGO
} HashAlgo_t;

typedef enum {
	CIPHER_DECRYPT=0,
	CIPHER_ENCRYPT
} Cipher_Function_t;

#endif /* SANDIP */

//extern unsigned int xlr_digest(struct xlr_sae_op *op);
//extern unsigned int xlr_crypt(struct xlr_sae_op *op);
//extern int xlr_sec_init(void);
