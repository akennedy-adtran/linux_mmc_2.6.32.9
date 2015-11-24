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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/des.h>

#include <linux/hardirq.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/netlogic/msgring.h>
#include "nlm_state_info.h"
#include "nlmsae.h"
#include "nlmsec_internal.h" 


#define AES_CTR_IV_SIZE		8
#define XLR_CRYPT_PRIORITY	300

//#define SEC_DEBUG

#ifdef SEC_DEBUG
#ifdef __KERNEL__
#define debug_print(fmt, args...) printk(fmt, ##args)
#else  /* __KERNEL__ */
#define debug_print(fmt, args...) printf(fmt, ##args)
#endif /* __KERNEL__ */
#else /* SEC_DEBUG */
#define debug_print(fmt, args...)
#endif /* SEC_DEBUG */


/* CRYPTO-API Functions */

static int xlr_aes_setkey(struct crypto_tfm *tfm, const u8 *in_key,
		       unsigned int len)
{
	struct xlr_crypt_state *op = crypto_tfm_ctx(tfm);
	u32 *flags = &tfm->crt_flags;

	op->key_len = len;

	debug_print("in %s keylen is %d\n",__FUNCTION__, len);

	switch (len) {
	case 16: op->cipher = NLM_AES128; break;
	case 24: op->cipher = NLM_AES192; break;
	case 32: op->cipher = NLM_AES256; break;
	default: printk(KERN_WARNING"[%s]: Cannot handle keylen = %d\n",
			__FUNCTION__,len);
		*flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		 return -EINVAL;
	}

	op->block_size = AES_BLOCK_SIZE;
	op->iv_len = AES_BLOCK_SIZE;
	memcpy(op->key, in_key, len);
	return 0;
}

static int xlr_aes_ctr_setkey(struct crypto_tfm *tfm, const u8 *in_key,
		       unsigned int len)
{
	struct xlr_crypt_state *op = crypto_tfm_ctx(tfm);
	u32 *flags = &tfm->crt_flags;

	debug_print("in %s keylen is %d\n",__FUNCTION__, len);
	op->key_len = len - 4;

	switch (len) {
	case 20: op->cipher = NLM_AES128; break;
	case 28: op->cipher = NLM_AES192; break;
	case 36: op->cipher = NLM_AES256; break;

	default: printk(KERN_WARNING"[%s]: Cannot handle keylen = %d\n",
			__FUNCTION__,len);
		*flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		 return -EINVAL;
	}

	op->block_size = 1; // stream cipher
	op->iv_len = AES_CTR_IV_SIZE;
	memcpy(op->key, in_key, len);
	memcpy(&op->nonce, in_key + len - 4, 4);
	return 0;
}

static int xlr_des_setkey(struct crypto_tfm *tfm, const u8 *in_key,
		       unsigned int len)
{
	struct xlr_crypt_state *op = crypto_tfm_ctx(tfm);
	u32 *flags = &tfm->crt_flags;

	op->key_len = len;
	op->cipher = NLM_DES;

	if (len == DES_KEY_SIZE) {
		memcpy(op->key, in_key, len);
		op->block_size = DES_BLOCK_SIZE;
	} else {
		*flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		 return -EINVAL;
	}
	op->iv_len = DES_BLOCK_SIZE;
	return 0;
}

static int xlr_des3_setkey(struct crypto_tfm *tfm, const u8 *in_key,
		       unsigned int len)
{
	struct xlr_crypt_state *op = crypto_tfm_ctx(tfm);
	u32 *flags = &tfm->crt_flags;

	op->key_len = len;
	op->cipher = NLM_DES3;

	if (len == DES3_EDE_KEY_SIZE) {
		memcpy(op->key, in_key, len);
		op->block_size = DES3_EDE_BLOCK_SIZE;
	} else {
		*flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		 return -EINVAL;
	}
	op->iv_len = DES3_EDE_BLOCK_SIZE;
	return 0;
}

int xlr_crypt_op_init(struct xlr_crypt_state *op, op_handle_t *handle, Crypto_Operation_pt *cop_ptr)
{
	operation_pt op_ptr;
	Crypto_Operation_pt cop;
	int ret;

	ret = nlmsec_op_init(handle);

	if(ret) {
		printk("Cannot malloc. returning from %s\n", __FUNCTION__);
		return ret;
	}

	op_ptr = (operation_pt)(*handle);
	cop = &op_ptr->cop_instance;
	*cop_ptr = cop;

	cop->c = op->cipher;
	cop->h = HASH_BYPASS;
	cop->hmac = NULL;
	cop->hmac_len = 0;
//	cop->cipher_mask = ; //probably nonce...
	cop->key = op->key; // use the thing setup in setkey...
	cop->key_len = op->key_len;
//	cop->hash_bytes_to_skip = 0;
//	cop->nonce = NULL;
//	cop->cksum_output = NULL;

	return 0;
}

static int
xlr_ecb_decrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct xlr_crypt_state *op = crypto_blkcipher_ctx(desc->tfm);
	op_handle_t handle = NLMSAE_HANDLE_INITIALIZER;
	Crypto_Operation_pt cop=NULL;
	struct blkcipher_walk walk;
	int err, ret;

	if (( ret = xlr_crypt_op_init(op, &handle, &cop)))
		return ret; 

	cop->m = NLM_ECB;
	cop->encrypt = CIPHER_DECRYPT;

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

	while((nbytes = walk.nbytes)) {
		cop->iv_len = 0;
		cop->input = walk.src.virt.addr;
		cop->input_len = nbytes - (nbytes % op->block_size);
		cop->output = walk.dst.virt.addr;

		ret = nlmsec_cipher_digest_hmac_cksum(handle, cop);
		if(IS_SUCCESS_SAEOP(ret)) 
			xlr_inc_enc_stat(cop->c, cop->m, cop->input_len);
		if (ret == -1)
			return ret;
		nbytes -= ret;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}

	nlmsec_op_cleanup(&handle);
	return err;
}

static int
xlr_ecb_encrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct xlr_crypt_state *op = crypto_blkcipher_ctx(desc->tfm);
	op_handle_t handle = NLMSAE_HANDLE_INITIALIZER;
	Crypto_Operation_pt cop=NULL;
	struct blkcipher_walk walk;
	int err, ret;

	if (( ret = xlr_crypt_op_init(op, &handle, &cop)))
		return ret; 

	cop->m = NLM_ECB;
	cop->encrypt = CIPHER_ENCRYPT;

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

	while((nbytes = walk.nbytes)) {
		cop->iv_len = 0;
		cop->input = walk.src.virt.addr;
		cop->input_len = nbytes - (nbytes % op->block_size);
		cop->output = walk.dst.virt.addr;

		ret = nlmsec_cipher_digest_hmac_cksum(handle, cop);
		if(IS_SUCCESS_SAEOP(ret)) 
			xlr_inc_enc_stat(cop->c, cop->m, cop->input_len);
		if (ret == -1)
			return ret;
		nbytes -= ret;
		ret =  blkcipher_walk_done(desc, &walk, nbytes);
	}
	nlmsec_op_cleanup(&handle);

	return err;
}

static void
xlr_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct xlr_crypt_state *op = crypto_tfm_ctx(tfm);
	op_handle_t handle = NLMSAE_HANDLE_INITIALIZER;
	Crypto_Operation_pt cop=NULL;
	int ret;

	if ((out == NULL) || (in == NULL))
		return;

        debug_print(KERN_INFO "in func %s\n",__FUNCTION__);

	if (xlr_crypt_op_init(op, &handle, &cop))
		return; 

	cop->m = NLM_ECB;
	cop->encrypt = CIPHER_ENCRYPT;
	cop->input = in;
	cop->input_len = op->block_size;
	cop->output = out;

	ret = nlmsec_cipher_digest_hmac_cksum(handle, cop);
	if(IS_SUCCESS_SAEOP(ret))
		xlr_inc_enc_stat(cop->c, cop->m, cop->input_len);
	nlmsec_op_cleanup(&handle);
	return;
}

static void
xlr_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct xlr_crypt_state *op = crypto_tfm_ctx(tfm);
	op_handle_t handle = NLMSAE_HANDLE_INITIALIZER;
	Crypto_Operation_pt cop=NULL;
	int ret;

	if ((out == NULL) || (in == NULL))
		return;

        debug_print(KERN_INFO "in func %s\n",__FUNCTION__);

	if (xlr_crypt_op_init(op, &handle, &cop))
		return; 

	cop->m = NLM_ECB;
	cop->encrypt = CIPHER_DECRYPT;
	cop->input = in;
	cop->input_len = op->block_size;
	cop->output = out;

	ret = nlmsec_cipher_digest_hmac_cksum(handle, cop);
	if(IS_SUCCESS_SAEOP(ret)) 
		xlr_inc_enc_stat(cop->c, cop->m, cop->input_len);
	nlmsec_op_cleanup(&handle);
	return;
}

static int
xlr_cbc_decrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct xlr_crypt_state *op = crypto_blkcipher_ctx(desc->tfm);
	op_handle_t handle = NLMSAE_HANDLE_INITIALIZER;
	Crypto_Operation_pt cop=NULL;
	struct blkcipher_walk walk;
	int err, ret;

        debug_print(KERN_INFO "in func %s:%d\n",__FUNCTION__,__LINE__);

	if (( ret = xlr_crypt_op_init(op, &handle, &cop)))
		return ret; 

	cop->m = NLM_CBC;
	cop->encrypt = CIPHER_DECRYPT;

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);
	while((nbytes = walk.nbytes)) {

		cop->iv = op->iv;
		cop->iv_len = op->iv_len;
		cop->input = walk.src.virt.addr;
		cop->input_len = nbytes - (nbytes % op->block_size);
		cop->output = walk.dst.virt.addr;

		memcpy(op->iv, walk.iv, op->iv_len);

        debug_print(KERN_INFO "in func %s:%d\n",__FUNCTION__,__LINE__);
		ret = nlmsec_cipher_digest_hmac_cksum(handle, cop);
		if(IS_SUCCESS_SAEOP(ret)) 
			xlr_inc_enc_stat(cop->c, cop->m, cop->input_len);

	debug_print(KERN_INFO "input_len = %d, retval = %d \n",cop->input_len, 
		    ret);
		if (ret == -1)
			return ret;
//		memcpy(walk.iv, op->iv, AES_IV_LENGTH);
		memcpy(walk.iv, cop->input + cop->input_len - op->iv_len,
		       op->iv_len);
//		nbytes -= ret;
		nbytes -= cop->input_len;

		err = blkcipher_walk_done(desc, &walk, nbytes);
		if (err)
			return err;
	}

	nlmsec_op_cleanup(&handle);
	return err;
}

static int
xlr_cbc_encrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct xlr_crypt_state *op = crypto_blkcipher_ctx(desc->tfm);
	op_handle_t handle = NLMSAE_HANDLE_INITIALIZER;
	Crypto_Operation_pt cop=NULL;
	struct blkcipher_walk walk;
	int err, ret;

	if (( ret = xlr_crypt_op_init(op, &handle, &cop)))
		return ret; 

	cop->m = NLM_CBC;
	cop->encrypt = CIPHER_ENCRYPT;

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

        debug_print(KERN_INFO "in func %s:%d\n",__FUNCTION__,__LINE__);
	while((nbytes = walk.nbytes)) {
		cop->iv = op->iv;
		cop->iv_len = op->iv_len;
		cop->input = walk.src.virt.addr;
		cop->input_len = nbytes - (nbytes % op->block_size);
		cop->output = walk.dst.virt.addr;

		memcpy(op->iv, walk.iv, op->iv_len);

		ret = nlmsec_cipher_digest_hmac_cksum(handle, cop);
		if(IS_SUCCESS_SAEOP(ret)) 
			xlr_inc_enc_stat(cop->c, cop->m, cop->input_len);

		if (ret == -1)
			return ret;
		
		memcpy(walk.iv, cop->output + cop->input_len - op->iv_len,
		       op->iv_len);

		nbytes -= cop->input_len;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}
	debug_print(KERN_INFO "in func %s:%d\n",__FUNCTION__,__LINE__);

	nlmsec_op_cleanup(&handle);
	return err;
}


static int
xlr_ctr_decrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct xlr_crypt_state *op = crypto_blkcipher_ctx(desc->tfm);
	op_handle_t handle = NLMSAE_HANDLE_INITIALIZER;
	Crypto_Operation_pt cop=NULL;
	struct blkcipher_walk walk;
	int err, ret;

        debug_print(KERN_INFO "in func %s:%d\n",__FUNCTION__,__LINE__);

	if (( ret = xlr_crypt_op_init(op, &handle, &cop)))
		return ret; 

	cop->m = NLM_CTR;
	cop->encrypt = CIPHER_DECRYPT;

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);
	while((nbytes = walk.nbytes)) {

		cop->iv = op->iv;
		cop->iv_len = op->iv_len;
		cop->nonce = op->nonce;
		cop->input = walk.src.virt.addr;
		cop->input_len = nbytes - (nbytes % op->block_size);
		cop->output = walk.dst.virt.addr;

		memcpy(op->iv, walk.iv, op->iv_len);

		debug_print(KERN_INFO "in func %s:%d\n",__FUNCTION__,__LINE__);
		ret = nlmsec_cipher_digest_hmac_cksum(handle, cop);
		if(IS_SUCCESS_SAEOP(ret)) 
			xlr_inc_enc_stat(cop->c, cop->m, cop->input_len);

		debug_print(KERN_INFO "input_len = %d, retval = %d \n",
			    cop->input_len, ret);
		if (ret == -1)
			return ret;
		memcpy(walk.iv, cop->input + cop->input_len - op->iv_len,
		       op->iv_len);
		nbytes -= cop->input_len;

		err = blkcipher_walk_done(desc, &walk, nbytes);
		if (err)
			return err;
	}

	nlmsec_op_cleanup(&handle);
	return err;
}

static int
xlr_ctr_encrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct xlr_crypt_state *op = crypto_blkcipher_ctx(desc->tfm);
	op_handle_t handle = NLMSAE_HANDLE_INITIALIZER;
	Crypto_Operation_pt cop=NULL;
	struct blkcipher_walk walk;
	int err, ret;

	if (( ret = xlr_crypt_op_init(op, &handle, &cop)))
		return ret; 


	cop->m = NLM_CTR;
	cop->encrypt = CIPHER_ENCRYPT;

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

        debug_print(KERN_INFO "in func %s:%d\n",__FUNCTION__,__LINE__);
	while((nbytes = walk.nbytes)) {
		cop->iv = op->iv;
		cop->iv_len = op->iv_len;
		cop->nonce = op->nonce;
		cop->input = walk.src.virt.addr;
		cop->input_len = nbytes - (nbytes % op->block_size);
		cop->output = walk.dst.virt.addr;

		memcpy(op->iv, walk.iv, op->iv_len);

		ret = nlmsec_cipher_digest_hmac_cksum(handle, cop);
		if(IS_SUCCESS_SAEOP(ret)) 
			xlr_inc_enc_stat(cop->c, cop->m, cop->input_len);
		debug_print(KERN_INFO "input_len = %d, retval = %d \n",
			    cop->input_len, ret);

		if (ret == -1)
			return ret;
		
		memcpy(walk.iv, cop->output + cop->input_len - op->iv_len,
		       op->iv_len);

		nbytes -= cop->input_len;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}
	debug_print(KERN_INFO "in func %s:%d\n",__FUNCTION__,__LINE__);

	nlmsec_op_cleanup(&handle);
	return err;
}

static struct crypto_alg xlr_ecb_aes_alg = {
	.cra_name		=	"ecb(aes)",
	.cra_driver_name	=	"ecb-aes-xlr",
	.cra_priority		=	XLR_CRYPT_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_alignmask		=	15,
	.cra_type		=	&crypto_blkcipher_type,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_ecb_aes_alg.cra_list),
	.cra_u			=	{
		.blkcipher = {
			.min_keysize		=	AES_MIN_KEY_SIZE,
			.max_keysize		=	AES_MAX_KEY_SIZE,
			.setkey			=	xlr_aes_setkey,
			.encrypt		=	xlr_ecb_encrypt,
			.decrypt		=	xlr_ecb_decrypt,
		}
	}
};

static struct crypto_alg xlr_aes_alg = {
	.cra_name               =       "aes",
	.cra_driver_name	=       "aes-xlr",
	.cra_priority           =       XLR_CRYPT_PRIORITY,
	.cra_alignmask          =       15,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_aes_alg.cra_list),
	.cra_u			=	{
		.cipher = {
			.cia_min_keysize	=  AES_MIN_KEY_SIZE,
			.cia_max_keysize	=  AES_MAX_KEY_SIZE,
			.cia_setkey		=  xlr_aes_setkey,
			.cia_encrypt		=  xlr_encrypt,
			.cia_decrypt		=  xlr_decrypt
		}
	}
};

static struct crypto_alg xlr_cbc_aes_alg = {
	.cra_name		=	"cbc(aes)",
	.cra_driver_name	=	"cbc-aes-xlr",
	.cra_priority		=	XLR_CRYPT_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_alignmask		=	15,
	.cra_type		=	&crypto_blkcipher_type,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_cbc_aes_alg.cra_list),
	.cra_u			=	{
		.blkcipher = {
			.min_keysize		=	AES_MIN_KEY_SIZE,
			.max_keysize		=	AES_MAX_KEY_SIZE,
			.setkey			=	xlr_aes_setkey,
			.encrypt		=	xlr_cbc_encrypt,
			.decrypt		=	xlr_cbc_decrypt,
			.ivsize			=	AES_BLOCK_SIZE,
		}
	}
};

static struct crypto_alg xlr_ctr_aes_alg = {
	.cra_name		=	"ctr(aes)",
	.cra_driver_name	=	"ctr-aes-xlr",
	.cra_priority		=	XLR_CRYPT_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_blocksize		=	1,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_alignmask		=	15,
	.cra_type		=	&crypto_blkcipher_type,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_ctr_aes_alg.cra_list),
	.cra_u			=	{
		.blkcipher = {
			.min_keysize		=	AES_MIN_KEY_SIZE,
			.max_keysize		=	AES_MAX_KEY_SIZE,
			.setkey			=	xlr_aes_ctr_setkey,
			.encrypt		=	xlr_ctr_encrypt,
			.decrypt		=	xlr_ctr_decrypt,
			.ivsize			=	AES_CTR_IV_SIZE,
		}
	}
};

static struct crypto_alg xlr_ecb_des_alg = {
	.cra_name		=	"ecb(des)",
	.cra_driver_name	=	"ecb-des-xlr",
	.cra_priority		=	XLR_CRYPT_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_blocksize		=	DES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_alignmask		=	15,
	.cra_type		=	&crypto_blkcipher_type,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_ecb_des_alg.cra_list),
	.cra_u			=	{
		.blkcipher = {
			.min_keysize		=	DES_KEY_SIZE,
			.max_keysize		=	DES_KEY_SIZE,
			.setkey			=	xlr_des_setkey,
			.encrypt		=	xlr_ecb_encrypt,
			.decrypt		=	xlr_ecb_decrypt,
		}
	}
};

static struct crypto_alg xlr_des_alg = {
	.cra_name               =       "des",
	.cra_driver_name	=       "des-xlr",
	.cra_priority           =       XLR_CRYPT_PRIORITY,
	.cra_alignmask          =       15,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	DES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_des_alg.cra_list),
	.cra_u			=	{
		.cipher = {
			.cia_min_keysize	=  DES_KEY_SIZE,
			.cia_max_keysize	=  DES_KEY_SIZE,
			.cia_setkey		=  xlr_des_setkey,
			.cia_encrypt		=  xlr_encrypt,
			.cia_decrypt		=  xlr_decrypt
		}
	}
};

static struct crypto_alg xlr_cbc_des_alg = {
	.cra_name		=	"cbc(des)",
	.cra_driver_name	=	"cbc-des-xlr",
	.cra_priority		=	XLR_CRYPT_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_blocksize		=	DES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_alignmask		=	15,
	.cra_type		=	&crypto_blkcipher_type,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_cbc_des_alg.cra_list),
	.cra_u			=	{
		.blkcipher = {
			.min_keysize		=	DES_KEY_SIZE,
			.max_keysize		=	DES_KEY_SIZE,
			.setkey			=	xlr_des_setkey,
			.encrypt		=	xlr_cbc_encrypt,
			.decrypt		=	xlr_cbc_decrypt,
			.ivsize			=	DES_BLOCK_SIZE,
		}
	}
};

static struct crypto_alg xlr_ecb_des3_alg = {
	.cra_name		=	"ecb(des3_ede)",
	.cra_driver_name	=	"ecb-des3-xlr",
	.cra_priority		=	XLR_CRYPT_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_blocksize		=	DES3_EDE_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_alignmask		=	15,
	.cra_type		=	&crypto_blkcipher_type,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_ecb_des3_alg.cra_list),
	.cra_u			=	{
		.blkcipher = {
			.min_keysize		=	DES3_EDE_KEY_SIZE,
			.max_keysize		=	DES3_EDE_KEY_SIZE,
			.setkey			=	xlr_des3_setkey,
			.encrypt		=	xlr_ecb_encrypt,
			.decrypt		=	xlr_ecb_decrypt,
		}
	}
};

static struct crypto_alg xlr_des3_alg = {
	.cra_name               =       "des3_ede",
	.cra_driver_name	=       "des3_ede-xlr",
	.cra_priority           =       XLR_CRYPT_PRIORITY,
	.cra_alignmask          =       15,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	DES3_EDE_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_des3_alg.cra_list),
	.cra_u			=	{
		.cipher = {
			.cia_min_keysize	=  DES3_EDE_KEY_SIZE,
			.cia_max_keysize	=  DES3_EDE_KEY_SIZE,
			.cia_setkey		=  xlr_des3_setkey,
			.cia_encrypt		=  xlr_encrypt,
			.cia_decrypt		=  xlr_decrypt
		}
	}
};

static struct crypto_alg xlr_cbc_des3_alg = {
	.cra_name		=	"cbc(des3_ede)",
	.cra_driver_name	=	"cbc-des3_ede-xlr",
	.cra_priority		=	XLR_CRYPT_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_blocksize		=	DES3_EDE_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_alignmask		=	15,
	.cra_type		=	&crypto_blkcipher_type,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(xlr_cbc_aes_alg.cra_list),
	.cra_u			=	{
		.blkcipher = {
			.min_keysize		=	DES3_EDE_KEY_SIZE,
			.max_keysize		=	DES3_EDE_KEY_SIZE,
			.setkey			=	xlr_des3_setkey,
			.encrypt		=	xlr_cbc_encrypt,
			.decrypt		=	xlr_cbc_decrypt,
			.ivsize			=	DES3_EDE_BLOCK_SIZE,
		}
	}
};


static int __init xlr_crypt_alg_init(void)
{
	int ret;

	if ((ret = crypto_register_alg(&xlr_aes_alg)))
		goto err_out;

	if ((ret = crypto_register_alg(&xlr_ecb_aes_alg)))
		goto err1;

	if ((ret = crypto_register_alg(&xlr_cbc_aes_alg)))
		goto err2;

	if ((ret = crypto_register_alg(&xlr_des_alg)))
		goto err3;

	if ((ret = crypto_register_alg(&xlr_ecb_des_alg)))
		goto err4;

	if ((ret = crypto_register_alg(&xlr_cbc_des_alg)))
		goto err5;

	if ((ret = crypto_register_alg(&xlr_des3_alg)))
		goto err6;

	if ((ret = crypto_register_alg(&xlr_ecb_des3_alg)))
		goto err7;

	if ((ret = crypto_register_alg(&xlr_cbc_des3_alg)))
		goto err8;
	
//	if ((ret = crypto_register_alg(&xlr_ctr_aes_alg)))
//		goto err9;

	printk(KERN_NOTICE "Using XLR hardware for AES/DES/3DES algorithm.\n");
	return 0;

//err9:
//	crypto_unregister_alg(&xlr_cbc_des3_alg);
err8:
	crypto_unregister_alg(&xlr_ecb_des3_alg);
err7:
	crypto_unregister_alg(&xlr_des3_alg);
err6:
	crypto_unregister_alg(&xlr_cbc_des_alg);
err5:
	crypto_unregister_alg(&xlr_ecb_des_alg);
err4:
	crypto_unregister_alg(&xlr_des_alg);
err3:
	crypto_unregister_alg(&xlr_cbc_aes_alg);
err2:
	crypto_unregister_alg(&xlr_ecb_aes_alg);
err1:
	crypto_unregister_alg(&xlr_aes_alg);
err_out:
	printk(KERN_ERR "XLR hardware AES/DES/3DES initialization failed.\n");
	return ret;
}

static void __exit xlr_crypt_alg_fini(void)
{
	crypto_unregister_alg(&xlr_cbc_aes_alg);
	crypto_unregister_alg(&xlr_ecb_aes_alg);
	crypto_unregister_alg(&xlr_aes_alg);
	crypto_unregister_alg(&xlr_cbc_des_alg);
	crypto_unregister_alg(&xlr_ecb_des_alg);
	crypto_unregister_alg(&xlr_des_alg);
	crypto_unregister_alg(&xlr_cbc_des3_alg);
	crypto_unregister_alg(&xlr_ecb_des3_alg);
	crypto_unregister_alg(&xlr_des3_alg);
//	crypto_unregister_alg(&xlr_ctr_aes_alg);
}

//module_init(xlr_crypt_alg_init);
//module_exit(xlr_crypt_alg_fini);

MODULE_DESCRIPTION("XLR Hardware AES/DES/3DES algorithms support.");
MODULE_LICENSE("GPL/BSD");
MODULE_AUTHOR("Sandip Matte");

