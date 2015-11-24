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

/*
 * Cryptographic API.
 *
 * Support for XLR hardware crypto engine.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <crypto/algapi.h>
#include <crypto/sha.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/cryptohash.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/scatterlist.h>
#include "nlm_state_info.h"
#include "nlmsae.h"
#include "nlmsec_internal.h" 

#define XLR_AUTH_PRIORITY      300
#define XLR_HMAC_PRIORITY      320

#define MD5_DIGEST_SIZE		16
#define MD5_BLOCK_SIZE		64

#define AUTH_BUFFER_SIZE	(16 * 1024)

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

static inline struct xlr_crypt_state* ctx(struct crypto_tfm *tfm)
{
	return crypto_tfm_ctx(tfm);
}

static void xlr_auth_init(struct crypto_tfm *tfm)
{
	struct xlr_crypt_state *op = ctx(tfm);
	op->auth_src = kmalloc(AUTH_BUFFER_SIZE, GFP_KERNEL);
        debug_print("in func %s\n",__FUNCTION__);
	op->len = 0;
}

static void xlr_auth_update(struct crypto_tfm *tfm,
			const uint8_t *data, unsigned int length)
{
	struct xlr_crypt_state *op = ctx(tfm);

	if (!op->auth_src)
		return;
	if (op->len + length > AUTH_BUFFER_SIZE)
		return;

	memcpy(op->auth_src + op->len, data, length);
	op->len += length;
        debug_print(KERN_INFO "in func %s\n",__FUNCTION__);
//	dump_stack();

}

static void xlr_auth_final(struct crypto_tfm *tfm, uint8_t *out)
{
	struct xlr_crypt_state *op = ctx(tfm);
	op_handle_t handle = NLMSAE_HANDLE_INITIALIZER;
	int ret;

	ret = nlmsec_op_init(&handle);
        debug_print("in func %s\n",__FUNCTION__);

	if (ret) {
		printk("Cannot malloc. returning from %s\n", __FUNCTION__);
		return; 
	}

	debug_print(KERN_INFO "hash = %d, hmac keylen = %d\n",op->hash, op->hmac_keylen);

	ret = nlmsec_cipher_and_hash(handle, CIPHER_BYPASS, NLM_ECB, NULL, NULL,
				     0, 0, op->hash, 
				     (op->hmac_keylen?op->hmac_key:NULL), 0, 0,
				     op->auth_src, op->len, NULL, out);

	if(IS_SUCCESS_SAEOP(ret)) {
		xlr_inc_auth_stat(op->hash, op->len);
	}
        debug_print(KERN_INFO "ret val after hashing is %d\n",ret);
	op->len = 0;
	nlmsec_op_cleanup(&handle);
	kfree(op->auth_src);
}

static void xlr_sha1_final(struct crypto_tfm *tfm, uint8_t *out)
{
	struct xlr_crypt_state *op = ctx(tfm);

	op->hash = NLM_SHA1;
	xlr_auth_final(tfm, out);
}

static void xlr_sha256_final(struct crypto_tfm *tfm, uint8_t *out)
{
	struct xlr_crypt_state *op = ctx(tfm);

	op->hash = NLM_SHA256;
	xlr_auth_final(tfm, out);
}

static void xlr_sha384_final(struct crypto_tfm *tfm, uint8_t *out)
{
	struct xlr_crypt_state *op = ctx(tfm);

	op->hash = NLM_SHA384;
	xlr_auth_final(tfm, out);
}

static void xlr_sha512_final(struct crypto_tfm *tfm, uint8_t *out)
{
	struct xlr_crypt_state *op = ctx(tfm);

	op->hash = NLM_SHA512;
	xlr_auth_final(tfm, out);
}

static void xlr_md5_final(struct crypto_tfm *tfm, uint8_t *out)
{
	struct xlr_crypt_state *op = ctx(tfm);

	op->hash = NLM_MD5;
	xlr_auth_final(tfm, out);
}

static int xlr_auth_setkey(struct crypto_tfm *tfm, const u8 *key, unsigned int keylen)
{
	struct xlr_crypt_state *op = ctx(tfm);
	op->hmac = 1;
	op->hmac_keylen = keylen;
	memcpy(op->hmac_key, key, keylen);
        debug_print(KERN_INFO "[%s] keylen is %d\n",__FUNCTION__ ,keylen);
	return 0;
}

static struct crypto_alg sha512_alg = {
	.cra_name		=	"sha512",
	.cra_driver_name	=	"sha512-xlr",
	.cra_priority		=	XLR_AUTH_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST|
					CRYPTO_ALG_HW,
	.cra_blocksize		=	SHA512_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(sha512_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	SHA512_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_sha512_final,
		}
	}
};


static struct crypto_alg sha384_alg = {
	.cra_name		=	"sha384",
	.cra_driver_name	=	"sha384-xlr",
	.cra_priority		=	XLR_AUTH_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST|
					CRYPTO_ALG_HW,
	.cra_blocksize		=	SHA384_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(sha384_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	SHA384_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_sha384_final,
		}
	}
};

static struct crypto_alg sha256_alg = {
	.cra_name		=	"sha256",
	.cra_driver_name	=	"sha256-xlr",
	.cra_priority		=	XLR_AUTH_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST |
					CRYPTO_ALG_HW,
	.cra_blocksize		=	SHA256_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(sha256_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	SHA256_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_sha256_final,
		}
	}
};

static struct crypto_alg sha1_alg = {
	.cra_name		=	"sha1",
	.cra_driver_name	=	"sha1-xlr",
	.cra_priority		=	XLR_AUTH_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST|
					CRYPTO_ALG_HW,
	.cra_blocksize		=	SHA1_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(sha1_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	SHA1_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_sha1_final,
		}
	}
};

static struct crypto_alg md5_alg = {
	.cra_name		=	"md5",
	.cra_driver_name	=	"md5-xlr",
	.cra_priority		=	XLR_AUTH_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST |
					CRYPTO_ALG_HW,
	.cra_blocksize		=	MD5_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(md5_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	MD5_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_md5_final,
		}
	}
};

static struct crypto_alg sha512_hmac_alg = {
	.cra_name		=	"hmac(sha512)",
	.cra_driver_name	=	"hmac-sha512-xlr",
	.cra_priority		=	XLR_HMAC_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST|
					CRYPTO_ALG_HW,
	.cra_blocksize		=	SHA512_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(sha512_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	SHA512_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_sha512_final,
			.dia_setkey     =	xlr_auth_setkey,
		}
	}
};


static struct crypto_alg sha384_hmac_alg = {
	.cra_name		=	"hmac(sha384)",
	.cra_driver_name	=	"hmac-sha384-xlr",
	.cra_priority		=	XLR_HMAC_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST|
					CRYPTO_ALG_HW,
	.cra_blocksize		=	SHA384_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(sha384_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	SHA384_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_sha384_final,
			.dia_setkey     =	xlr_auth_setkey,
		}
	}
};


static struct crypto_alg sha256_hmac_alg = {
	.cra_name		=	"hmac(sha256)",
	.cra_driver_name	=	"hmac-sha256-xlr",
	.cra_priority		=	XLR_HMAC_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST |
					CRYPTO_ALG_HW,
	.cra_blocksize		=	SHA256_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(sha256_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	SHA256_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_sha256_final,
			.dia_setkey     =	xlr_auth_setkey,
		}
	}
};

static struct crypto_alg sha1_hmac_alg = {
	.cra_name		=	"hmac(sha1)",
	.cra_driver_name	=	"hmac-sha1-xlr",
	.cra_priority		=	XLR_HMAC_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST|
					CRYPTO_ALG_HW,
	.cra_blocksize		=	SHA1_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(sha1_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	SHA1_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_sha1_final,
			.dia_setkey     =	xlr_auth_setkey,
		}
	}
};

static struct crypto_alg md5_hmac_alg = {
	.cra_name		=	"hmac(md5)",
	.cra_driver_name	=	"hmac-md5-xlr",
	.cra_priority		=	XLR_HMAC_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_DIGEST |
					CRYPTO_ALG_HW,
	.cra_blocksize		=	MD5_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct xlr_crypt_state),
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(md5_alg.cra_list),
	.cra_u			=	{
		.digest = {
			.dia_digestsize	=	MD5_DIGEST_SIZE,
			.dia_init   	= 	xlr_auth_init,
			.dia_update 	=	xlr_auth_update,
			.dia_final  	=	xlr_md5_final,
			.dia_setkey	=	xlr_auth_setkey,
		}
	}
};

static int __init xlr_auth_alg_init(void)
{
	int rc = -ENODEV;

	rc = crypto_register_alg(&sha1_alg);
	if (rc)
		goto out;

	rc = crypto_register_alg(&sha256_alg);
	if (rc)
		goto out_unreg1;

	rc = crypto_register_alg(&sha384_alg);
	if (rc)
		goto out_unreg2;

	rc = crypto_register_alg(&sha512_alg);
	if (rc)
		goto out_unreg3;

	rc = crypto_register_alg(&md5_alg);
	if (rc)
		goto out_unreg4;

	rc = crypto_register_alg(&sha1_hmac_alg);
	if (rc)
		goto out_unreg5;

	rc = crypto_register_alg(&sha256_hmac_alg);
	if (rc)
		goto out_unreg6;

	rc = crypto_register_alg(&sha384_hmac_alg);
	if (rc)
		goto out_unreg7;

	rc = crypto_register_alg(&sha512_hmac_alg);
	if (rc)
		goto out_unreg8;

	rc = crypto_register_alg(&md5_hmac_alg);
	if (rc)
		goto out_unreg9;

	printk(KERN_NOTICE "Using XLR hardware for SHA/MD5 algorithms.\n");

	return 0;

out_unreg9:
	crypto_unregister_alg(&sha512_hmac_alg);
out_unreg8:
	crypto_unregister_alg(&sha384_hmac_alg);
out_unreg7:
	crypto_unregister_alg(&sha256_hmac_alg);
out_unreg6:
	crypto_unregister_alg(&sha1_hmac_alg);
out_unreg5:
	crypto_unregister_alg(&md5_alg);
out_unreg4:
	crypto_unregister_alg(&sha512_alg);
out_unreg3:
	crypto_unregister_alg(&sha384_alg);
out_unreg2:
	crypto_unregister_alg(&sha256_alg);
out_unreg1:
	crypto_unregister_alg(&sha1_alg);
out:
	printk(KERN_ERR "XLR SHA/MD5 initialization failed.\n");
	return rc;

}

static void __exit xlr_auth_alg_fini(void)
{
	crypto_unregister_alg(&sha1_alg);
	crypto_unregister_alg(&sha256_alg);
	crypto_unregister_alg(&sha384_alg);
	crypto_unregister_alg(&sha512_alg);
	crypto_unregister_alg(&md5_alg);
	crypto_unregister_alg(&md5_hmac_alg);
	crypto_unregister_alg(&sha1_hmac_alg);
	crypto_unregister_alg(&sha256_hmac_alg);
	crypto_unregister_alg(&sha384_hmac_alg);
	crypto_unregister_alg(&sha512_hmac_alg);
}

//module_init(xlr_auth_alg_init);
//module_exit(xlr_auth_alg_fini);

MODULE_DESCRIPTION("XLR SHA/MD5 algorithms support.");
MODULE_LICENSE("GPL/BSD");
MODULE_AUTHOR("Sandip Matte");

MODULE_ALIAS("sha-xlr");
MODULE_ALIAS("md5-xlr");
