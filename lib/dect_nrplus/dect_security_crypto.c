/* dect_nrplus/dect_security_crypto.c */
/**
 * @file dect_security_crypto.c
 * @brief Unified cryptography implementation for DECT NR+ security
 *
 * Provides hardware-agnostic crypto operations using PSA Crypto API as backend.
 * This abstraction allows future support for alternative crypto implementations.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#if IS_ENABLED(CONFIG_DECT_SECURITY_ENABLE)

#include "dect_security_crypto.h"

/* Use PSA Crypto as the backend */
#include <psa/crypto.h>

LOG_MODULE_REGISTER(dect_crypto, CONFIG_DECT_NRPLUS_LOG_LEVEL);

/* Internal state */
static bool crypto_initialized = false;

int dect_crypto_init(void)
{
	if (crypto_initialized) {
		return 0;
	}

	psa_status_t status = psa_crypto_init();
	if (status != PSA_SUCCESS) {
		LOG_ERR("PSA Crypto init failed: %d", (int)status);
		return -EIO;
	}

	crypto_initialized = true;
	LOG_INF("DECT crypto subsystem initialized (PSA backend)");
	return 0;
}

int dect_crypto_aes_cmac(const uint8_t *key, size_t key_len,
                         const uint8_t *data, size_t data_len,
                         uint8_t *mac_out, size_t mac_len)
{
	if (!key || !data || !mac_out) {
		return -EINVAL;
	}

	if (key_len != 16) {
		LOG_ERR("Invalid key length: %zu (expected 16)", key_len);
		return -EINVAL;
	}

	if (mac_len == 0 || mac_len > 16) {
		LOG_ERR("Invalid MAC length: %zu (must be 1-16)", mac_len);
		return -EINVAL;
	}

	psa_status_t status;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = 0;
	uint8_t full_mac[16];
	size_t mac_length;

	/* Set key attributes for CMAC */
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
	psa_set_key_algorithm(&attributes, PSA_ALG_CMAC);
	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attributes, 128);

	/* Import key */
	status = psa_import_key(&attributes, key, key_len, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CMAC key import failed: %d", (int)status);
		return -EIO;
	}

	/* Compute CMAC */
	status = psa_mac_compute(key_id, PSA_ALG_CMAC,
	                         data, data_len,
	                         full_mac, sizeof(full_mac), &mac_length);

	/* Destroy key immediately after use */
	psa_destroy_key(key_id);

	if (status != PSA_SUCCESS) {
		LOG_ERR("CMAC computation failed: %d", (int)status);
		return -EIO;
	}

	/* Truncate MAC to requested length */
	memcpy(mac_out, full_mac, mac_len);

	LOG_DBG("CMAC computed: %zu bytes data -> %zu bytes MAC", data_len, mac_len);
	return 0;
}

int dect_crypto_aes_ctr_crypt(const uint8_t *key, uint8_t *iv,
                               uint8_t *data_inout, size_t len)
{
	if (!key || !iv || !data_inout) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0; /* Nothing to encrypt */
	}

	psa_status_t status;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = 0;
	psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;
	size_t output_len;

	/* Set key attributes for CTR */
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
	psa_set_key_algorithm(&attributes, PSA_ALG_CTR);
	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attributes, 128);

	/* Import key */
	status = psa_import_key(&attributes, key, 16, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CTR key import failed: %d", (int)status);
		return -EIO;
	}

	/* Setup cipher operation */
	status = psa_cipher_encrypt_setup(&operation, key_id, PSA_ALG_CTR);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CTR setup failed: %d", (int)status);
		psa_destroy_key(key_id);
		return -EIO;
	}

	/* Set IV */
	status = psa_cipher_set_iv(&operation, iv, 16);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CTR set IV failed: %d", (int)status);
		psa_cipher_abort(&operation);
		psa_destroy_key(key_id);
		return -EIO;
	}

	/* Encrypt/decrypt in-place */
	status = psa_cipher_update(&operation, data_inout, len,
	                            data_inout, len, &output_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CTR update failed: %d", (int)status);
		psa_cipher_abort(&operation);
		psa_destroy_key(key_id);
		return -EIO;
	}

	/* Finalize operation */
	size_t final_len;
	status = psa_cipher_finish(&operation, data_inout + output_len,
	                            len - output_len, &final_len);
	
	psa_destroy_key(key_id);

	if (status != PSA_SUCCESS) {
		LOG_ERR("CTR finish failed: %d", (int)status);
		return -EIO;
	}

	LOG_DBG("AES-CTR: %zu bytes processed", len);
	return 0;
}

int dect_crypto_aes_ccm_encrypt_auth(const uint8_t *key,
                                       const uint8_t *nonce, size_t nonce_len,
                                       const uint8_t *aad, size_t aad_len,
                                       const uint8_t *plaintext, size_t plaintext_len,
                                       uint8_t *ciphertext,
                                       uint8_t *tag, size_t tag_len)
{
	if (!key || !nonce || !plaintext || !ciphertext || !tag) {
		return -EINVAL;
	}

	psa_status_t status;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = 0;
	psa_aead_operation_t operation = PSA_AEAD_OPERATION_INIT;
	size_t output_len, tag_length;

	/* Set key attributes for CCM */
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
	psa_set_key_algorithm(&attributes, PSA_ALG_CCM);
	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attributes, 128);

	/* Import key */
	status = psa_import_key(&attributes, key, 16, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CCM key import failed: %d", (int)status);
		return -EIO;
	}

	/* Setup AEAD operation */
	status = psa_aead_encrypt_setup(&operation, key_id, PSA_ALG_CCM);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CCM encrypt setup failed: %d", (int)status);
		psa_destroy_key(key_id);
		return -EIO;
	}

	/* Set nonce */
	status = psa_aead_set_nonce(&operation, nonce, nonce_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CCM set nonce failed: %d", (int)status);
		psa_aead_abort(&operation);
		psa_destroy_key(key_id);
		return -EIO;
	}

	/* Update AAD if present */
	if (aad && aad_len > 0) {
		status = psa_aead_update_ad(&operation, aad, aad_len);
		if (status != PSA_SUCCESS) {
			LOG_ERR("CCM update AAD failed: %d", (int)status);
			psa_aead_abort(&operation);
			psa_destroy_key(key_id);
			return -EIO;
		}
	}

	/* Encrypt plaintext */
	status = psa_aead_update(&operation, plaintext, plaintext_len,
	                         ciphertext, plaintext_len, &output_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CCM update failed: %d", (int)status);
		psa_aead_abort(&operation);
		psa_destroy_key(key_id);
		return -EIO;
	}

	/* Finalize and get tag */
	status = psa_aead_finish(&operation, ciphertext + output_len,
	                         plaintext_len - output_len, &output_len,
	                         tag, tag_len, &tag_length);
	
	psa_destroy_key(key_id);

	if (status != PSA_SUCCESS) {
		LOG_ERR("CCM finish failed: %d", (int)status);
		return -EIO;
	}

	LOG_DBG("CCM encrypt: %zu bytes plaintext -> %zu bytes ciphertext + %zu bytes tag",
	        plaintext_len, output_len, tag_length);
	return 0;
}

int dect_crypto_aes_ccm_decrypt_verify(const uint8_t *key,
                                         const uint8_t *nonce, size_t nonce_len,
                                         const uint8_t *aad, size_t aad_len,
                                         const uint8_t *ciphertext, size_t ciphertext_len,
                                         const uint8_t *tag, size_t tag_len,
                                         uint8_t *plaintext)
{
	if (!key || !nonce || !ciphertext || !tag || !plaintext) {
		return -EINVAL;
	}

	psa_status_t status;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key_id = 0;
	psa_aead_operation_t operation = PSA_AEAD_OPERATION_INIT;
	size_t output_len;

	/* Set key attributes for CCM */
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DECRYPT);
	psa_set_key_algorithm(&attributes, PSA_ALG_CCM);
	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attributes, 128);

	/* Import key */
	status = psa_import_key(&attributes, key, 16, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CCM key import failed: %d", (int)status);
		return -EIO;
	}

	/* Setup AEAD operation */
	status = psa_aead_decrypt_setup(&operation, key_id, PSA_ALG_CCM);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CCM decrypt setup failed: %d", (int)status);
		psa_destroy_key(key_id);
		return -EIO;
	}

	/* Set nonce */
	status = psa_aead_set_nonce(&operation, nonce, nonce_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CCM set nonce failed: %d", (int)status);
		psa_aead_abort(&operation);
		psa_destroy_key(key_id);
		return -EIO;
	}

	/* Update AAD if present */
	if (aad && aad_len > 0) {
		status = psa_aead_update_ad(&operation, aad, aad_len);
		if (status != PSA_SUCCESS) {
			LOG_ERR("CCM update AAD failed: %d", (int)status);
			psa_aead_abort(&operation);
			psa_destroy_key(key_id);
			return -EIO;
		}
	}

	/* Decrypt ciphertext */
	status = psa_aead_update(&operation, ciphertext, ciphertext_len,
	                         plaintext, ciphertext_len, &output_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CCM update failed: %d", (int)status);
		psa_aead_abort(&operation);
		psa_destroy_key(key_id);
		return -EIO;
	}

	/* Verify tag and finalize */
	status = psa_aead_verify(&operation, plaintext + output_len,
	                         ciphertext_len - output_len, &output_len,
	                         tag, tag_len);
	
	psa_destroy_key(key_id);

	if (status != PSA_SUCCESS) {
		if (status == PSA_ERROR_INVALID_SIGNATURE) {
			LOG_WRN("CCM authentication failed - tag mismatch");
			return -EBADMSG;
		}
		LOG_ERR("CCM verify failed: %d", (int)status);
		return -EIO;
	}

	LOG_DBG("CCM decrypt: %zu bytes ciphertext -> %zu bytes plaintext (verified)",
	        ciphertext_len, output_len);
	return 0;
}

#endif /* IS_ENABLED(CONFIG_DECT_SECURITY_ENABLE) */
