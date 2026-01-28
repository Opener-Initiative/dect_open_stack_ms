/* dect_mac/dect_mac_security.c */
#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <psa/crypto.h>

#include <mac/dect_mac_security.h>


LOG_MODULE_REGISTER(dect_mac_security, CONFIG_DECT_MAC_SECURITY_LOG_LEVEL);

/* Forward declarations for internal functions */
static int security_cmac_kdf(const uint8_t *key, const char *label, uint32_t id_local,
			     uint32_t id_peer, uint8_t *out_key);

static int security_cmac_compute(const uint8_t *key, size_t key_len, const uint8_t *data,
				 size_t data_len, uint8_t *tag, size_t tag_len);



/* Helper functions */
void security_build_iv(uint8_t *iv_out, uint32_t transmitter_long_rd_id,
		       uint32_t receiver_long_rd_id, uint32_t hpc, uint16_t psn)
{
	if (!iv_out) {
		LOG_ERR("IV Build: Output buffer is NULL");
		return;
	}

	/* As per ETSI TS 103 636-4, Table 5.9.1.3-1 */
	/* All fields are Big Endian in the IV */
	sys_put_be32(transmitter_long_rd_id, &iv_out[0]);
	sys_put_be32(receiver_long_rd_id, &iv_out[4]);
	sys_put_be32(hpc, &iv_out[8]);
	uint16_t psn_field_value = (psn & 0x0FFF) << 4;
	sys_put_be16(psn_field_value, &iv_out[12]);
	iv_out[14] = 0;
	iv_out[15] = 0;
}

int security_calculate_mic(const uint8_t *pdu_data_for_mic, size_t pdu_data_len,
			   const uint8_t *integrity_key, uint8_t *mic_out_5_bytes)
{
	printk("security_calculate_mic started... \n");
	if (!pdu_data_for_mic || pdu_data_len == 0 || !integrity_key || !mic_out_5_bytes) {
		return -EINVAL;
	}

	LOG_DBG("MIC_CALC: Attempting to import integrity key...");
	LOG_HEXDUMP_DBG(integrity_key, 16, "Integrity Key Material:");

	psa_status_t status;
	psa_key_id_t key_id = 0;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	uint8_t full_mic_tag[16];
	size_t actual_mic_len;

	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
	psa_set_key_algorithm(&attributes, PSA_ALG_CMAC);
	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attributes, 128);
	psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);

	status = psa_import_key(&attributes, integrity_key, 16, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("MIC Calc: Failed to import integrity key: %d", (int)status);
		return -EIO;
	}

	status = psa_mac_compute(key_id, PSA_ALG_CMAC, pdu_data_for_mic, pdu_data_len,
				 full_mic_tag, sizeof(full_mic_tag), &actual_mic_len);

	psa_destroy_key(key_id);

	if (status != PSA_SUCCESS) {
		LOG_ERR("MIC Calc: psa_mac_compute failed: %d", (int)status);
		return -EIO;
	}

	memcpy(mic_out_5_bytes, full_mic_tag, 5);
	return 0;
}

// int security_crypt_payload(uint8_t *payload_in_out, size_t len, const uint8_t *cipher_key,
// 			   uint8_t *iv, bool encrypt)
// {
// 	printk("security_crypt_payload started... \n");

// 	if (!payload_in_out || (len > 0 && !iv) || !cipher_key) {
// 		return -EINVAL;
// 	}
// 	if (len == 0) {
// 		return 0;
// 	}

// 	LOG_DBG("CRYPT: Attempting to import cipher key for %s...", encrypt ? "encryption" : "decryption");
// 	LOG_HEXDUMP_DBG(cipher_key, 16, "Cipher Key Material:");

// 	psa_status_t status;
// 	psa_key_id_t key_id = 0;
// 	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
// 	size_t output_len;
// 	psa_algorithm_t alg = PSA_ALG_CTR;
// 	psa_key_usage_t usage = encrypt ? PSA_KEY_USAGE_ENCRYPT : PSA_KEY_USAGE_DECRYPT;

// 	psa_set_key_usage_flags(&attributes, usage);
// 	psa_set_key_algorithm(&attributes, alg);
// 	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
// 	psa_set_key_bits(&attributes, 128);
//  psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);

// 	status = psa_import_key(&attributes, cipher_key, 16, &key_id);
// 	if (status != PSA_SUCCESS) {
// 		LOG_ERR("[]Crypt: Failed to import cipher key: %d", (int)status);
// 		return -EIO;
// 	}

// 	status = psa_cipher_encrypt(key_id, alg, payload_in_out, len,
//                                 payload_in_out, len, &output_len);					  

// 	psa_destroy_key(key_id);

// 	if (status != PSA_SUCCESS) {
// 		LOG_ERR("Crypt: %s failed: %d", encrypt ? "Encryption" : "Decryption", (int)status);
// 		return -EIO;
// 	}

// 	return 0;
// }
int security_crypt_payload(uint8_t *payload_in_out, size_t len, const uint8_t *cipher_key,
               uint8_t *iv, bool encrypt)
{
    if (!payload_in_out || (len > 0 && !iv) || !cipher_key) {
        return -EINVAL;
    }
    if (len == 0) {
        return 0;
    }
    
    /* Validate IV length */
    if (len > 0 && iv != NULL) {
        LOG_DBG("IV length validation passed");
    }

	LOG_DBG("CRYPT: Attempting to import cipher key for %s...", encrypt ? "encryption" : "decryption");
	LOG_HEXDUMP_DBG(cipher_key, 16, "Cipher Key Material:");

    psa_status_t status;
    psa_key_id_t key_id = 0;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_cipher_operation_t operation = PSA_CIPHER_OPERATION_INIT;
    size_t output_len = 0;
    size_t total_output_len = 0;
    psa_algorithm_t alg = PSA_ALG_CTR;
    int ret = 0;
    
    /* Set key attributes */
    psa_reset_key_attributes(&attributes);
    psa_set_key_usage_flags(&attributes, encrypt ? PSA_KEY_USAGE_ENCRYPT : PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, alg);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);
    
    /* Import the key */
    status = psa_import_key(&attributes, cipher_key, 16, &key_id);
    if (status != PSA_SUCCESS) {
        LOG_ERR("[MAC_SEC] Crypt: Failed to import cipher key: %d", (int)status);
        return -EIO;
    }
    
    /* Setup cipher operation */
    if (encrypt) {
        status = psa_cipher_encrypt_setup(&operation, key_id, alg);
    } else {
        status = psa_cipher_decrypt_setup(&operation, key_id, alg);
    }
    if (status != PSA_SUCCESS) {
        LOG_ERR("Crypt: Failed to setup cipher operation: %d", (int)status);
        ret = -EIO;
        goto exit_destroy_key;
    }
    
    /* Set IV (required for CTR mode) */
    status = psa_cipher_set_iv(&operation, iv, 16);
    if (status != PSA_SUCCESS) {
        LOG_ERR("Crypt: Failed to set IV: %d", (int)status);
        ret = -EIO;
        goto abort_operation;
    }
    
    /* Process the data in one go */
    status = psa_cipher_update(&operation, payload_in_out, len, 
                              payload_in_out, len, &output_len);
    if (status != PSA_SUCCESS) {
        LOG_ERR("Crypt: Failed to update cipher operation: %d", (int)status);
        ret = -EIO;
        goto abort_operation;
    }
    total_output_len += output_len;
    
    /* Finish the operation */
    status = psa_cipher_finish(&operation, 
                              payload_in_out + total_output_len, 
                              len - total_output_len, 
                              &output_len);
    if (status != PSA_SUCCESS) {
        LOG_ERR("Crypt: Failed to finish cipher operation: %d", (int)status);
        ret = -EIO;
        goto abort_operation;
    }
    total_output_len += output_len;
    
    /* Verify output length matches input length */
    if (total_output_len != len) {
        LOG_ERR("Crypt: Output length mismatch: %zu != %zu", total_output_len, len);
        ret = -EIO;
        goto abort_operation;
    }
    
    goto exit_destroy_key;

abort_operation:
    psa_cipher_abort(&operation);

exit_destroy_key:
    /* Destroy the key */
    status = psa_destroy_key(key_id);
    if (status != PSA_SUCCESS) {
        LOG_ERR("Crypt: Failed to destroy key: %d", (int)status);
        /* Don't overwrite previous error */
    }
    
    return ret;
}




int security_derive_auth_key(const uint8_t *master_key, uint8_t *out_auth_key)
{
	if (!master_key || !out_auth_key) {
		return -EINVAL;
	}
	return security_cmac_kdf(master_key, "DECT-NR-AuthKey", 0, 0, out_auth_key);
}

int security_generate_auth_mac(const uint8_t *auth_key, uint32_t pt_nonce, uint32_t ft_nonce,
			       uint32_t pt_long_id, uint32_t ft_long_id,
			       uint8_t *out_mac_8_bytes)
{
	if (!auth_key || !out_mac_8_bytes) {
		return -EINVAL;
	}

	uint8_t data_to_mac[16];
	sys_put_be32(pt_nonce, &data_to_mac[0]);
	sys_put_be32(ft_nonce, &data_to_mac[4]);
	sys_put_be32(pt_long_id, &data_to_mac[8]);
	sys_put_be32(ft_long_id, &data_to_mac[12]);

	uint8_t full_mic_tag[16];
	int err = security_cmac_compute(auth_key, 16, data_to_mac, sizeof(data_to_mac),
					full_mic_tag, sizeof(full_mic_tag));
	if (err != 0) {
		return err;
	}

	memcpy(out_mac_8_bytes, full_mic_tag, DECT_MAC_AUTH_MAC_SIZE);
	return 0;
}

int security_derive_session_keys(const uint8_t *auth_key, uint32_t id_local, uint32_t id_peer,
				 uint8_t *out_session_integrity_key,
				 uint8_t *out_session_cipher_key)
{
	if (!auth_key || !out_session_integrity_key || !out_session_cipher_key) {
		return -EINVAL;
	}

	int err;
	err = security_cmac_kdf(auth_key, "IntegrityKey", id_local, id_peer,
				out_session_integrity_key);
	if (err != 0) {
		LOG_ERR("Failed to derive integrity key: %d", err);
		return err;
	}

	err = security_cmac_kdf(auth_key, "CipheringKey", id_local, id_peer,
				out_session_cipher_key);
	if (err != 0) {
		LOG_ERR("Failed to derive cipher key: %d", err);
	}

	return err;
}

static int security_cmac_compute(const uint8_t *key, size_t key_len, const uint8_t *data,
				 size_t data_len, uint8_t *tag, size_t tag_len)
{
	psa_status_t status;
	psa_key_id_t key_id = 0; /* Use 0 as it's not a valid persistent key ID */
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	size_t mac_length;
	int ret = -EIO; /* Default to I/O error */

	if (!key || !data || !tag) {
		LOG_ERR("Invalid NULL parameter");
		return -EINVAL;
	}

	/* AES-CMAC only supports 128-bit keys in this implementation */
	if (key_len != 16) {
		LOG_ERR("Invalid key length: %zu (must be 16)", key_len);
		return -EINVAL;
	}

	/* The PSA API requires a buffer large enough for the full tag */
	if (tag_len < 16) {
		LOG_ERR("Tag buffer too small: %zu (must be at least 16)", tag_len);
		return -EMSGSIZE;
	}

	/* Set up key attributes for a transient AES-128 CMAC key */
	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attributes, 128);
	psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
	psa_set_key_algorithm(&attributes, PSA_ALG_CMAC);

	status = psa_import_key(&attributes, key, key_len, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Failed to import key: %d", (int)status);
		/* No key to destroy, so we can return directly */
		return -EIO;
	}

	status = psa_mac_compute(key_id, PSA_ALG_CMAC, data, data_len, tag, tag_len, &mac_length);
	if (status != PSA_SUCCESS) {
		LOG_ERR("CMAC computation failed: %d", (int)status);
		/* goto cleanup to ensure the imported key is destroyed */
		goto cleanup;
	}

	if (mac_length != 16) {
		LOG_ERR("Unexpected MAC length: %zu (expected 16)", mac_length);
		/* goto cleanup to ensure the imported key is destroyed */
		goto cleanup;
	}

	LOG_DBG("CMAC computed successfully, length: %zu", mac_length);
	ret = 0; /* Success */

cleanup:
	/* Always destroy the transient key to prevent resource leaks */
	status = psa_destroy_key(key_id);
	if (status != PSA_SUCCESS) {
		LOG_WRN("Failed to destroy key: %d", (int)status);
		/* If the main operation succeeded, we still return success,
		 * but the warning indicates a potential resource leak.
		 */
	}

	return ret;
}

static int security_cmac_kdf(const uint8_t *key, const char *label, uint32_t id_local,
			     uint32_t id_peer, uint8_t *out_key)
{
	uint8_t kdf_input[64];
	size_t current_len = 0;
	size_t label_len = strlen(label);

	if ((label_len + 11) > sizeof(kdf_input)) {
		return -EMSGSIZE;
	}

	kdf_input[current_len++] = 0x01;
	memcpy(&kdf_input[current_len], label, label_len);
	current_len += label_len;
	kdf_input[current_len++] = 0x00;
	sys_put_be32(id_local, &kdf_input[current_len]);
	current_len += sizeof(uint32_t);
	sys_put_be32(id_peer, &kdf_input[current_len]);
	current_len += sizeof(uint32_t);
	sys_put_be16(128, &kdf_input[current_len]);
	current_len += sizeof(uint16_t);

	return security_cmac_compute(key, 16, kdf_input, current_len, out_key, 16);
}

#endif /* IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) */