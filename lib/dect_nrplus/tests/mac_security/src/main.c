/*
 * Copyright (c) 2026 Manulytica Ltd
 */

/* lib/dect_nrplus/tests/mac_security/src/main.c */
// Overview: Adds a comprehensive Ztest suite for the MAC security module, verifying IV generation, MIC calculation (CMAC), and CTR encryption/decryption against standard test vectors.
#include <zephyr/ztest.h>
#include <mac/dect_mac_security.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>



#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <psa/crypto.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dect_mac_security_test, CONFIG_DECT_MAC_SECURITY_LOG_LEVEL);

/* --- Test Vectors --- */

/* NIST SP 800-38B Example D.1 for AES-128 CMAC */
static const uint8_t nist_cmac_key[16] = {
	0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
	0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};

static const uint8_t nist_cmac_msg[16] = {
	0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
	0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a
};

static const uint8_t nist_cmac_expected_mic_truncated[5] = { 0x07, 0x0a, 0x16, 0xb4, 0x6b };


/***************************************  UNUSED WARNING ***************************************/
// static const uint8_t nist_cmac_expected_mic_full[16] = {
// 	0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
// 	0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7d
// };



/* RFC 3686 Appendix B Test Vector for AES-128 CTR */
static const uint8_t rfc_ctr_key[16] = {
	0xAE, 0x68, 0x52, 0xF8, 0x12, 0x10, 0x67, 0xCC,
	0x4B, 0xF7, 0xA5, 0x76, 0x55, 0x77, 0xF3, 0x9E
};
static const uint8_t rfc_ctr_iv[16] = {
	0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};
static const uint8_t rfc_ctr_plaintext[16] = {
	0x53, 0x69, 0x6E, 0x67, 0x6C, 0x65, 0x20, 0x62,
	0x6C, 0x6F, 0x63, 0x6B, 0x20, 0x6D, 0x73, 0x67
};
static const uint8_t rfc_ctr_expected_ciphertext[16] = {
	0xE4, 0x09, 0x5D, 0x4F, 0xB7, 0xA7, 0xB3, 0x79,
	0x2D, 0x61, 0x75, 0xA3, 0x26, 0x13, 0x11, 0xB8
};

/* --- Test Setup --- */
static void *dect_mac_security_setup(void)
{
	psa_status_t status = psa_crypto_init();
printk("dect_mac_security_setup %d \n", status);

	zassert_equal(status, PSA_SUCCESS, "psa_crypto_init() failed with status: %d", (int)status);
	return NULL;
}

/* --- Test Cases --- */

/* ========================================================================= *
 *                      MAC SECURITY NEGATIVE TESTS                          *
 * ========================================================================= */

/**
 * @brief Negative Test: MIC Calculation with Tampered Payload
 * Proves that if a single bit of the payload changes over the air, 
 * the resulting MIC will be completely different, failing authentication.
 */
ZTEST(dect_mac_security, test_neg_mic_tampered_payload)
{
	uint8_t tampered_msg[16];
	uint8_t calculated_mic[5];
	int ret;

	/* Copy the NIST test message and flip exactly one bit */
	memcpy(tampered_msg, nist_cmac_msg, sizeof(nist_cmac_msg));
	tampered_msg[0] ^= 0x01; 

	ret = security_calculate_mic(tampered_msg, sizeof(tampered_msg),
					 nist_cmac_key, calculated_mic);
	zassert_ok(ret, "security_calculate_mic failed with code %d", ret);

	/* ASSERTION: The new MIC must NOT match the expected NIST MIC */
	zassert_false(memcmp(calculated_mic, nist_cmac_expected_mic_truncated, sizeof(calculated_mic)) == 0,
			  "SECURITY FAILED: Tampered payload produced a valid MIC!");
			  
	LOG_INF("[PASS] 1-bit payload modification resulted in completely invalid MIC.");
}

/**
 * @brief Negative Test: MIC Calculation with Incorrect Key
 * Proves that if a device tries to authenticate a packet using the wrong 
 * session integrity key, the MIC will not match.
 */
ZTEST(dect_mac_security, test_neg_mic_wrong_key)
{
	uint8_t wrong_key[16];
	uint8_t calculated_mic[5];
	int ret;

	/* Copy the NIST test key and flip exactly one bit */
	memcpy(wrong_key, nist_cmac_key, sizeof(nist_cmac_key));
	wrong_key[15] ^= 0xFF; 

	ret = security_calculate_mic(nist_cmac_msg, sizeof(nist_cmac_msg),
					 wrong_key, calculated_mic);
	zassert_ok(ret, "security_calculate_mic failed with code %d", ret);

	/* ASSERTION: The new MIC must NOT match the expected NIST MIC */
	zassert_false(memcmp(calculated_mic, nist_cmac_expected_mic_truncated, sizeof(calculated_mic)) == 0,
			  "SECURITY FAILED: Wrong key produced a valid MIC!");
			  
	LOG_INF("[PASS] Invalid integrity key resulted in completely invalid MIC.");
}

/**
 * @brief Negative Test: AES-CTR Ciphertext Corruption
 * AES-CTR mode is malleable. If an attacker flips a bit in the ciphertext, 
 * decryption will not explicitly "fail", but that exact bit will be flipped 
 * in the resulting plaintext. This proves that corrupted ciphertext results 
 * in corrupted plaintext (which the MIC would subsequently catch).
 */
ZTEST(dect_mac_security, test_neg_ctr_corrupted_ciphertext)
{
	uint8_t buffer[16];
	uint8_t iv[16];
	int ret;

	/* 1. Encrypt the standard plaintext */
	memcpy(buffer, rfc_ctr_plaintext, sizeof(buffer));
	memcpy(iv, rfc_ctr_iv, sizeof(iv));
	ret = security_crypt_payload(buffer, sizeof(buffer), rfc_ctr_key, iv, true);
	zassert_ok(ret, "Encryption failed");

	/* 2. Simulate over-the-air corruption: Flip a bit in the ciphertext */
	buffer[5] ^= 0x01;

	/* 3. Decrypt the corrupted ciphertext */
	memcpy(iv, rfc_ctr_iv, sizeof(iv)); /* Reset IV for decryption */
	ret = security_crypt_payload(buffer, sizeof(buffer), rfc_ctr_key, iv, false);
	zassert_ok(ret, "Decryption failed");

	/* ASSERTION: The decrypted buffer must NOT match the original plaintext */
	zassert_false(memcmp(buffer, rfc_ctr_plaintext, sizeof(buffer)) == 0,
			  "SECURITY FAILED: Corrupted ciphertext decrypted into valid plaintext!");
			  
	LOG_INF("[PASS] Ciphertext corruption successfully corrupted the resulting plaintext.");
}

/**
 * @brief Negative Test: IV Uniqueness and Sequence Numbers
 * In AES-CTR, reusing an IV is a catastrophic security failure.
 * This test proves that incrementing the Packet Sequence Number (PSN) 
 * generates a completely different IV, which in turn generates completely 
 * different ciphertext for the exact same plaintext.
 */
ZTEST(dect_mac_security, test_neg_iv_uniqueness_prevents_replay)
{
	uint8_t iv_packet_1[16];
	uint8_t iv_packet_2[16];
	uint8_t ciphertext_1[16];
	uint8_t ciphertext_2[16];
	
	/* Same connection details, but PSN increments from 1 to 2 */
	security_build_iv(iv_packet_1, 0x11223344, 0xAABBCCDD, 0x00000001, 1);
	security_build_iv(iv_packet_2, 0x11223344, 0xAABBCCDD, 0x00000001, 2);

	/* 1. Prove the IVs are distinct */
	zassert_false(memcmp(iv_packet_1, iv_packet_2, 16) == 0, 
	              "SECURITY FAILED: IVs are identical despite different PSNs!");

	/* 2. Encrypt the same plaintext with both IVs */
	memcpy(ciphertext_1, rfc_ctr_plaintext, 16);
	security_crypt_payload(ciphertext_1, 16, rfc_ctr_key, iv_packet_1, true);

	memcpy(ciphertext_2, rfc_ctr_plaintext, 16);
	security_crypt_payload(ciphertext_2, 16, rfc_ctr_key, iv_packet_2, true);

	/* 3. Prove the ciphertexts are distinct */
	zassert_false(memcmp(ciphertext_1, ciphertext_2, 16) == 0, 
	              "SECURITY FAILED: Identical ciphertext produced despite different IVs! (CTR IV reuse)");
	
	LOG_INF("[PASS] PSN increment generated unique IVs and distinct ciphertexts.");
}

/**
 * @brief Negative Test: Timing-Safe Comparison
 * Verifies that the custom constant-time array comparison function correctly
 * rejects arrays that are almost identical but differ at the end.
 */
ZTEST(dect_mac_security, test_neg_timing_safe_compare)
{
	uint8_t valid_mac[DECT_MAC_AUTH_MAC_SIZE] =   { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22 };
	uint8_t forged_mac[DECT_MAC_AUTH_MAC_SIZE] =  { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x00 };

	/* Only the very last byte differs */
	bool result = dect_mac_security_timing_safe_equal(valid_mac, forged_mac, DECT_MAC_AUTH_MAC_SIZE);

	zassert_false(result, "SECURITY FAILED: Timing-safe compare returned true for different arrays!");
	
	LOG_INF("[PASS] Timing-safe compare correctly rejected forged MAC.");
}

/* ========================================================================= *
 *                       MAC SECURITY POSITIVE TESTS                         *
 * ========================================================================= */

/**
 * @brief Test the IV Construction Logic
 * Verifies that the IV is constructed correctly by concatenating the TX ID,
 * RX ID, HPC, and Packet Sequence Number (PSN) in the correct order and bit positions.
 */
ZTEST(dect_mac_security, test_iv_build)
{
	uint8_t iv[16];
	uint32_t tx_id = 0xAABBCCDD;
	uint32_t rx_id = 0x11223344;
	uint32_t hpc = 0x12345678;
	uint16_t psn = 0x0ABC;

	uint8_t expected_iv[16] = {
		0xAA, 0xBB, 0xCC, 0xDD, /* TX ID */
		0x11, 0x22, 0x33, 0x44, /* RX ID */
		0x12, 0x34, 0x56, 0x78, /* HPC */
		0xAB, 0xC0, 0x00, 0x00  /* PSN (12 bits) shifted into place */
	};

	security_build_iv(iv, tx_id, rx_id, hpc, psn);

	zassert_mem_equal(iv, expected_iv, sizeof(expected_iv), "IV was not built correctly");
}

/**
 * @brief Test the MAC (CMAC) Calculation
 * Verifies that the Message Integrity Code (MIC) is calculated correctly
 * using the AES-128 CMAC algorithm based on NIST SP 800-38B test vectors.
 */
ZTEST(dect_mac_security, test_mic_calculation)
{
	uint8_t mic_out[5];

	int ret = security_calculate_mic(nist_cmac_msg, sizeof(nist_cmac_msg),
					 nist_cmac_key, mic_out);

	zassert_ok(ret, "security_calculate_mic failed with code %d", ret);
	zassert_mem_equal(mic_out, nist_cmac_expected_mic_truncated, sizeof(mic_out),
			  "Calculated MIC does not match NIST test vector");
}

/**
 * @brief Test the AES-CTR Encryption and Decryption Pipeline
 * Verifies that the CTR mode encryption produces the expected ciphertext
 * and that the same function can decrypt it back to the original plaintext.
 */
ZTEST(dect_mac_security, test_ctr_encryption_decryption)
{
	uint8_t buffer[16];
	uint8_t iv[16];
	int ret;

	/* --- Test Encryption --- */
	memcpy(buffer, rfc_ctr_plaintext, sizeof(buffer));
	memcpy(iv, rfc_ctr_iv, sizeof(iv));

	ret = security_crypt_payload(buffer, sizeof(buffer), rfc_ctr_key, iv, true);
	zassert_ok(ret, "Encryption failed with code %d", ret);
	zassert_mem_equal(buffer, rfc_ctr_expected_ciphertext, sizeof(buffer),
			  "Encrypted ciphertext does not match RFC test vector");

	/* --- Test Decryption --- */
	/* 'buffer' now holds the ciphertext, 'iv' was modified by the encrypt call */
	memcpy(iv, rfc_ctr_iv, sizeof(iv)); /* Reset IV for decryption */

	ret = security_crypt_payload(buffer, sizeof(buffer), rfc_ctr_key, iv, false);
	zassert_ok(ret, "Decryption failed with code %d", ret);
	zassert_mem_equal(buffer, rfc_ctr_plaintext, sizeof(buffer),
			  "Decrypted buffer does not match original plaintext");
}

/**
 * @brief Diagnostic Test: Direct PSA API Call
 * A low-level diagnostic test to verify that the PSA crypto API can be called
 * directly from the test code, bypassing the abstraction layer.
 */
ZTEST(dect_mac_security, test_direct_psa_api_call)
{
    psa_status_t status;
    psa_key_id_t key_id = 0;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    
    printk("\n--- DIAGNOSTIC TEST: Calling PSA API directly ---\n");
    
    /* Step 1: Initialize the PSA crypto subsystem */
    status = psa_crypto_init();
    zassert_equal(status, PSA_SUCCESS, "psa_crypto_init() failed with status: %d", (int)status);
    
    /* Step 2: Set up attributes for an AES key for CMAC */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attributes, PSA_ALG_CMAC);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);
    
    printk("Attempting to import key directly in test...\n");
    LOG_HEXDUMP_DBG(nist_cmac_key, sizeof(nist_cmac_key), "Key material:");

    /* Step 3: Attempt to import the key */
    status = psa_import_key(&attributes, nist_cmac_key, sizeof(nist_cmac_key), &key_id);
    
    /* Critical assertion */
    zassert_equal(status, PSA_SUCCESS, 
                 "Direct call to psa_import_key() failed with status: %d", (int)status);
    
    /* Clean up if successful */
    if (status == PSA_SUCCESS) {
        status = psa_destroy_key(key_id);
        zassert_equal(status, PSA_SUCCESS, 
                     "psa_destroy_key() failed with status: %d", (int)status);
    }
}



















static void print_psa_info(void)
{
    printk("PSA Crypto Implementation Info:\n");
    printk("  PSA_CRYPTO_DRIVER_CC310: %s\n",
           IS_ENABLED(CONFIG_PSA_CRYPTO_DRIVER_CC310) ? "enabled" : "disabled");
    printk("  PSA_CRYPTO_DRIVER_OBERON: %s\n",
           IS_ENABLED(CONFIG_PSA_CRYPTO_DRIVER_OBERON) ? "enabled" : "disabled");
    printk("  CRYPTO_NRF_CC310: %s\n",
           IS_ENABLED(CONFIG_CRYPTO_NRF_CC310) ? "enabled" : "disabled");
    printk("  MBEDTLS_PSA_CRYPTO_C: %s\n",
           IS_ENABLED(CONFIG_MBEDTLS_PSA_CRYPTO_C) ? "enabled" : "disabled");
    printk("  MBEDTLS_PSA_CRYPTO_CONFIG: %s\n",
           IS_ENABLED(CONFIG_MBEDTLS_PSA_CRYPTO_CONFIG) ? "enabled" : "disabled");
    printk("  PSA_CRYPTO_INIT: %s\n",
           IS_ENABLED(CONFIG_PSA_CRYPTO_INIT) ? "enabled" : "disabled");
}

/* Add to your test suite */
ZTEST(dect_mac_security, test_psa_info)
{
    print_psa_info();
    zassert_true(true, "PSA info check completed");
}

static void check_crypto_driver_status(void)
{
    printk("Crypto Driver Status:\n");
    
#ifdef CONFIG_PSA_CRYPTO_DRIVER_OBERON
    printk("Oberon driver: ENABLED\n");
#else
    printk("Oberon driver: DISABLED\n");
#endif
    
#ifdef CONFIG_CRYPTO_NRF_CC310
    printk("CC310 driver: ENABLED\n");
#else
    printk("CC310 driver: DISABLED\n");
#endif
    
#ifdef CONFIG_PSA_CRYPTO_DRIVER_CC310
    printk("PSA CC310 driver: ENABLED\n");
#else
    printk("PSA CC310 driver: DISABLED\n");
#endif
}

/* Add to your test suite */
ZTEST(dect_mac_security, test_driver_status)
{
    check_crypto_driver_status();
    zassert_true(true, "Driver check completed");
}

static void print_crypto_support(void)
{
    printk("PSA Crypto Support Status:\n");
#ifdef PSA_WANT_ALG_CMAC
    printk("CMAC algorithm: ENABLED\n");
#else
    printk("CMAC algorithm: DISABLED\n");
#endif
#ifdef PSA_WANT_KEY_TYPE_AES
    printk("AES key type: ENABLED\n");
#else
    printk("AES key type: DISABLED\n");
#endif
#ifdef CONFIG_PSA_CRYPTO_CLIENT
    printk("PSA Crypto Client: ENABLED\n");
#else
    printk("PSA Crypto Client: DISABLED\n");
#endif
}

ZTEST(dect_mac_security, test_crypto_support)
{
    print_crypto_support();
    zassert_true(true, "Support check completed");
}

ZTEST(dect_mac_security, test_psa_init)
{
    psa_status_t status = psa_crypto_init();
    zassert_equal(status, PSA_SUCCESS, "PSA init failed: %d", (int)status);
    printk("PSA Crypto initialized successfully\n");
}

static void print_psa_status(const char *operation, psa_status_t status)
{
    printk("%s: ", operation);
    switch (status) {
        case PSA_SUCCESS:
            printk("Success\n");
            break;
        case PSA_ERROR_NOT_PERMITTED:
            printk("Not permitted (-134)\n");
            break;
        case PSA_ERROR_INSUFFICIENT_MEMORY:
            printk("Insufficient memory\n");
            break;
        case PSA_ERROR_INSUFFICIENT_STORAGE:
            printk("Insufficient storage\n");
            break;
        case PSA_ERROR_COMMUNICATION_FAILURE:
            printk("Communication failure\n");
            break;
        case PSA_ERROR_HARDWARE_FAILURE:
            printk("Hardware failure\n");
            break;
        case PSA_ERROR_CORRUPTION_DETECTED:
            printk("Corruption detected\n");
            break;
        default:
            printk("Unknown error (%d)\n", status);
    }
}

ZTEST(dect_mac_security, test_key_attributes)
{
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    
    /* Set key attributes */
    psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    psa_set_key_algorithm(&attributes, PSA_ALG_CMAC);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    
    /* Verify attributes */
    zassert_equal(psa_get_key_type(&attributes), PSA_KEY_TYPE_AES, "Key type mismatch");
    zassert_equal(psa_get_key_bits(&attributes), 128, "Key bits mismatch");
    zassert_equal(psa_get_key_algorithm(&attributes), PSA_ALG_CMAC, "Algorithm mismatch");
    zassert_equal(psa_get_key_usage_flags(&attributes), PSA_KEY_USAGE_SIGN_MESSAGE, "Usage flags mismatch");
    zassert_equal(psa_get_key_lifetime(&attributes), PSA_KEY_LIFETIME_VOLATILE, "Lifetime mismatch");
    
    printk("Key attributes verified\n");
}

ZTEST(dect_mac_security, test_key_generation_minimal)
{
    psa_status_t status;
    psa_key_id_t key_id = 0;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    
    /* Initialize PSA */
    status = psa_crypto_init();
    zassert_equal(status, PSA_SUCCESS, "PSA init failed");
    
    /* Set minimal key attributes */
    psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    psa_set_key_algorithm(&attributes, PSA_ALG_CMAC);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    
    /* Generate key */
    status = psa_generate_key(&attributes, &key_id);
    print_psa_status("Key generation", status);
    zassert_equal(status, PSA_SUCCESS, "Key generation failed");
    
    /* Clean up */
    status = psa_destroy_key(key_id);
    print_psa_status("Key destroy", status);
    zassert_equal(status, PSA_SUCCESS, "Key destroy failed");
}




















/* --- Test Suite Definition --- */
ZTEST_SUITE(dect_mac_security, NULL, dect_mac_security_setup, NULL, NULL, NULL);