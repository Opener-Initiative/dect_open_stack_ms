/* dect_nrplus/include/dect_security_crypto.h */
/**
 * @file dect_security_crypto.h
 * @brief Unified cryptography API for DECT NR+ security
 *
 * This module provides a hardware-agnostic interface for cryptographic
 * operations required by both MAC and CVG layer security. It abstracts
 * the underlying crypto implementation (PSA Crypto, TinyCrypt, etc.)
 * to allow different backends based on platform capabilities.
 */

#ifndef DECT_SECURITY_CRYPTO_H__
#define DECT_SECURITY_CRYPTO_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Compute AES-128-CMAC tag
 *
 * Computes full 16-byte CMAC tag and optionally truncates to requested length.
 * Used for MAC integrity protection and authentication handshake.
 *
 * @param key Pointer to 16-byte AES-128 key
 * @param key_len Must be 16 for AES-128
 * @param data Input data to authenticate
 * @param data_len Length of input data
 * @param mac_out Output buffer for MAC tag
 * @param mac_len Desired MAC length (typically 5 or 8 bytes for DECT)
 * @return 0 on success, negative error code on failure
 */
int dect_crypto_aes_cmac(const uint8_t *key, size_t key_len,
                         const uint8_t *data, size_t data_len,
                         uint8_t *mac_out, size_t mac_len);

/**
 * @brief Encrypt or decrypt data using AES-128-CTR mode
 *
 * Performs in-place AES-CTR encryption/decryption. The operation is
 * symmetric - encryption and decryption are the same operation.
 *
 * @param key Pointer to 16-byte AES-128 cipher key
 * @param iv Pointer to 16-byte initialization vector (will be modified)
 * @param data_inout Input/output buffer (modified in place)
 * @param len Length of data to encrypt/decrypt
 * @return 0 on success, negative error code on failure
 */
int dect_crypto_aes_ctr_crypt(const uint8_t *key, uint8_t *iv,
                               uint8_t *data_inout, size_t len);

/**
 * @brief Encrypt and authenticate using AES-128-CCM
 *
 * Provides authenticated encryption with associated data (AEAD).
 * This is useful for future CVG security enhancements that may
 * require CCM mode instead of separate CMAC+CTR.
 *
 * @param key Pointer to 16-byte AES-128 key
 * @param nonce Pointer to nonce (IV) for CCM
 * @param nonce_len Length of nonce (typically 13 bytes for CCM)
 * @param aad Additional authenticated data (header)
 * @param aad_len Length of AAD
 * @param plaintext Input plaintext
 * @param plaintext_len Length of plaintext
 * @param ciphertext Output buffer for ciphertext
 * @param tag Output buffer for authentication tag
 * @param tag_len Desired tag length (typically 8 or 16 bytes)
 * @return 0 on success, negative error code on failure
 */
int dect_crypto_aes_ccm_encrypt_auth(const uint8_t *key,
                                       const uint8_t *nonce, size_t nonce_len,
                                       const uint8_t *aad, size_t aad_len,
                                       const uint8_t *plaintext, size_t plaintext_len,
                                       uint8_t *ciphertext,
                                       uint8_t *tag, size_t tag_len);

/**
 * @brief Decrypt and verify using AES-128-CCM
 *
 * Verifies authentication tag and decrypts data. Returns error if
 * authentication fails.
 *
 * @param key Pointer to 16-byte AES-128 key
 * @param nonce Pointer to nonce (IV) for CCM
 * @param nonce_len Length of nonce
 * @param aad Additional authenticated data (header)
 * @param aad_len Length of AAD
 * @param ciphertext Input ciphertext
 * @param ciphertext_len Length of ciphertext
 * @param tag Authentication tag to verify
 * @param tag_len Length of tag
 * @param plaintext Output buffer for plaintext
 * @return 0 on success, -EBADMSG if authentication fails, other negative on error
 */
int dect_crypto_aes_ccm_decrypt_verify(const uint8_t *key,
                                         const uint8_t *nonce, size_t nonce_len,
                                         const uint8_t *aad, size_t aad_len,
                                         const uint8_t *ciphertext, size_t ciphertext_len,
                                         const uint8_t *tag, size_t tag_len,
                                         uint8_t *plaintext);

/**
 * @brief Initialize the crypto subsystem
 *
 * Initializes the selected crypto backend. Should be called once during
 * system initialization before any crypto operations.
 *
 * @return 0 on success, negative error code on failure
 */
int dect_crypto_init(void);

#endif /* DECT_SECURITY_CRYPTO_H__ */
