#ifndef MBEDTLS_NATIVE_SIM_CONFIG_H
#define MBEDTLS_NATIVE_SIM_CONFIG_H

/* Enable PSA Crypto support inside MbedTLS */
#define MBEDTLS_PSA_CRYPTO_C

/* The generic Cipher wrapper (Required by CMAC) */
#define MBEDTLS_CIPHER_C

/* Enable the required algorithms for DECT NR+ */
#define MBEDTLS_CMAC_C
#define MBEDTLS_AES_C
#define MBEDTLS_CIPHER_MODE_CTR

#endif /* MBEDTLS_NATIVE_SIM_CONFIG_H */