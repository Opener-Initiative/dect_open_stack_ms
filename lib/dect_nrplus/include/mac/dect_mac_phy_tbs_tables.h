


#ifndef DECT_MAC_PHY_TBS_TABLES_H__
#define DECT_MAC_PHY_TBS_TABLES_H__

#include <stdint.h>

// Constants for table dimensions
#define TBS_MAX_MCS_INDEX 11   // MCS0 to MCS11
#define TBS_MAX_SUB_SLOTS_J 16 // Corresponds to j=1 to 16 subslots (PCC packet_length field 0-15)

// Declare arrays as extern
extern const uint32_t tbs_single_slot_mu1_beta1[TBS_MAX_MCS_INDEX + 1][TBS_MAX_SUB_SLOTS_J];
extern const uint32_t tbs_single_slot_mu2_beta1[TBS_MAX_MCS_INDEX + 1][TBS_MAX_SUB_SLOTS_J];
extern const uint32_t tbs_single_slot_mu4_beta1[TBS_MAX_MCS_INDEX + 1][TBS_MAX_SUB_SLOTS_J];

#endif /* DECT_MAC_PHY_TBS_TABLES_H__ */