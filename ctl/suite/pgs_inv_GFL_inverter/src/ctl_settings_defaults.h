/**
 * @file ctl_settings_defaults.h
 * @brief Compatibility include for the shared GFL inverter SDPE contract.
 */

#ifndef FILE_PGS_INV_GFL_CTL_SETTINGS_DEFAULTS_H
#define FILE_PGS_INV_GFL_CTL_SETTINGS_DEFAULTS_H

#include <sdpe_pgs_inv_gfl_common_settings.h>

/*
 * BUILD_LEVEL 2:
 * Basic d/q-axis current closed-loop commissioning references.
 *
 * Id = +0.05 pu: 5% rated d-axis current
 * Iq =  0.00 pu: zero q-axis current
 */
#ifndef GFL_CURRENT_LEVEL2_ID_PU
#define GFL_CURRENT_LEVEL2_ID_PU  (+0.05f)
#endif

#ifndef GFL_CURRENT_LEVEL2_IQ_PU
#define GFL_CURRENT_LEVEL2_IQ_PU  (0.0f)
#endif

#endif // FILE_PGS_INV_GFL_CTL_SETTINGS_DEFAULTS_H
