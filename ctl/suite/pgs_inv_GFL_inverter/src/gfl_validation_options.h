/**
 * @file gfl_validation_options.h
 * @brief SIL-only feature overrides for staged GFL validation.
 *
 * These macros are intentionally opt-in. A target that does not define them
 * retains the BUILD_LEVEL behaviour used by the hardware projects.
 */

#ifndef FILE_PGS_INV_GFL_VALIDATION_OPTIONS_H
#define FILE_PGS_INV_GFL_VALIDATION_OPTIONS_H

#ifndef GMP_GFL_ENABLE_NEGATIVE_SEQUENCE
#define GMP_GFL_ENABLE_NEGATIVE_SEQUENCE (BUILD_LEVEL >= 3 && BUILD_LEVEL <= 5)
#endif

#ifndef GMP_GFL_ENABLE_EXTERNAL_FEEDFORWARD
#define GMP_GFL_ENABLE_EXTERNAL_FEEDFORWARD (BUILD_LEVEL >= 4 && BUILD_LEVEL <= 5)
#endif

#ifndef GMP_GFL_ENABLE_DECOUPLING
#define GMP_GFL_ENABLE_DECOUPLING (BUILD_LEVEL >= 4 && BUILD_LEVEL <= 5)
#endif

#ifndef GMP_GFL_ENABLE_ACTIVE_DAMPING
#define GMP_GFL_ENABLE_ACTIVE_DAMPING (BUILD_LEVEL >= 4 && BUILD_LEVEL <= 5)
#endif

#ifndef GMP_GFL_ENABLE_LEAD_COMPENSATOR
#define GMP_GFL_ENABLE_LEAD_COMPENSATOR (BUILD_LEVEL >= 4 && BUILD_LEVEL <= 5)
#endif

/* 0 keeps the configured constant P/Q references; 1 enables the SIL profile. */
#ifndef GMP_GFL_PQ_PROFILE
#define GMP_GFL_PQ_PROFILE (0)
#endif

#endif // FILE_PGS_INV_GFL_VALIDATION_OPTIONS_H
