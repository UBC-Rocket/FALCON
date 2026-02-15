#ifndef LWGPS_OPTS_H
#define LWGPS_OPTS_H

/* Use float instead of double to save memory on Cortex-M4 */
#define LWGPS_CFG_DOUBLE 0

/* We don't need satellite detail descriptors */
#define LWGPS_CFG_STATEMENT_GPGSV_SAT_DET 0

#endif
