/* SPDX-License-Identifier: GPL-2.0 */
/* EC_RTDM is not defined here; master/Kbuild adds -DEC_RTDM when
 * CONFIG_XENOMAI=y. */

#ifndef CONFIG_H
#define CONFIG_H

#define PACKAGE "ethercat"
#define PACKAGE_NAME "ethercat"
#define PACKAGE_STRING "ethercat 1.6.8"
#define PACKAGE_TARNAME "ethercat"
#define PACKAGE_VERSION "1.6.8"
#define VERSION "1.6.8"

#define EC_MAX_NUM_DEVICES 1
#define EC_IDENT_WILDCARDS 1
#define EC_SII_ASSIGN 1
#define EC_RT_SYSLOG 1

#endif
