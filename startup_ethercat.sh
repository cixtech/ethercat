#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# Script: startup_ethercat.sh
# Description: Initialize EtherCAT system with proper error handling and logging
# Author: Lihua Liu <lihua.liu@cixtech.com>
# Date: 2025-10-30

set -euo pipefail  # Exit on error, undefined vars, pipe failures

# Default LD_LIBRARY_PATH so the append below doesn't trip `set -u`.
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}

# Configuration
# dkms installs the EtherCAT .ko under /usr/lib/modules/${KVER}/updates/dkms/.
MODULES_PATH="/usr/lib/modules/$(uname -r)/updates/dkms/"

# Logging functions
log_info() {
    echo "[INFO] $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_error() {
    echo "[ERROR] $(date '+%Y-%m-%d %H:%M:%S') - $1" >&2
}

# Function to check if module is already loaded
is_module_loaded() {
    local module_name="$1"
    lsmod | grep -q "^$module_name\|^${module_name//_/-}" 2>/dev/null
}

# Function to unload module if it exists
unload_module() {
    local module_name="$1"
    if is_module_loaded "$module_name"; then
        log_info "Unloading existing $module_name module..."
        rmmod "$module_name" 2>/dev/null || {
            log_error "Failed to unload $module_name module"
            return 1
        }
    fi
}

# Error handling
cleanup_on_error() {
    log_error "Script failed."
    exit 1
}

trap cleanup_on_error ERR

# Main execution
main() {
    log_info "Starting EtherCAT initialization..."
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/xenomai/lib
    export PATH=$PATH:/usr/xenomai/bin
    # Update module dependencies
    depmod -a
    if is_module_loaded "ec_macb"; then
        unload_module "ec_macb"
    fi

    # Start EtherCAT master
    log_info "Starting EtherCAT master..."
    if is_module_loaded "ec_master"; then
        unload_module "ec_master"
    fi
    insmod $MODULES_PATH/ec_master.ko main_devices=FF:FF:FF:FF:FF:FF

    # Wait for master to initialize
    sleep 1

    # Load phylink module
    # phylink is kernel-builtin under /kernel/drivers/net/phy/, not in the
    # dkms path; modprobe resolves it via modules.dep.
    if is_module_loaded "phylink"; then
        log_info "phylink module already loaded"
    else
        log_info "Loading phylink module..."
        modprobe phylink || { log_error "Failed to load phylink"; exit 1; }
    fi

    # Unload MACB Module
    if is_module_loaded "macb"; then
        unload_module "macb"
    fi
    # Load ec_macb module
    if is_module_loaded "ec_macb"; then
        log_info "ec_macb module already loaded"
    else
        insmod "$MODULES_PATH/ec_macb.ko" || { log_error "Failed to load ec_macb.ko"; exit 1; }
    fi

    # Wait for slaves after NIC bind
    sleep 2

    # List EtherCAT slaves
    log_info "Listing EtherCAT slaves:"
    ethercat slave || log_error "Failed to list EtherCAT slaves"

    log_info "EtherCAT initialization completed."
}

# Run main function
main "$@"
