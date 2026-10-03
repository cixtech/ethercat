#-----------------------------------------------------------------------------
#
#  Copyright (C) 2006-2026  Florian Pose, Ingenieurgemeinschaft IgH
#
#  This file is part of the IgH EtherCAT Master.
#
#  The IgH EtherCAT Master is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License version 2, as
#  published by the Free Software Foundation.
#
#  The IgH EtherCAT Master is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
#  Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with the IgH EtherCAT Master; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
#-----------------------------------------------------------------------------

include $(top_srcdir)/Makefile.kbuild
all-local:
	@echo "Building macb kernel module"

install-data-local:
	@echo "Installing macb kernel module (handled by modules_install)"

clean-local:
	@echo "Cleaning macb kernel module"

distclean-local:
	@echo "Distcleaning macb kernel module"
	-rm -f *.o *.ko *.mod.c .*.cmd *.markers *.order *.symvers *.mod
	-rm -rf .tmp_versions

.PHONY: all install clean distclean uninstall
.PHONY: all-local install-data-local install-exec-local clean-local distclean-local uninstall-local

EXTRA_DIST = \
	macb-6.6-orig.h \
	macb-6.6-ethercat.h \
	macb_ptp-6.6-orig.h \
	macb_ptp-6.6-ethercat.h \
	macb_main-6.6-ethercat.c \
	macb_main-6.6-orig.c \
	macb_ptp-6.6-ethercat.c \
	macb_ptp-6.6-orig.c \
	update.sh

install-exec-local:
	@echo "No executable files to install for macb"

uninstall-local:
	@echo "Uninstalling macb module"
#-----------------------------------------------------------------------------