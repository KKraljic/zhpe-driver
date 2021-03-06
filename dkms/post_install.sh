#!/bin/bash
set -e

# dkms_post_install.sh

HNAME="zhpe_offloaded_helper"
HDIR="/usr/local/libexec"   # optional on Debian/Ubuntu
HPATH="${HDIR}/${HNAME}"
PCONF="dkms/modprobe_zhpe_offloaded.conf"
PPATH="/etc/modprobe.d/zhpe_offloaded.conf"
MCONF="dkms/modules_zhpe_offloaded.conf"
MPATH="/etc/modules-load.d/zhpe_offloaded.conf"

(( $# == 2 )) || exit 1

ZHPE_OFFLOADED_HELPER=$1/$HNAME
KERNELVER=$2

# Setup on first version
if [ ! -f $HPATH ] ; then
    mkdir -p $HDIR
    cp $ZHPE_OFFLOADED_HELPER $HDIR
    cp $PCONF $PPATH
    cp $MCONF $MPATH
fi

ln $HPATH ${HPATH}-${KERNELVER}

exit 0
