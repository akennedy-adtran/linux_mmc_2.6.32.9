#!/bin/sh
export CMD=$1
export TOPDIR=$2
export EXTERNAL_SOURCE_TREE=$3
if [ x = x"$1" -o x = x"$2" -o x = x"$3" ]; then
    echo 'Invalid parameters to create links:';
    echo 'Usage: symlinks_mgmt.sh [c|d] <topdir> <external_source_dir>';
    exit 1;
fi

if [ $CMD = 'c' ]; then
    if [ -f ${TOPDIR}/.linkscreated ]; then
        exit 0;
    fi

    (cd ${TOPDIR}/drivers/char/rmisec/phxdrv; ln -s ${EXTERNAL_SOURCE_TREE}/linux/security/sec_api.c .)
    (cd ${TOPDIR}/drivers/char/rmisec/phxdrv; ln -s ${EXTERNAL_SOURCE_TREE}/linux/security/rmisec_internal.h .)
    (cd ${TOPDIR}/include/asm-mips/rmi/; ln -s ${EXTERNAL_SOURCE_TREE}/linux/security/rmisae.h .)

    touch ${TOPDIR}/.linkscreated
else
# clean up operation
# remove all symbolic links created
rm -f ${TOPDIR}/drivers/char/rmisec/phxdrv/sec_api.c
rm -f ${TOPDIR}/drivers/char/rmisec/phxdrv/rmisec_internal.h
rm -f ${TOPDIR}/include/asm-mips/rmi/rmisae.h

rm -f ${TOPDIR}/.linkscreated    
fi
