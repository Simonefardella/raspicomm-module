#!/bin/sh

COMPILETIME="`date "+%Y-%m-%d %H:%M:%S"`"
FLAGS="-DCOMPILETIME=\"\\\"$COMPILETIME\\\"\""
DEBUG="-DDEBUG"
CMDS=""
while [ -n "$1" ]; do
	case "$1" in
		nodebug)
			DEBUG=""
			;;
		*)
			CMDS="$CMDS $1"
			;;
	esac
	shift
done
FLAGS="-Werror -Wall $FLAGS $DEBUG"

yes '' | head -n 20
#make -f Makefile_rpi "KCPPFLAGS=$FLAGS -DCOMPILETIME=\"\\\"$COMPILETIME\\\"\"" $CMDS
echo "|$FLAGS|"
make -f Makefile "KCPPFLAGS=$FLAGS" $CMDS

