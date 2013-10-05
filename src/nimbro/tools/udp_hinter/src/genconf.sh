#!/bin/bash

NAME=$(g++ -x c++ -S $1 -o- | grep "^_.*:$" | sed -e 's/:$//' | grep negotiateConnection | grep -v _GLOBAL)

cat << EOS
#ifndef UDP_HINTER_CONFIG_H
#define UDP_HINTER_CONFIG_H

const char* MANGLED_NEGOTIATE_CONNECTION = "$NAME";

#endif
EOS
