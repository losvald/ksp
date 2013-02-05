#!/bin/sh
autoreconf --force --install $(dirname $(readlink -f $0))
