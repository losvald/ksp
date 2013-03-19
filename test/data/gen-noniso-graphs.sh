#!/bin/bash
if [ "$1" == "--help" ]; then
    echo "Usage: <nauty-home>"
fi
[ $# -ge 1 ] || nauty_home=/usr/local/bin/nauty
echo "home = $nauty_home"
for i in `seq 3 9`; do
    f=graph${i}c;
    [ -f $f.g6 ] || wget http://cs.anu.edu.au/~bdm/data/$f.g6
    [ -f $f.gen.in ] || $nauty_home/showg $f.g6 -e | \
	grep -v 'Graph' | tail -n +2 > $f.gen.in
done
