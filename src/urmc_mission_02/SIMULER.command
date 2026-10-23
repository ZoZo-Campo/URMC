#!/bin/zsh
cd -- "${0:A:h}/../.." || exit 1
/bin/bash tools/mac.sh 02
result=$?
read 'reply?Appuie sur Entrée pour fermer.'
exit "$result"
