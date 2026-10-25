#!/bin/zsh
cd -- "${0:A:h}/.." || exit 1
colima status >/dev/null 2>&1 || colima start || exit 1
docker build -t urmc-slam:local -f Dockerfile.slam . || exit 1
print 'Banc SLAM prêt. Ouvrir la console → Enceinte / SLAM → Banc sur Mac.'
