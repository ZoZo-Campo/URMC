#!/bin/zsh
cd -- "${0:A:h}" || exit 1
mkdir -p livraison
COPYFILE_DISABLE=1 tar -czf livraison/urmc_ros2.tar.gz --exclude='__pycache__' --exclude='*.pyc' -C . src tests tools kit_robot README.md docs
result=$?
if [[ "$result" == 0 ]]; then
  shasum -a 256 livraison/urmc_ros2.tar.gz > livraison/urmc_ros2.tar.gz.sha256
  print 'Archive créée : livraison/urmc_ros2.tar.gz'
fi
read 'reply?Appuie sur Entrée pour fermer.'
exit "$result"
