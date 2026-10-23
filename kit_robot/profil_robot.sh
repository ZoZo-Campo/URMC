# À compléter après inventaire ; conserver ce fichier sur le robot.
export ROS_DOMAIN_ID="" # Domaine du bringup à confirmer, ne pas deviner.
SCAN_TOPIC=/scan
IMAGE_TOPIC=/image_raw/compressed
IMAGE_TRANSPORT=compressed # ou raw, avec IMAGE_TOPIC=/image_raw
ODOM_TOPIC=/odom
CMD_TOPIC=/cmd_vel # doit accepter geometry_msgs/msg/Twist
