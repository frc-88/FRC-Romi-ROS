#!/usr/bin/env bash

echo "Running romi ros systemd service uninstall script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=/etc/systemd/system/ros
fi

SERVICE_NAME=roscore.service
LAUNCH_SERVICE_NAME=roslaunch.service

rm -r ~/.config/systemd/systemd/${SERVICE_NAME}
rm -r ~/.config/systemd/systemd/${LAUNCH_SERVICE_NAME}

echo "Disabling systemd services"
systemctl stop ${SERVICE_NAME}
systemctl stop ${LAUNCH_SERVICE_NAME}

systemctl disable ${SERVICE_NAME}
systemctl disable ${LAUNCH_SERVICE_NAME}
systemctl daemon-reload
echo "romi ros systemd service uninstallation complete"
