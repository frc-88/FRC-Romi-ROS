#!/usr/bin/env bash

echo "Running romi_bridge systemd service copy script"

BASE_DIR=$(realpath "$(dirname $0)")

BASE_INSTALL_DIR=/etc/systemd/system
BIN_INSTALL_DIR=/usr/bin

SCRIPT_NAME=romi_bridge.sh
SERVICE_NAME=romi_bridge.service

chmod +x ${BASE_DIR}/${SCRIPT_NAME}

echo "Copying service files"
cp ${BASE_DIR}/${SERVICE_NAME} ${BASE_INSTALL_DIR}
cp ${BASE_DIR}/${SCRIPT_NAME} ${BIN_INSTALL_DIR}

echo "romi_bridge systemd service copy complete"
