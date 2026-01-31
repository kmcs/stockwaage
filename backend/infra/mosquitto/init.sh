#!/bin/sh

if [ ! -s /mosquitto/config/passwd ]; then
  echo "Creating default mosquitto user 'telegraf' with password from MOSQUITTO_TELEGRAF_PASSWORD env var"
  : "${MOSQUITTO_TELEGRAF_PASSWORD:?Missing MOSQUITTO_TELEGRAF_PASSWORD}"
  touch /mosquitto/config/passwd
  chmod 0700 /mosquitto/config/passwd
  mosquitto_passwd -c -b /mosquitto/config/passwd telegraf "$MOSQUITTO_TELEGRAF_PASSWORD"
  echo "Creating mosquitto user 'device1' with password from MOSQUITTO_DEVICE1_PASSWORD env var"
  : "${MOSQUITTO_DEVICE1_PASSWORD:?Missing MOSQUITTO_DEVICE1_PASSWORD}"
  mosquitto_passwd -b /mosquitto/config/passwd device1 "$MOSQUITTO_DEVICE1_PASSWORD"
else
  echo "Mosquitto password file already exists, skipping user creation"
fi