#!/bin/bash

BROKER="127.0.0.1"
PORT="1883"

TOPIC="stockwaage/device/1/env/telemetry"
TOPIC_SCLE="stockwaage/device/1/scale/1/telemetry"

source .env.secrets

USER="device1"
PASS="$MOSQUITTO_DEVICE1_PASSWORD"

# Zeitparameter
INTERVAL=600            # 10 Minuten in Sekunden
DAYS=7
NOW=$(date -u +%s)
START=$((NOW - DAYS*24*3600))

ts=$START

while [ $ts -le $NOW ]; do
  # Zufallswerte erzeugen
  TEMP=$(awk -v min=10 -v max=35 'BEGIN{srand(); printf "%.2f", min+rand()*(max-min)}')
  HUMI=$(awk -v min=30 -v max=90 'BEGIN{srand(); printf "%.2f", min+rand()*(max-min)}')
  PRESS=$(awk -v min=90000 -v max=105000 'BEGIN{srand(); printf "%d", min+rand()*(max-min)}')
  BAT=$(awk -v min=3.6 -v max=4.2 'BEGIN{srand(); printf "%.2f", min+rand()*(max-min)}')
  WGHT=$(awk -v min=22 -v max=100 'BEGIN{srand(); printf "%.2f", min+rand()*(max-min)}')

  ISO_TS=$(date -u -d "@$ts" +"%Y-%m-%dT%H:%M:%SZ")

  PAYLOAD=$(cat <<EOF
{
  "temp": $TEMP,
  "press": $PRESS,
  "humi": $HUMI,
  "bat": $BAT,
  "ts": "$ISO_TS"
}
EOF
)


  mosquitto_pub \
    -h "$BROKER" \
    -p "$PORT" \
    -t "$TOPIC" \
    -u "$USER" \
    -P "$PASS" \
    -m "$PAYLOAD"

  PAYLOAD=$(cat <<EOF
{
  "wght": $WGHT,
  "ts": "$ISO_TS"
}
EOF
)

  mosquitto_pub \
    -h "$BROKER" \
    -p "$PORT" \
    -t "$TOPIC_SCLE" \
    -u "$USER" \
    -P "$PASS" \
    -m "$PAYLOAD"

  echo "sent: $ISO_TS"

  ts=$((ts + INTERVAL))
done
