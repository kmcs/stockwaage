#!/usr/bin/env bash
set -euo pipefail

HOST="${INFLUX_HOST:-http://influxdb:8086}"
ORG="${DOCKER_INFLUXDB_INIT_ORG:-bees}"
TOKEN="${DOCKER_INFLUXDB_INIT_ADMIN_TOKEN:?Missing DOCKER_INFLUXDB_INIT_ADMIN_TOKEN}"

echo "Configuring influx CLI..."

echo "token=$TOKEN" 
influx config create \
  --config-name bee-stack \
  --host-url "$HOST" \
  --org "$ORG" \
  --token "$TOKEN" \
  --active >/dev/null 2>&1 || true

create_bucket () {
  local name="$1"
  if influx bucket list --org "$ORG" --name "$name" --hide-headers | grep -q "$name"; then
    echo "Bucket exists: $name"
  else
    echo "Creating bucket: $name"
    influx bucket create --org "$ORG" --name "$name" --retention 0 >/dev/null
  fi
}

create_bucket "$DOCKER_INFLUXDB_INIT_BUCKET"

echo "Done."

