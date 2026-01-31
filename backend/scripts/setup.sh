#!/bin/bash


source .env

ORG="${DOCKER_INFLUXDB_INIT_ORG:-bees}"
INFLUX_HOST="${INFLUX_HOST:-http://influxdb:8086}"
TOKEN_DESC="grafana-read-only-bees-raw"

# ===== Helper =====
gen_pw() {
  # 32 Zeichen, URL- & shell-sicher
  openssl rand -base64 48 | tr -d '=+/[:space:]' | cut -c1-32
}

set_env_var() {
  local key="$1"
  local value="$2"

  if grep -q "^${key}=" "$ENV_FILE" 2>/dev/null; then
    echo "🔒 $key already exists – keeping existing value"
  else
    echo "$key=$value" >> "$ENV_FILE"
    echo "✅ $key generated"
  fi
}






# ===== Main =====
ENV_FILE=".env.secrets"
touch "$ENV_FILE"

echo "🔐 Generating secrets in $ENV_FILE"
echo

set_env_var "GF_SECURITY_ADMIN_PASSWORD" "$(gen_pw)"
set_env_var "DOCKER_INFLUXDB_INIT_PASSWORD" "$(gen_pw)"
set_env_var "DOCKER_INFLUXDB_INIT_ADMIN_TOKEN" "$(gen_pw)"
set_env_var "MOSQUITTO_TELEGRAF_PASSWORD" "$(gen_pw)"


MOSQUITTO_DEVICE1_PASSWORD="$(gen_pw)"
set_env_var "MOSQUITTO_DEVICE1_PASSWORD" "$MOSQUITTO_DEVICE1_PASSWORD"

source $ENV_FILE

echo
echo "✨ Done."
echo "👉 Secrets stored in $ENV_FILE (do NOT commit this file!)"

touch .env.grafana-token

docker compose up -d
while [ "`docker inspect -f {{.State.Health.Status}} influxdb`" != "healthy" ]; do     sleep 2; done

# Bucket-ID ermitteln
BUCKET_ID=$(
  docker compose exec -T influxdb influx bucket list \
    --host "$INFLUX_HOST" \
    --org "$ORG" \
    --name "$DOCKER_INFLUXDB_INIT_BUCKET" \
    --token "$DOCKER_INFLUXDB_INIT_ADMIN_TOKEN" \
    --json | jq -r '.[0].id'
)

if [[ -z "$BUCKET_ID" || "$BUCKET_ID" == "null" ]]; then
  echo "❌ Bucket '$DOCKER_INFLUXDB_INIT_BUCKET' nicht gefunden."
  exit 1
fi

echo "🔐 Erstelle Read-only Token …"

AUTH_JSON=$(docker compose exec influxdb bash -c "influx auth create \
  --host "$INFLUX_HOST" \
  --org '$ORG' \
  --read-bucket '$BUCKET_ID' \
  --description '$TOKEN_DESC' \
  --token "$DOCKER_INFLUXDB_INIT_ADMIN_TOKEN" \
  --json")

TOKEN=$(echo "$AUTH_JSON" | jq -r '.token')
AUTH_ID=$(echo "$AUTH_JSON" | jq -r '.id')


echo 'GRAFANA_INFLUX_TOKEN="'$TOKEN'"' > .env.grafana-token

echo
echo "✅ Token erfolgreich erstellt und in .env.grafana-token gespeichert:"
echo "----------------------------------------"
echo "Auth ID : $AUTH_ID"
echo "Token   :"
echo "$TOKEN"
echo "----------------------------------------"
echo


echo "🚨 Please setup your scale with MQTT user device1 and password=$MOSQUITTO_DEVICE1_PASSWORD"
