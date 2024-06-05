#!/bin/bash
set -e

# Check if the database exists
DB_EXISTS=$(psql -U "$POSTGRES_USER" -d postgres -tAc "SELECT 1 FROM pg_database WHERE datname='$GISNAV_DB'")
if [ "$DB_EXISTS" != "1" ]; then
  # Create the gisnav database
  psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" <<-EOSQL
      CREATE DATABASE $GISNAV_DB;
EOSQL
else
  echo "Database $GISNAV_DB already exists"
fi

# Enable PostGIS extension on the gisnav database
psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" --dbname "$GISNAV_DB" <<-EOSQL
      CREATE EXTENSION IF NOT EXISTS postgis;
EOSQL

# Create the feature schema if it doesn't exist
psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" --dbname "$GISNAV_DB" <<-EOSQL
      CREATE SCHEMA IF NOT EXISTS feature;
EOSQL

# Create feature.position if it does not exist
psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" --dbname "$GISNAV_DB" <<-EOSQL
      CREATE TABLE IF NOT EXISTS feature.position (
          id SERIAL PRIMARY KEY,
          geom GEOMETRY(Point, 4326),
          timestamp TIMESTAMPTZ DEFAULT NOW()
      );
EOSQL
