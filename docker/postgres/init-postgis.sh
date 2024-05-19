#!/bin/bash
set -e

# Create a new database called "gisnav" with PostGIS extension
psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" <<-EOSQL
    CREATE DATABASE gisnav;
    \c gisnav
    CREATE EXTENSION postgis;
EOSQL
