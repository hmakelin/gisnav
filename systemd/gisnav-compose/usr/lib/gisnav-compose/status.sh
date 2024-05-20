#!/bin/bash

# Define common variables
project_name="gisnav"

# Launch Docker Compose with the determined options
docker compose -p $project_name ps
