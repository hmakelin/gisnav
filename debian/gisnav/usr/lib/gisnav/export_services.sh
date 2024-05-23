#!/bin/bash
# Exports top-level Docker Compose services to deploy in the
# GISNAV_SERVICES environment variable. The environment variable does not have
# to and should not include any potential dependencies - the dependencies should
# be defined in the 'depends_on' key in the Docker Compose configuration files.

export GISNAV_SERVICES=gisnav
