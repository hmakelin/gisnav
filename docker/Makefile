SHELL := /bin/bash

# Supported autopilots, must match docker/docker-compose.yaml service name
AUTOPILOTS := px4 ardupilot

# Prefix for Docker Compose service images
PROJECT_NAME := gisnav

# List of Docker Compose service names that need GUI access
GUI_SERVICES = px4 ardupilot qgc rviz gisnav fileserver

# Expose xhost to containers that need GUI (see x11 extension in docker-compose.yaml)
expose-xhost:
	@if ! command -v xhost &> /dev/null; then \
		echo -e "\033[1;33;40mWarning: xhost command not found. GUIs will not be available.\033[0m"; \
	else \
		for containerId in `docker ps -f name=$(PROJECT_NAME) -aq`; do \
			serviceName=`docker inspect --format '{{index .Config.Labels "com.docker.compose.service" }}' $$containerId`; \
			if [ ! -z $$serviceName ] && echo $(GUI_SERVICES) | grep -w $$serviceName > /dev/null; then \
				xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $$containerId`; \
			fi; \
		done; \
	fi
