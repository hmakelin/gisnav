# This Makefile is used as a Docker container orchestration tool to deploy
# various Docker Compose services that support GISNav.
#
# The terminology used in this Makefile includes the following:
#   - FMU: Flight Management Unit, the onboard flight controller computer
#		board (e.g., Pixhawk) that runs the autopilot software.
#   - HIL: Hardware-In-The-Loop, a simulation mode where the autopilot runs
#		onboard the FMU.
#   - SITL: Software-In-The-Loop, a simulation mode where the autopilot is
#		executed offboard on a separate computer, replicating the drone's
#		flight dynamics and sensors, enabling testing and development without
#		using actual hardware.
#   - Offboard: Refers to services that run on a computer not carried by the
#		drone and not powered by the drone battery (e.g., a powerful desktop).
#   - Onboard: Refers to services that run on the drone itself, powered by the
#		drone's battery, like the FMU and onboard companion computer (e.g.,
#  		Nvidia Jetson Nano). In the case of GISNav, all onboard services are
#		intended to run on the companion computer and not e.g. on the FMU.
#   - Middleware: Software that sits between the autopilot and GISNav, enabling
#		communication between them.
#   - QGC: QGroundControl, a ground control station software used to monitor
#		and control drones.
#   - Gazebo: A robotics simulator used to simulate the drone's environment and
#		its interactions with the world.
#
# The target options in the Makefile are as follows:
#   - onboard-hil-*: Targets for onboard hardware-in-the-loop services,
#		including GIS server, ROS middleware, autoheal, and GISNav.
#   - onboard-sitl-*: Targets for onboard software-in-the-loop services, with
#		the same services as onboard HIL but using a different middleware
#		configuration.
#   - offboard-sitl-*: Targets for offboard software-in-the-loop services,
#		including Gazebo simulation and QGroundControl.
#   - offboard-sitl-test-*: Targets for SITL testing services, which exclude
#		GISNav and QGC for automated testing scenarios. Gazebo in headless mode.
#   - offboard-sitl-dev-*: Targets for SITL development services, including
#		Gazebo simulation, ROS middleware, mapserver, and QGC, but
#		excluding GISNav whose development version is assumed to be run locally.
#   - demo-*: Shortcut targets for demo purposes, setting up all SITL services
#		offboard, including GIS server, ROS middleware, Gazebo simulation, QGC,
#		and GISNav.

SHELL := /bin/bash

# Supported autopilots, must match docker/docker-compose.yaml service name
AUTOPILOTS := px4 ardupilot

# Prefix for Docker Compose service images
PROJECT_NAME := gisnav

# Define a reusable template for creating Docker Compose targets
# Parameters:
#   1. The prefix for the target name.
#   2. The dependency target.
#   3. The optional Docker Compose override files to include (if any).
#   4. The Docker Compose services to start or build - use $$* here for
#		autopilot, the first '$' is to escape the second '$'.
define compose_template

# Create phony create targets with the specified prefix for each autopilot
.PHONY: $(addprefix create-$(1)-, $(AUTOPILOTS))

# Generate rules for the build prefixed targets, depending on the specified build dependency target
$(addprefix create-$(1)-, $(AUTOPILOTS)): $(if $(2),create-$(1)-%: create-$(2)-%,create-$(2)-%)
	# Evaluate the additional Docker Compose files
	$$(eval compose_files := $(if $(3),-f docker-compose.yaml $(foreach file,$(3),-f $(file))))
	# Build the specified Docker Compose services and create the containers
	@docker compose -p $(PROJECT_NAME) $$(compose_files) create $(4)

# Create phony build targets with the specified prefix for each autopilot
.PHONY: $(addprefix build-$(1)-, $(AUTOPILOTS))

# Generate rules for the build prefixed targets, depending on the specified build dependency target
$(addprefix build-$(1)-, $(AUTOPILOTS)): $(if $(2),build-$(1)-%: build-$(2)-%,build-$(2)-%)
	# Evaluate the additional Docker Compose files
	$$(eval compose_files := $(if $(3),-f docker-compose.yaml $(foreach file,$(3),-f $(file))))
	# Build the specified Docker Compose services
	@docker compose -p $(PROJECT_NAME) $$(compose_files) build $(4)

# Create phony up targets with the specified prefix for each autopilot
.PHONY: $(addprefix up-$(1)-, $(AUTOPILOTS))

# Generate rules for the prefixed up targets, depending on the specified up
# dependency target
# IMPORTANT: Need to depend on create targets here before up targets to first
# expose xhost to containers before running them
$(addprefix up-$(1)-, $(AUTOPILOTS)): $(if $(2),up-$(1)-%: create-$(1)-% expose-xhost up-$(2)-%,up-$(1)-%: create-$(1)-% expose-xhost)
	# Evaluate the additional Docker Compose files
	$$(eval compose_files := $(if $(3),-f docker-compose.yaml $(foreach file,$(3),-f $(file))))
	# Run Docker Compose with the specified services
	@docker compose -p $(PROJECT_NAME) $$(compose_files) up -d $(4)
endef

# Define a reusable template for creating Docker Compose middleware targets
#
# Parameters:
#   1. The prefix for the target name.
#   2. The additional Docker Compose options, such as file overrides.
define middleware_template
.PHONY: up-$(1)-% build-$(1)-% create-$(1)-%

up-$(1)-%:
	$$(call run_middleware, $(2), up -d)

build-$(1)-%:
	$$(call run_middleware, $(2), build)

create-$(1)-%:
	$$(call run_middleware, $(2), create)
endef

# The run_middleware function is a helper function for executing middleware
# targets with the necessary Docker Compose options.
#
# Parameters:
#   1. Additional Docker Compose options, such as file overrides.
#   2. The Docker Compose command to execute (either 'up -d' or 'build').
define run_middleware
	@if [ "$*" = "px4" ]; then \
		docker compose -p $(PROJECT_NAME) $(1) $(2) mavros; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -p $(PROJECT_NAME) $(1) $(2) mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi
endef

# The empty argument check for some reason is not working in the compose_template.
# So we will pass a dummy dependency as a dependency target instead of an empty
# argument.
.PHONY: $(addprefix build-dummy-dependency-, $(AUTOPILOTS))
.PHONY: $(addprefix create-dummy-dependency-, $(AUTOPILOTS))
.PHONY: $(addprefix up-dummy-dependency-, $(AUTOPILOTS))

build-dummy-dependency-px4:

create-dummy-dependency-px4:

up-dummy-dependency-px4:

build-dummy-dependency-ardupilot:

create-dummy-dependency-ardupilot:

up-dummy-dependency-ardupilot:

# Define middleware targets
$(eval $(call middleware_template,onboard-hil-middleware,-f docker-compose.yaml -f docker-compose.serial.yaml))
$(eval $(call middleware_template,onboard-sitl-middleware,))
$(eval $(call middleware_template,offboard-sitl-middleware,))  # same as onboard

# onboard HIL services: GIS server, ROS middleware, autoheal, gscam and GISNav
$(eval $(call compose_template,onboard-hil,onboard-hil-middleware,docker-compose.arm64.yaml,mapserver autoheal gscam gisnav))

# onboard SITL services: Same as with HIL but middleware is same as offboard (UDP, not serial)
$(eval $(call compose_template,onboard-sitl,offboard-sitl-middleware,docker-compose.arm64.yaml,mapserver gscam gisnav))

# offboard SITL services: Gazebo simulation, QGC
$(eval $(call compose_template,offboard-sitl,dummy-dependency,,$$* qgc))

# SITL testing services: Gazebo simulation, ROS middleware, mapserver, gscam, but excluding GISNav and QGC
$(eval $(call compose_template,offboard-sitl-test,offboard-sitl-middleware,docker-compose.headless.yaml,$$* gscam mapserver))

# SITL development services: Gazebo simulation, ROS middleware, mapserver, QGC, gscam rviz, but excluding GISNav
$(eval $(call compose_template,offboard-sitl-dev,offboard-sitl-middleware,,$$* qgc gscam mapserver rviz qgis))

# All SITL services offboard: GIS server, ROS middleware, Gazebo simulation, QGC, gscam, gisnav
$(eval $(call compose_template,demo,offboard-sitl-dev,,gisnav))

# List of Docker Compose service names that need GUI access
GUI_SERVICES = px4 ardupilot qgc rviz qgis gisnav fileserver

# Expose xhost to containers that need GUI (see x11 extension in docker-compose.yaml)
expose-xhost:
	@for containerId in `docker ps -f name=$(PROJECT_NAME) -aq`; do \
		serviceName=`docker inspect --format '{{index .Config.Labels "com.docker.compose.service" }}' $$containerId`; \
		if [ ! -z $$serviceName ] && echo $(GUI_SERVICES) | grep -w $$serviceName > /dev/null; then \
			xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $$containerId`; \
		fi; \
	done

# shutdown any and all services (stop and remove containers)
.PHONY: down
down:
	@docker compose -p $(PROJECT_NAME) down

# start existing containers
# Note: You should create the containers with the "make create-..." first,
# otherwise you will get an error like "no container found for project "gisnav":
# not found"
.PHONY: start
start:
	@docker compose -p $(PROJECT_NAME) start

# shutdown any and all services (stop but do not remove containers)
.PHONY: stop
stop:
	@docker compose -p $(PROJECT_NAME) stop

# build all services
# Note: This builds many services with overlapping functionality such as px4 and
# arudpilot, you may want to be more specific about what to build
.PHONY: build
build:
	@docker compose -p $(PROJECT_NAME) build

# Use this target to start supporting services when using a local installation
# of GISNav. The mapserver needs to be started first and the local /etc/hosts
# file updated so that the gisnav-mapserver-1 hostname resolves to the mapserver
# container ID on your local host. socat is used to create a virtual serial port
# (a pseudo-tty) and to bridge it to the TCP port which the px4 container
# is listening on.
.PHONY: base-dev-setup
base-dev-setup:
	# Recreate containers
	docker compose -p gisnav create mapserver px4 qgis gscam postgres rviz
	$(MAKE) expose-xhost

	# Start the mapserver container
	docker compose -p gisnav start mapserver postgres

	# Extract the IP address of the mapserver and postgres containers
	$(eval MAPSERVER_IP=`docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' gisnav-mapserver-1`)
	@echo "Debug: MAPSERVER_IP is '$(MAPSERVER_IP)'"
	$(eval POSTGRES_IP=`docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' gisnav-postgres-1`)
	@echo "Debug: POSTGRES_IP is '$(POSTGRES_IP)'"

	# Check if the entry already exists and update the /etc/hosts file
	@if grep -q "gisnav-mapserver-1" /etc/hosts; then \
		sudo sed -i'' -e "s/.*gisnav-mapserver-1/$(MAPSERVER_IP) gisnav-mapserver-1/" /etc/hosts; \
	else \
		echo "$(MAPSERVER_IP) gisnav-mapserver-1" | sudo tee -a /etc/hosts; \
	fi
	@echo "Updated /etc/hosts with IP $(MAPSERVER_IP) for gisnav-mapserver-1"

	@if grep -q "gisnav-postgres-1" /etc/hosts; then \
		sudo sed -i'' -e "s/.*gisnav-postgres-1/$(POSTGRES_IP) gisnav-postgres-1/" /etc/hosts; \
	else \
		echo "$(POSTGRES_IP) gisnav-postgres-1" | sudo tee -a /etc/hosts; \
	fi
	@echo "Updated /etc/hosts with IP $(POSTGRES_IP) for gisnav-postgres-1"

	# Start the other development services
	@docker compose -p gisnav start px4 qgis gscam rviz

.PHONY: dev-nmea
dev-nmea: base-dev-setup
	# Specific setup for NMEA output
	@echo "Setting up NMEA output..."
	$(eval PX4_IP=`docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' gisnav-px4-1`)
	@echo "Debug: PX4_IP is '$(PX4_IP)'"
	@socat pty,link=/tmp/gisnav-pty-link,raw,echo=0 tcp:$(PX4_IP):15000 || echo "Could not establish serial-to-TCP bridge" &
	@sleep 3  # Give socat time to create the pty
	@echo PTS device created at: `readlink /tmp/gisnav-pty-link`
	@ros2 launch gisnav default.launch.py protocol:=nmea port:=`readlink /tmp/gisnav-pty-link` baudrate:=115200

.PHONY: dev-uorb
dev-uorb: base-dev-setup
	@ros2 launch gisnav dev.launch.py protocol:=uorb

# shortcut for demo
.PHONY: demo
demo:
	docker compose -p gisnav create gisnav
	$(MAKE) expose-xhost
	docker compose -p gisnav up gisnav
