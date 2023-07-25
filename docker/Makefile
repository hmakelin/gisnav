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
#		Gazebo simulation, ROS middleware, mapserver, torch-serve, and QGC, but
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
	docker compose -p $(PROJECT_NAME) $$(compose_files) create --no-recreate $(4)

# Create phony build targets with the specified prefix for each autopilot
.PHONY: $(addprefix build-$(1)-, $(AUTOPILOTS))

# Generate rules for the build prefixed targets, depending on the specified build dependency target
$(addprefix build-$(1)-, $(AUTOPILOTS)): $(if $(2),build-$(1)-%: build-$(2)-%,build-$(2)-%)
	# Evaluate the additional Docker Compose files
	$$(eval compose_files := $(if $(3),-f docker-compose.yaml $(foreach file,$(3),-f $(file))))
	# Build the specified Docker Compose services
	docker compose -p $(PROJECT_NAME) $$(compose_files) build $(4)

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
	docker compose -p $(PROJECT_NAME) $$(compose_files) up -d $(4)
endef

# Define a reusable template for creating Docker Compose middleware targets
#
# GISNav needs both micro-ros-agent and mavros for the mock GPS node in PX4
# configuration, while MAVROS alone is sufficient in ArduPilot configuration,
# which complicates things just enough to require separate templates.
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
	$$(call run_middleware, $(2), create --no-recreate)
endef

# The run_middleware function is a helper function for executing middleware
# targets with the necessary Docker Compose options.
#
# Parameters:
#   1. Additional Docker Compose options, such as file overrides.
#   2. The Docker Compose command to execute (either 'up -d' or 'build').
define run_middleware
	@if [ "$*" = "px4" ]; then \
		docker compose -p $(PROJECT_NAME) $(1) $(2) micro-ros-agent mavros; \
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

# onboard HIL services: GIS server, ROS middleware, autoheal, and GISNav
$(eval $(call compose_template,onboard-hil,onboard-hil-middleware,docker-compose.arm64.yaml,mapserver torch-serve autoheal gisnav))

# onboard SITL services: Same as with HIL but middleware is same as offboard (UDP, not serial)
$(eval $(call compose_template,onboard-sitl,offboard-sitl-middleware,docker-compose.arm64.yaml,mapserver torch-serve gisnav))

# offboard SITL services: Gazebo simulation, QGC
$(eval $(call compose_template,offboard-sitl,dummy-dependency,,$$* qgc))

# SITL testing services: Gazebo simulation, ROS middleware, mapserver, torch-serve, but excluding GISNav and QGC
$(eval $(call compose_template,offboard-sitl-test,offboard-sitl-middleware,docker-compose.headless.yaml,$$* torch-serve mapserver))

# SITL development services: Gazebo simulation, ROS middleware, mapserver, torch-serve, QGC, rviz, but excluding GISNav
$(eval $(call compose_template,offboard-sitl-dev,offboard-sitl-middleware,,$$* qgc torch-serve mapserver rviz))

# All SITL services offboard: GIS server, ROS middleware, Gazebo simulation, QGC, gisnav
$(eval $(call compose_template,demo,offboard-sitl-dev,,gisnav))

# List of Docker Compose service names that need GUI access
GUI_SERVICES = px4 ardupilot qgc rviz torch-serve

# Expose xhost to containers that need GUI (see x11 extension in docker-compose.yaml)
expose-xhost:
	@for containerId in `docker ps -f name=$(PROJECT_NAME) -aq`; do \
		serviceName=`docker inspect --format '{{index .Config.Labels "com.docker.compose.service" }}' $$containerId`; \
		if [ ! -z $$serviceName ] && echo $(GUI_SERVICES) | grep -w $$serviceName > /dev/null; then \
			xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $$containerId`; \
		fi; \
	done

# shortcut for demo
.PHONY: demo
demo: up-offboard-sitl-dev-px4
	docker compose -p $(PROJECT_NAME) up gisnav

# shutdown any and all services
.PHONY: down
down:
	@docker compose -p $(PROJECT_NAME) down

# build all services
.PHONY: build
build:
	@docker compose -p $(PROJECT_NAME) build
