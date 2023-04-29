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
#  		Nvidia Jetson Nano).
#   - Middleware: Software that sits between the autopilot and GISNav, enabling
#		communication between them.
#   - QGC: QGroundControl, a ground control station software used to monitor
#		and control drones.
#   - Gazebo: A robotics simulator used to simulate the drone's environment and
#		its interactions with the world.
#
# The target options in the Makefile are as follows:
#	- onboard-hil-*: Targets for onboard hardware-in-the-loop services,
#		including GIS server, ROS middleware, and GISNav.
#   - onboard-sitl-*: Targets for onboard software-in-the-loop services, with
#		the same services as onboard HIL but using a different middleware
#		configuration.
#   - offboard-sitl-*: Targets for offboard software-in-the-loop services,
#		including Gazebo simulation and QGroundControl.
#   - offboard-sitl-test-*: Targets for SITL testing services, which exclude
#		GISNav and QGC for automated testing scenarios.
#   - offboard-sitl-dev-*: Targets for SITL development services, including
#		Gazebo simulation, ROS middleware, mapserver, torch-serve, and QGC, but
#		excluding GISNav whose development version is assumed to be run locally.
#   - demo-*: Shortcut targets for demo purposes, setting up all SITL services
#		offboard, including GIS server, ROS middleware, Gazebo simulation, QGC,
#		and GISNav.

SHELL := /bin/bash

# Supported autopilots, must match docker/docker-compose.yaml service name
AUTOPILOTS := px4 ardupilot

# Define a reusable template for creating Docker Compose targets
# Parameters:
#   1. The prefix for the target name.
#   2. The dependency target.
#   3. The optional Docker Compose override files to include (if any).
#   4. The Docker Compose services to start or build - use $$* here for
#		autopilot, the first '$' is to escape the second '$'.
define compose_template
# Create phony targets with the specified prefix for each autopilot
.PHONY: $(addprefix $(1)-, $(AUTOPILOTS))

# Generate rules for the prefixed targets, depending on the specified dependency target
$(addprefix $(1)-, $(AUTOPILOTS)): $(if $(2),$(1)-%: $(2)-%,$(1)-%)
	# Evaluate the additional Docker Compose files
	$$(eval compose_files := $(if $(3),-f docker-compose.yaml $(foreach file,$(3),-f $(file))))
	# Run Docker Compose with the specified services
	@docker compose $$(compose_files) up -d $(4)

# Create phony build targets with the specified prefix for each autopilot
.PHONY: $(addprefix build-$(1)-, $(AUTOPILOTS))

# Generate rules for the build prefixed targets, depending on the specified build dependency target
$(addprefix build-$(1)-, $(AUTOPILOTS)): $(if $(2),build-$(1)-%: build-$(2)-%,build-$(1)-%)
	# Evaluate the additional Docker Compose files
	$$(eval compose_files := $(if $(3),-f docker-compose.yaml $(foreach file,$(3),-f $(file))))
	# Build the specified Docker Compose services
	@docker compose $$(compose_files) build $(4)
endef

# Define targets below

# onboard HIL middleware targets
# TODO: not tested
. PHONY: $(addprefix onboard-hil-middleware-, $(AUTOPILOTS))
onboard-hil-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml up -d micro-ros-agent mavros; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml up -d mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

# TODO: not tested
. PHONY: $(addprefix build-onboard-hil-middleware-, $(AUTOPILOTS))
build-onboard-hil-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml build micro-ros-agent mavros; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml build mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

# onboard SITL middleware targets
onboard-sitl-middleware-%: offboard-sitl-middleware-%

build-onboard-sitl-middleware-%: build-offboard-sitl-middleware-%

# offboard SITL middleware targets
. PHONY: $(addprefix offboard-sitl-middleware-, $(AUTOPILOTS))
offboard-sitl-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose up -d micro-ros-agent mavros; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.mavros-sitl-ardupilot.yaml up -d mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

. PHONY: $(addprefix build-offboard-sitl-middleware-, $(AUTOPILOTS))
build-offboard-sitl-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose build micro-ros-agent mavros; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.mavros-sitl-ardupilot.yaml build mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

# onboard HIL services: GIS server, ROS middleware, and GISNav
$(eval $(call compose_template,onboard-hil,onboard-hil-middleware,docker-compose.arm64.yaml,mapserver torch-serve gisnav))

# onboard SITL services: Same as with HIL but middleware is same as offboard (UDP, not serial)
$(eval $(call compose_template,onboard-sitl,offboard-sitl-middleware,docker-compose.arm64.yaml,mapserver torch-serve gisnav))

# offboard SITL services: Gazebo simulation, QGC
$(eval $(call compose_template,offboard-sitl,offboard-sitl-dev,,qgc))

# SITL testing services: Gazebo simulation, ROS middleware, mapserver, torch-serve, but excluding GISNav and QGC
$(eval $(call compose_template,offboard-sitl-test,offboard-sitl-middleware,docker-compose.headless.yaml,$$* torch-serve mapserver))

# SITL development services: Gazebo simulation, ROS middleware, mapserver, torch-serve, QGC, but excluding GISNav
$(eval $(call compose_template,offboard-sitl-dev,offboard-sitl-middleware,,$$* qgc torch-serve mapserver))

# All SITL services offboard: GIS server, ROS middleware, Gazebo simulation, QGC, gisnav
$(eval $(call compose_template,demo,offboard-sitl-dev,,gisnav))

# shortcut for demo-px4
.PHONY: demo
demo:
	@$(MAKE) -C . demo-px4

# shutdown any and all services
.PHONY: down
down:
	@docker compose down

# build all services
.PHONY: build
build:
	@docker compose build
