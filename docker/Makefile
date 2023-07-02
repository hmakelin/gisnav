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

define compose_template
# $(1) - The prefix to use for the build, create, and up targets
# $(2) - The dependency target for the build and create rules
# $(3) - Additional Docker Compose files to include in the build, create, and up operations
# $(4) - The services to include in the build, create, and up operations

# Define a macro for generating build and create rules
define build_and_create_rule
# $(1) - The operation type (either "build" or "create")
# $(2) - The prefix to use for the target
# $(3) - The dependency target (if any)
# $(4) - Additional Docker Compose files to include
# $(5) - The services to include in the build or create operations

.PHONY: $(addprefix $(1)-$(2)-, $(AUTOPILOTS))

$(addprefix $(1)-$(2)-, $(AUTOPILOTS)): $(if $(3),$(1)-$(2)-%: $(1)-$(3)-%,$(1)-$(2)-%)
	$$(eval compose_files := $(if $(4),-f docker-compose.yaml $(foreach file,$(4),-f $(file))))
	@if [ "$(1)" = "create" ]; then \
		COMPOSE_COMMAND="create --build"; \
	else \
		COMPOSE_COMMAND="$(1)"; \
	fi; \
	@COMPOSE_PROJECT_NAME=$(PROJECT_NAME) docker compose $$(compose_files) $$COMPOSE_COMMAND $(5)
	# Expose xhost to containers to make GUI applications work
	@if [ "$(1)" = "create" -o "$(1)" = "up" ]; then \
		for containerId in $$(docker ps -f name=$(PROJECT_NAME) -aq); do \
			xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $$containerId`; \
		done \
	fi
endef

# Define a macro for generating up rules
define up_rule
# $(1) - The prefix to use for the target
# $(2) - The dependency target (which is always the corresponding "create" target)
# $(3) - Additional Docker Compose files to include
# $(4) - The services to include in the up operation

.PHONY: $(addprefix up-$(1)-, $(AUTOPILOTS))

$(addprefix up-$(1)-, $(AUTOPILOTS)): create-$(1)-%
	$$(eval compose_files := $(if $(3),-f docker-compose.yaml $(foreach file,$(3),-f $(file))))
	@COMPOSE_PROJECT_NAME=$(PROJECT_NAME) docker compose $$(compose_files) up -d $(4)
endef

# Use the new macros to generate build, create and up rules
$(eval $(call build_and_create_rule, build, $(1), $(2), $(3), $(4)))
$(eval $(call build_and_create_rule, create, $(1), $(2), $(3), $(4)))
$(eval $(call up_rule, $(1), $(2), $(3), $(4)))
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
.PHONY: $(1)-% build-$(1)-%

$(1)-%:
	$$(call run_middleware, $(2), up -d)

build-$(1)-%:
	$$(call run_middleware, $(2), build)

create-$(1)-%:
	$$(call run_middleware, $(2), create --build)
endef

# The run_middleware function is a helper function for executing middleware
# targets with the necessary Docker Compose options.
#
# Parameters:
#   1. Additional Docker Compose options, such as file overrides.
#   2. The Docker Compose command to execute (either 'up -d' or 'build').
define run_middleware
	@if [ "$*" = "px4" ]; then \
		docker compose $(1) $(2) micro-ros-agent mavros; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose $(1) $(2) mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi
endef

# Define middleware targets
$(eval $(call middleware_template,onboard-hil-middleware,-f docker-compose.yaml -f docker-compose.serial.yaml))
$(eval $(call middleware_template,onboard-sitl-middleware,))
$(eval $(call middleware_template,offboard-sitl-middleware,))  # same as onboard

# onboard HIL services: GIS server, ROS middleware, autoheal, and GISNav
$(eval $(call compose_template,onboard-hil,onboard-hil-middleware,docker-compose.arm64.yaml,mapserver torch-serve autoheal gisnav))

# onboard SITL services: Same as with HIL but middleware is same as offboard (UDP, not serial)
$(eval $(call compose_template,onboard-sitl,offboard-sitl-middleware,docker-compose.arm64.yaml,mapserver torch-serve gisnav))

# offboard SITL services: Gazebo simulation, QGC
$(eval $(call compose_template,offboard-sitl,offboard-sitl-dev,,qgc))

# SITL testing services: Gazebo simulation, ROS middleware, mapserver, torch-serve, but excluding GISNav and QGC
$(eval $(call compose_template,offboard-sitl-test,offboard-sitl-middleware,docker-compose.headless.yaml,$$* torch-serve mapserver))

# SITL development services: Gazebo simulation, ROS middleware, mapserver, torch-serve, QGC, rviz, but excluding GISNav
$(eval $(call compose_template,offboard-sitl-dev,offboard-sitl-middleware,,$$* qgc torch-serve mapserver rviz))

# All SITL services offboard: GIS server, ROS middleware, Gazebo simulation, QGC, gisnav
$(eval $(call compose_template,demo,offboard-sitl-dev,,gisnav))

# shortcut for up-demo-px4
.PHONY: demo
demo:
	@$(MAKE) -C . up-demo-px4

# shutdown any and all services
.PHONY: down
down:
	@docker compose down

# build all services
.PHONY: build
build:
	@docker compose build
