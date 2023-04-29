SHELL := /bin/bash

# See gisnav.org/pages/developer_guide/launch/launch_with_make.html
# docker compose section start
AUTOPILOTS := px4 ardupilot  # Supported autopilots, must match docker/docker-compose.yaml service name

# onboard hil services: GIS server, ROS middleware, and GISNav
# The arm64 override for Jetson Nano is used for the mapserver service.
. PHONY: $(addprefix onboard-hil-, $(AUTOPILOTS))
onboard-hil-%: onboard-hil-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml up -d mapserver torch-serve
	@docker compose up -d gisnav

. PHONY: $(addprefix build-onboard-hil-, $(AUTOPILOTS))
build-onboard-hil-%: build-onboard-hil-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml build mapserver torch-serve
	@docker compose build gisnav

# onboard sitl services: Same as with hil but middleware is same as offboard (UDP, not serial)
. PHONY: $(addprefix onboard-sitl-, $(AUTOPILOTS))
onboard-sitl-%: offboard-sitl-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml up -d mapserver torch-serve
	@docker compose up -d gisnav

. PHONY: $(addprefix build-onboard-sitl-, $(AUTOPILOTS))
build-onboard-sitl-%: build-offboard-sitl-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml build mapserver torch-serve
	@docker compose build gisnav

# offboard sitl services: Gazebo simulation, QGC
. PHONY: $(addprefix offboard-sitl-, $(AUTOPILOTS))
offboard-sitl-%: offboard-sitl-dev-%
	@docker compose up -d $* qgc

. PHONY: $(addprefix build-offboard-sitl-, $(AUTOPILOTS))
build-offboard-sitl-%: build-offboard-sitl-dev-%
	@docker compose build $* qgc

# sitl testing services: Gazebo simulation, ROS middleware, mapserver, torch-serve, but excluding GISNav and QGC
# Used for e.g. automated testing where local GISNav testing/development version is used together
# with something like MAVSDK so that a ground control software (QGC) is not needed
# Gazebo runs in HEADLESS mode
. PHONY: $(addprefix offboard-sitl-test-, $(AUTOPILOTS))
offboard-sitl-test-%: offboard-sitl-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.headless.yaml up -d $* torch-serve mapserver

. PHONY: $(addprefix build-offboard-sitl-test-, $(AUTOPILOTS))
build-offboard-sitl-test-%: build-offboard-sitl-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.headless.yaml build $* torch-serve mapserver

# sitl development services: Gazebo simulation, ROS middleware, mapserver, torch-serve, QGC, but excluding GISNav
. PHONY: $(addprefix offboard-sitl-dev-, $(AUTOPILOTS))
offboard-sitl-dev-%: offboard-sitl-middleware-%
	@docker compose up -d $* torch-serve mapserver qgc rviz2

. PHONY: $(addprefix build-offboard-sitl-dev-, $(AUTOPILOTS))
build-offboard-sitl-dev-%: build-offboard-sitl-middleware-%
	@docker compose build $* torch-serve mapserver qgc rviz2

# offboard sitl middleware
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

# TODO: not tested
# onboard hil middleware
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

# shortcut for demo-px4
. PHONY: demo
demo:
	@$(MAKE) -C . demo-px4

# All sitl services offboard: GIS server, ROS middleware, Gazebo simulation, QGC, gisnav
# With px4 for autopilot this is the same as the "mock GPS demo". Not realistic simulation since
# GISNav, middleware and GIS server is run on more powerful offboard workstation computer
. PHONY: $(addprefix demo-, $(AUTOPILOTS))
demo-%: offboard-sitl-dev-%
	if [ "$*" = "px4" ]; then \
		docker compose up -d gisnav; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.gisnav-ardupilot.yaml up -d gisnav; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

. PHONY: $(addprefix build-demo-, $(AUTOPILOTS))
build-demo-%: build-offboard-sitl-dev-%
	if [ "$*" = "px4" ]; then \
		docker compose build gisnav; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.gisnav-ardupilot.yaml build gisnav; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

# shutdown any and all services
. PHONY: down
down:
	@docker compose down

# build all services
. PHONY: build
build:
	@docker compose build
# docker compose section end
