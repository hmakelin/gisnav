SHELL := /bin/bash

# See gisnav.org/pages/developer_guide/launch/launch_with_make.html
# docker compose section start
AUTOPILOTS := px4 ardupilot  # Supported autopilots, must match docker/docker-compose.yaml service name

# companion hil services: GIS server, ROS middleware, and GISNav
# The arm64 override for Jetson Nano is used for the mapserver service.
. PHONY: $(addprefix up-companion-hil-, $(AUTOPILOTS))
up-companion-hil-%: up-companion-hil-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml up -d mapserver torch-serve
	@docker compose up -d gisnav

. PHONY: $(addprefix build-companion-hil-, $(AUTOPILOTS))
build-companion-hil-%: build-companion-hil-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml build mapserver torch-serve
	@docker compose build gisnav

# companion sitl services: Same as with hil but middleware is same as offboard (UDP, not serial)
. PHONY: $(addprefix up-companion-sitl-, $(AUTOPILOTS))
up-companion-sitl-%: up-offboard-sitl-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml up -d mapserver torch-serve
	@docker compose up -d gisnav

. PHONY: $(addprefix build-companion-sitl-, $(AUTOPILOTS))
build-companion-sitl-%: build-offboard-sitl-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml build mapserver torch-serve
	@docker compose build gisnav

# offboard sitl services: Gazebo simulation, QGC
. PHONY: $(addprefix up-offboard-sitl-, $(AUTOPILOTS))
up-offboard-sitl-%: up-offboard-sitl-dev-%
	@docker compose up -d $* qgc

. PHONY: $(addprefix build-offboard-sitl-, $(AUTOPILOTS))
build-offboard-sitl-%: build-offboard-sitl-dev-%
	@docker compose build $* qgc

# sitl testing services: Gazebo simulation, ROS middleware, mapserver, torch-serve, but excluding GISNav and QGC
# Used for e.g. automated testing where local GISNav testing/development version is used together
# with something like MAVSDK so that a ground control software (QGC) is not needed
# Gazebo runs in HEADLESS mode
. PHONY: $(addprefix up-offboard-sitl-test-, $(AUTOPILOTS))
up-offboard-sitl-test-%: up-offboard-sitl-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.headless.yaml up -d $* torch-serve mapserver

. PHONY: $(addprefix build-offboard-sitl-test-, $(AUTOPILOTS))
build-offboard-sitl-test-%: build-offboard-sitl-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.headless.yaml build $* torch-serve mapserver

# sitl development services: Gazebo simulation, ROS middleware, mapserver, torch-serve, QGC, but excluding GISNav
. PHONY: $(addprefix up-offboard-sitl-dev-, $(AUTOPILOTS))
up-offboard-sitl-dev-%: up-offboard-sitl-middleware-%
	@docker compose up -d $* torch-serve mapserver qgc

. PHONY: $(addprefix build-offboard-sitl-dev-, $(AUTOPILOTS))
build-offboard-sitl-dev-%: build-offboard-sitl-middleware-%
	@docker compose build $* torch-serve mapserver qgc

# offboard sitl middleware: micro-ros-agent over UDP if PX4, mavros if Ardupilot
. PHONY: $(addprefix up-offboard-sitl-middleware-, $(AUTOPILOTS))
up-offboard-sitl-middleware-%:
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
# onboard hil middleware: micro-ros-agent over serial port if PX4, mavros if Ardupilot
. PHONY: $(addprefix up-companion-hil-middleware-, $(AUTOPILOTS))
up-companion-hil-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml up -d micro-ros-agent mavros; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml up -d mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

# TODO: not tested
. PHONY: $(addprefix build-companion-hil-middleware-, $(AUTOPILOTS))
build-companion-hil-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml build micro-ros-agent mavros; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml build mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

# shortcut for up-demo-px4
. PHONY: demo
demo:
	@$(MAKE) -C . up-demo-px4

# All sitl services offboard: GIS server, ROS middleware, Gazebo simulation, QGC, gisnav
# With px4 for autopilot this is the same as the "mock GPS demo". Not realistic simulation since
# GISNav, middleware and GIS server is run on more powerful offboard workstation computer
. PHONY: $(addprefix up-demo-, $(AUTOPILOTS))
up-demo-%: up-offboard-sitl-dev-%
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
