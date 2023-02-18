SHELL := /bin/bash

# See gisnav.org/pages/developer_guide/launch/launch_with_make.html
# docker compose section start
AUTOPILOTS := px4 ardupilot  # Supported autopilots, must match docker/docker-compose.yaml service name

# companion hil services: GIS server, ROS middleware, and GISNav
# The arm64 override for Jetson Nano is used for the mapserver service.
. PHONY: $(addprefix up-companion-hil-, $(AUTOPILOTS))
up-companion-hil-%: up-companion-hil-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml up -d mapserver
	@docker compose up -d gisnav

. PHONY: $(addprefix build-companion-hil-, $(AUTOPILOTS))
build-companion-hil-%: build-companion-hil-middleware-%
	@docker compose -f docker-compose.yaml -f docker-compose.arm64.yaml build mapserver
	@docker compose build gisnav

# companion sitl services: Same as with hil (with sitl the autopilot is run offboard but companion is same)
. PHONY: $(addprefix up-companion-sitl-, $(AUTOPILOTS))
up-companion-sitl-%: up-companion-hil-%

. PHONY: $(addprefix build-companion-sitl-, $(AUTOPILOTS))
build-companion-sitl-%: build-companion-hil-%

# sitl testing services: Gazebo simulation, ROS middleware, mapserver, but excluding GISNav and QGC
# Used for e.g. automated testing where local GISNav testing/development version is used together
# with something like MAVSDK so that a ground control software (QGC) is not needed
. PHONY: $(addprefix up-offboard-sitl-test-, $(AUTOPILOTS))
up-offboard-sitl-test-%: up-offboard-sitl-middleware-%
	@docker compose up -d $* mapserver

. PHONY: $(addprefix build-offboard-sitl-test-, $(AUTOPILOTS))
build-offboard-sitl-test-%: build-offboard-sitl-middleware-%
	@docker compose build $* qgc mapserver

# sitl development services: Gazebo simulation, ROS middleware, mapserver, QGC, and excluding GISNav
. PHONY: $(addprefix up-offboard-sitl-dev-, $(AUTOPILOTS))
up-offboard-sitl-dev-%: up-offboard-sitl-test-%
	@docker compose up -d $* qgc

. PHONY: $(addprefix build-offboard-sitl-dev-, $(AUTOPILOTS))
build-offboard-sitl-dev-%: build-offboard-sitl-test-%
	@docker compose build $* qgc

# offboard sitl services: GIS server, Gazebo simulation, QGC
. PHONY: $(addprefix up-offboard-sitl-, $(AUTOPILOTS))
up-offboard-sitl-%: up-offboard-sitl-dev-%
	@docker compose up -d $*

. PHONY: $(addprefix build-offboard-sitl-, $(AUTOPILOTS))
build-offboard-sitl-%: build-offboard-sitl-dev-%
	@docker compose build $*

# offboard sitl middleware: micro-ros-agent over UDP if PX4, mavros if Ardupilot
. PHONY: $(addprefix up-offboard-sitl-middleware-, $(AUTOPILOTS))
up-offboard-sitl-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose up -d micro-ros-agent; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose up -d mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

. PHONY: $(addprefix build-offboard-sitl-middleware-, $(AUTOPILOTS))
build-offboard-sitl-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose build micro-ros-agent; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose build mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

# onboard hil middleware: micro-ros-agent over serial port if PX4, mavros if Ardupilot
. PHONY: $(addprefix up-companion-hil-middleware-, $(AUTOPILOTS))
up-companion-hil-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml up -d micro-ros-agent; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml up -d mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

. PHONY: $(addprefix build-companion-hil-middleware-, $(AUTOPILOTS))
build-companion-hil-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose -f docker-compose.yaml -f docker-compose.serial.yaml build micro-ros-agent; \
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
	@docker compose up -d gisnav

. PHONY: $(addprefix build-demo-, $(AUTOPILOTS))
build-demo-%: build-offboard-sitl-dev-%
	@docker compose build gisnav

# shutdown any and all services
. PHONY: down
down:
	@docker compose down

# build all services
. PHONY: build
build:
	@docker compose build
# docker compose section end
