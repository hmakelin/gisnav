SHELL := /bin/bash

# docker compose
# Onboard services
up-%: up-middleware-%
	@docker compose up -d mapserver gisnav

build-%:
	@docker compose build mapserver gisnav

# SITL simulation (e.g. on host machine, intended to be paired with up-% on companion computer)
up-sitl-%: up-middleware-%
	@docker compose up -d $* qgc

build-sitl-%: build-middleware-%
	@docker compose build $* qgc

# For local development & testing, everything but GISNav itself (SITL simulation + mapserver)
up-dev-%: up-sitl-%
	@docker compose up -d mapserver

build-dev-%: build-sitl-%
	@docker compose build mapserver

up-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose up -d micro-ros-agent; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose up -d mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

build-middleware-%:
	@if [ "$*" = "px4" ]; then \
		docker compose build micro-ros-agent; \
	elif [ "$*" = "ardupilot" ]; then \
		docker compose build mavros; \
	else \
		echo "Unsupported target '$*' (try 'px4' or 'ardupilot')."; \
	fi

down:
	@docker compose down

build:
	@docker compose build

prune:
	@docker compose system prune -f
# docker compose end

# test
test-sitl:
	@python test/sitl/sitl_test_mock_gps_node.py

test-launch:
	@launch_test src/gisnav/test/launch/test_px4_launch.py
	@launch_test src/gisnav/test/launch/test_ardupilot_launch.py
# test end

.PHONY: docs
docs:
	@$(MAKE) -C docs html
