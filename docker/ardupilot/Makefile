SHELL:=/bin/bash

sim_vehicle: gazebo
	sim_vehicle.py -v ArduCopter -f gazebo-iris -L KSQL_Airport -m '--cmd="module load GPSInput"'

gazebo:
	cd ${HOME} && \
	if [ -z ${HEADLESS} ]; then \
		echo "Launching Gazebo client & server..."; \
		gazebo --verbose worlds/ksql_airport.world & \
	else \
		echo "Launching Gazebo in headless mode..."; \
		gzserver --verbose worlds/ksql_airport.world & \
	fi
