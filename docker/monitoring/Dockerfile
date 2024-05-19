FROM ubuntu:jammy

# Update the package list and install required packages
RUN apt update \
    && apt -y install python3-pip python3-pynvml \
    && pip install glances[all] \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean

# Expose the port on which Glances will run
EXPOSE 61208
