FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

# Install core tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy the whole repo into the container
COPY . /workspace

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Auto-source ROS + workspace every time bash opens
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /workspace/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
