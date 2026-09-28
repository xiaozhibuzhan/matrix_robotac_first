FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl gnupg2 lsb-release ca-certificates wget sudo build-essential \
    python3-pip jq unzip xz-utils xvfb x11-utils \
    libgl1-mesa-glx libxrandr2 libxinerama1 libxcursor1 libxi6 libglu1-mesa \
    libvulkan1 vulkan-tools mesa-vulkan-drivers procps git && \
    locale-gen en_US.UTF-8 && rm -rf /var/lib/apt/lists/*


RUN apt update -y && apt install locales -y && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    locale && \
    apt install software-properties-common -y && \
    add-apt-repository universe -y

RUN apt update && apt install curl -y && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    apt update && \
    apt install ros-humble-desktop -y
# Install ROS 2 Humble
# Create workspace and copy repository
WORKDIR /workspace
COPY . /workspace
# Run project build script to install project-level dependencies (dpkg -i on deps/)
# Run as root during image build so apt/dpkg can execute. Fail the build if the
# script exits non-zero so problems are noticed during image build.
RUN cd /workspace && chmod +x ./build.sh && \
    ./build.sh

# Create a non-root user for development
RUN useradd -m -s /bin/bash developer && echo "developer ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/developer && \
    chown -R developer:developer /workspace && \
    usermod -aG input,dialout,video developer

USER developer
ENV HOME=/home/developer
WORKDIR /home/developer

# Source ROS in user shell and add workspace source hook (if present)
RUN echo "source /opt/ros/humble/setup.bash" >> /home/developer/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/developer/.bashrc

WORKDIR /workspace

# Entrypoint to source environment and forward X/ROS usage
COPY scripts/docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sudo chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["./open_sim_launcher"]
