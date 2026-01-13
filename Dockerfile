from osrf/ros:humble-desktop-full
#base image

env debian_frontend=noninteractive
env ros_distro=humble
#prevents apt prompts

arg username=kol
arg user_uid=1000
arg user_gid=1000

#stuff ill need for deving,
run apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    unzip \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    python3-argcomplete \
    python3-numpy \
    python3-pyqt5 \
    python3-lxml \
    tmux \
    ripgrep \
    fd-find \
    x11-apps \
    mesa-utils \
    sudo \
    && rm -rf /var/lib/apt/lists/*

#creating a non-root usr
run groupadd --gid ${ user_gid } ${ username } \
	&& useradd --uid ${ user_uid } --gid ${ user_gid } -m ${ username } \
	$$ echo "${ username } all=(all) nopasswd:all" > /etc/sudoers.d/${ username }
	$$ chmod 0440 /etc/sudoers.d/${ username }

run rosdep init || true
run rosdep update

#ros2 humble stuff
run apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-control-interface \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    && rm -rf /var/lib/apt/lists/*

#latest nvim
run curl -lo https://github.com/neovim/neovim/releases/latest/download/nvim-linux64.tar.gz \
    && tar -c /opt -xzf nvim-linux64.tar.gz \
    && ln -s /opt/nvim-linux64/bin/nvim /usr/local/bin/nvim \
    && rm nvim-linux64.tar.gz

run git clone https://github.com/nvim-lua/kickstart.nvim.git \
    /home/${username}/.config/nvim \
    && chown -r ${username}:${username} /home/${username}/.config


run mkdir -p /home/${username}/.colcon && \
    printf '{\n  "build": {\n    "symlink-install": true,\n    "cmake-args": ["-dcmake_build_type=release"]\n  }\n}\n' \
    > /home/${username}/.colcon/defaults.yaml && \
    chown -r ${username}:${username} /home/${username}/.colcon


run mkdir -p /home/${username}/ws/src \
    && chown -r ${username}:${username} /home/${username}/ws

workdir /home/${username}/ws


run echo "source /opt/ros/humble/setup.bash" >> /home/${username}/.bashrc \
    && echo "source ~/ws/install/setup.bash 2>/dev/null" >> /home/${username}/.bashrc \
    && chown ${username}:${username} /home/${username}/.bashrc


-v $(pwd)/ws:/home/kol/ws
