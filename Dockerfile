FROM dynorobotics/ros2:master

# Terminal utils, etc...
RUN apt-get update && apt-get install -y \
  curl \
  wget \
  vim \
  less \
  python-pip \
  ranger \
  tmux \
  python3-matplotlib \
  libyaml-dev \
  libxaw7-dev \
  clang-format \
  net-tools \
  iputils-ping \
  htop \
  && rm -rf /var/likb/apt/lists/*

# ROS 2 - tools
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget \
  && python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-watch \
  pytest-runner \
  setuptools \
  && rm -rf /var/likb/apt/lists/*

# copy ros package repo
ENV MAIN_WS /opt/turtlebot3_ws
RUN mkdir -p $MAIN_WS/src
WORKDIR $MAIN_WS/src
COPY ./ turtlebot3_unity/

# clone dependency ros package repos
ENV DEPENDENCIES_WS /opt/turtlebot3_dependencies_ws
RUN mkdir -p $DEPENDENCIES_WS/src
WORKDIR $DEPENDENCIES_WS
RUN vcs import src < $MAIN_WS/src/turtlebot3_unity/ros2_dependencies.repos
# RUN vcs import src < $DEPENDENCIES_WS/src/turtlebot3/turtlebot3.repos
# RUN vcs import src < $DEPENDENCIES_WS/src/navigation2/tools/ros2_dependencies.repos

# install dependency ros package dependencies
WORKDIR $DEPENDENCIES_WS
RUN . $ROS2_WS/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build dependency ros package source
ARG CMAKE_BUILD_TYPE=Release
WORKDIR $DEPENDENCIES_WS
RUN . $ROS2_WS/install/setup.sh && \
     colcon build \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# install ros package dependencies
WORKDIR $MAIN_WS
RUN . $DEPENDENCIES_WS/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && pip3 install -r src/turtlebot3_unity/requirements.txt \
    && rm -rf /var/lib/apt/lists/*

# build package source
ARG CMAKE_BUILD_TYPE=Release
WORKDIR $MAIN_WS
RUN . $DEPENDENCIES_WS/install/setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# source workspace from entrypoint if available
COPY ros_entrypoint.sh /
