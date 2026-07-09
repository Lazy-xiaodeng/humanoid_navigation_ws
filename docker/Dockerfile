# syntax=docker/dockerfile:1

ARG BASE_IMAGE=osrf/ros:jazzy-desktop-full
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=jazzy
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV WORKSPACE=/home/ubuntu/humanoid_ws
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/.config/fastdds_shm.xml
ENV RMW_FASTRTPS_USE_QOS_FROM_XML=1
ENV SKIP_COLCON_BUILD=1
ENV Open3D_DIR=/opt/open3d/lib/cmake/Open3D
ENV LD_LIBRARY_PATH=/opt/open3d/lib

SHELL ["/bin/bash", "-c"]

RUN printf '%s\n' \
      'Acquire::Retries "5";' \
      'Acquire::http::Timeout "30";' \
      'Acquire::https::Timeout "30";' \
      'Acquire::http::Pipeline-Depth "0";' \
      'Acquire::BrokenProxy "true";' \
      'DPkg::Lock::Timeout "60";' \
      > /etc/apt/apt.conf.d/80-humanoid-retries

RUN groupadd -f plugdev \
    && if ! getent group "${USER_GID}" >/dev/null; then groupadd --gid "${USER_GID}" "${USERNAME}"; fi \
    && if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
         useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}"; \
       fi \
    && usermod -aG dialout,video,plugdev "${USERNAME}" || true

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       bash-completion \
       build-essential \
       ca-certificates \
       cmake \
       curl \
       git \
       gnupg \
       iproute2 \
       iputils-ping \
       less \
       libc++-dev \
       libc++1 \
       libc++abi-dev \
       libc++abi1 \
       libgl1 \
       libglib2.0-0 \
       libx11-6 \
       libxext6 \
       libxrender1 \
       locales \
       mesa-utils \
       nano \
       net-tools \
       ocl-icd-opencl-dev \
       opencl-headers \
       python3-colcon-common-extensions \
       python3-pip \
       python3-rosdep \
       python3-serial \
       python3-websockets \
       python3-yaml \
       rsync \
       sudo \
       udev \
       vim \
       wget \
       xz-utils \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true

WORKDIR ${WORKSPACE}
COPY --chown=${USER_UID}:${USER_GID} . ${WORKSPACE}

RUN mkdir -p /home/ubuntu/.config \
    && if [ -f "${WORKSPACE}/deploy/docker_offline/config/fastdds_shm.xml" ]; then \
         cp "${WORKSPACE}/deploy/docker_offline/config/fastdds_shm.xml" /home/ubuntu/.config/fastdds_shm.xml; \
       fi \
    && chown -R "${USER_UID}:${USER_GID}" /home/ubuntu/.config

RUN apt-get update \
    && for attempt in 1 2 3 4 5; do \
         rosdep update --rosdistro "${ROS_DISTRO}" && break; \
         if [ "${attempt}" = "5" ]; then exit 1; fi; \
         sleep 5; \
       done \
    && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}" --skip-keys "websockets" \
    && rm -rf /var/lib/apt/lists/*

RUN for attempt in 1 2 3 4 5; do \
        rm -f /tmp/open3d-devel.tar.xz; \
        curl -L --fail --retry 3 --retry-all-errors \
          --connect-timeout 20 --speed-limit 1024 --speed-time 30 \
          -o /tmp/open3d-devel.tar.xz \
          https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz \
        && break; \
        if [ "${attempt}" = "5" ]; then exit 1; fi; \
        sleep 5; \
      done \
    && test -s /tmp/open3d-devel.tar.xz \
    && mkdir -p /opt/open3d \
    && tar -xJf /tmp/open3d-devel.tar.xz -C /opt/open3d --strip-components=1 \
    && rm -f /tmp/open3d-devel.tar.xz \
    && test -f "${Open3D_DIR}/Open3DConfig.cmake"

RUN for attempt in 1 2 3 4 5; do \
        rm -rf /tmp/Livox-SDK2 /tmp/livox-sdk2.tar.gz; \
        mkdir -p /tmp/Livox-SDK2; \
        curl -L --fail --retry 3 --retry-all-errors \
          --connect-timeout 20 --speed-limit 1024 --speed-time 30 \
          -o /tmp/livox-sdk2.tar.gz \
          https://github.com/Livox-SDK/Livox-SDK2/archive/refs/tags/v1.3.1.tar.gz \
        && tar -xzf /tmp/livox-sdk2.tar.gz -C /tmp/Livox-SDK2 --strip-components=1 \
        && break; \
        if [ "${attempt}" = "5" ]; then exit 1; fi; \
        sleep 5; \
      done \
    && if ! grep -q '#include <cstdint>' /tmp/Livox-SDK2/sdk_core/comm/define.h; then \
         sed -i '/#include <atomic>/a #include <cstdint>' /tmp/Livox-SDK2/sdk_core/comm/define.h; \
       fi \
    && if ! grep -q '#include <cstdint>' /tmp/Livox-SDK2/sdk_core/logger_handler/file_manager.h; then \
         sed -i '/#include <map>/a #include <cstdint>' /tmp/Livox-SDK2/sdk_core/logger_handler/file_manager.h; \
       fi \
    && cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/Livox-SDK2/build --parallel \
    && cmake --install /tmp/Livox-SDK2/build \
    && ldconfig \
    && test -f /usr/local/lib/liblivox_lidar_sdk_shared.so \
    && test -f /usr/local/include/livox_lidar_api.h \
    && rm -rf /tmp/Livox-SDK2 /tmp/livox-sdk2.tar.gz

ARG PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
ARG PIP_TRUSTED_HOST=pypi.tuna.tsinghua.edu.cn
ENV PIP_INDEX_URL=${PIP_INDEX_URL} \
    PIP_TRUSTED_HOST=${PIP_TRUSTED_HOST} \
    PIP_DEFAULT_TIMEOUT=120

RUN for attempt in 1 2 3 4 5; do \
        python3 -m pip install --break-system-packages --no-cache-dir --no-deps \
          --retries 10 --timeout 120 open3d==0.19.0 \
        && break; \
        if [ "${attempt}" = "5" ]; then exit 1; fi; \
        sleep 10; \
      done

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --symlink-install --cmake-args \
       -DCMAKE_BUILD_TYPE=Release \
       -DOpen3D_DIR="${Open3D_DIR}" \
       -DROS_EDITION=ROS2 \
       -DDISTRO_ROS="${ROS_DISTRO}" \
    && chown -R "${USER_UID}:${USER_GID}" "${WORKSPACE}"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-configargparse \
       python3-flask \
       python3-ipywidgets \
       python3-nbformat \
       python3-pandas \
       python3-plotly \
       python3-retrying \
       python3-sklearn \
       python3-tqdm \
       python3-typing-extensions \
       python3-werkzeug \
    && python3 -m pip install --break-system-packages --no-cache-dir --retries 5 --timeout 120 --no-deps \
       addict \
       dash==2.14.2 \
       pyquaternion \
    && rm -rf /var/lib/apt/lists/*

RUN chmod +x \
      ${WORKSPACE}/docker/entrypoint.sh \
      ${WORKSPACE}/start_navigation.sh \
      ${WORKSPACE}/start_mapping.sh \
      ${WORKSPACE}/stop_navigation.sh \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc \
    && echo "source ${WORKSPACE}/install/setup.bash" >> /home/ubuntu/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/ubuntu/.bashrc \
    && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/.config/fastdds_shm.xml" >> /home/ubuntu/.bashrc \
    && echo "export RMW_FASTRTPS_USE_QOS_FROM_XML=1" >> /home/ubuntu/.bashrc \
    && echo "export Open3D_DIR=${Open3D_DIR}" >> /home/ubuntu/.bashrc \
    && echo "export LD_LIBRARY_PATH=/opt/open3d/lib:\${LD_LIBRARY_PATH}" >> /home/ubuntu/.bashrc \
    && chown "${USER_UID}:${USER_GID}" /home/ubuntu/.bashrc

USER ${USERNAME}
WORKDIR ${WORKSPACE}
ENTRYPOINT ["/home/ubuntu/humanoid_ws/docker/entrypoint.sh"]
CMD ["bash"]
