# syntax=docker/dockerfile:1.7
# Reproducible ROS Humble recording, diagnostics and processing environment.
# Builds natively on Jetson Orin (arm64) and on x86-64. The image carries only
# dependencies; deploy/run-ros.sh mounts this checkout and runs its code.
#
#   docker buildx build -f docker/Dockerfile -t wallering-mapping:humble .
#   docker buildx build -f docker/Dockerfile --target pycolmap-wheel \
#     --output type=local,dest=runs/pycolmap-wheel .
#
# Pins: Ubuntu base digest, dated ROS snapshot (signed, key vendored here), Ceres
# release checksum, COLMAP commit (its fetched PoseLib/faiss archives are SHA256-pinned upstream) and every
# Python package. Ubuntu archive packages follow jammy-updates; the resulting
# versions are written to /opt/wallering/manifest/ in the image.

ARG UBUNTU_BASE=ubuntu:22.04@sha256:5ec03bb3441e8b0bf3b4f9cd4629a1ae763010dc3035bb8da3ae6cf026486401

# ---------------------------------------------------------------------------
# PyCOLMAP 3.12.6 from source: PyPI has no Linux ARM64 wheel for this release.
# CPU-only; supports model IO and quality assessment, not the COLMAP CLI.
FROM ${UBUNTU_BASE} AS pycolmap-build
ARG CERES_VERSION=2.2.0
ARG COLMAP_VERSION=3.12.6
ARG COLMAP_COMMIT=4d5b60e19ad268072adaf1267d21fa38a9a828ca
# Orin Nano (8 GB) runs out of memory above two compile jobs.
ARG BUILD_JOBS=2
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential ca-certificates git ninja-build patchelf python3-dev python3-venv \
      libboost-graph-dev libboost-program-options-dev libboost-system-dev \
      libeigen3-dev libfreeimage-dev libgflags-dev libgoogle-glog-dev \
      libglew-dev libgl-dev libmetis-dev libopenblas-dev libsqlite3-dev libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /build
RUN git init -q colmap \
    && git -C colmap fetch -q --depth 1 https://github.com/colmap/colmap.git "$COLMAP_COMMIT" \
    && git -C colmap checkout -q FETCH_HEAD \
    && test "$(git -C colmap rev-parse HEAD)" = "$COLMAP_COMMIT"
RUN python3 -m venv /build/venv \
    && /build/venv/bin/python -m pip install --no-cache-dir 'pip==26.2.1' \
    && /build/venv/bin/python -m pip install --no-cache-dir \
      'scikit-build-core==1.1.1' 'pybind11==3.0.1' 'cmake==3.31.10' 'auditwheel==6.8.2' 'numpy==1.26.4'
# PyCOLMAP 3.12.6 binds Ceres >= 2.1 enums; jammy ships 2.0, so build a pinned CPU Ceres.
ADD --checksum=sha256:48b2302a7986ece172898477c3bcd6deb8fb5cf19b3327bc49969aad4cede82d \
    http://ceres-solver.org/ceres-solver-${CERES_VERSION}.tar.gz /build/ceres.tar.gz
RUN tar xzf ceres.tar.gz \
    && /build/venv/bin/cmake -S ceres-solver-"$CERES_VERSION" -B ceres-native -GNinja \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/build/install -DBUILD_SHARED_LIBS=ON \
      -DUSE_CUDA=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
      -DBUILD_DOCUMENTATION=OFF \
    && /build/venv/bin/cmake --build ceres-native -j"$BUILD_JOBS" \
    && /build/venv/bin/cmake --install ceres-native
RUN /build/venv/bin/cmake -S colmap -B native -GNinja \
      -DCeres_DIR=/build/install/lib/cmake/Ceres -DCMAKE_BUILD_TYPE=Release \
      '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG' -DCMAKE_INSTALL_PREFIX=/build/install \
      -DCUDA_ENABLED=OFF -DGUI_ENABLED=OFF -DOPENGL_ENABLED=OFF -DCGAL_ENABLED=OFF \
      -DLSD_ENABLED=OFF -DDOWNLOAD_ENABLED=OFF -DTESTS_ENABLED=OFF -DIPO_ENABLED=OFF \
      -DCCACHE_ENABLED=OFF \
    && /build/venv/bin/cmake --build native -j"$BUILD_JOBS" \
    && /build/venv/bin/cmake --install native
# auditwheel needs patchelf >= 0.14.5; jammy ships 0.14.3.
RUN /build/venv/bin/python -m pip install --no-cache-dir 'patchelf==0.17.2.1'
RUN CMAKE_PREFIX_PATH=/build/install CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" \
      /build/venv/bin/python -m pip wheel --no-cache-dir --no-build-isolation --no-deps \
      -Ccmake.define.Ceres_DIR=/build/install/lib/cmake/Ceres \
      -Ccmake.define.GENERATE_STUBS=OFF \
      -Ccmake.define.CMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
      '-Ccmake.define.CMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' \
      -Cbuild-dir=/build/python-native ./colmap -w wheel \
    && PATH=/build/venv/bin:$PATH LD_LIBRARY_PATH=/build/install/lib /build/venv/bin/auditwheel repair wheel/pycolmap-"$COLMAP_VERSION"-*.whl -w /out

# Export only the repaired wheel, e.g. for a host venv on the workstation.
FROM scratch AS pycolmap-wheel
COPY --from=pycolmap-build /out/ /

# ---------------------------------------------------------------------------
# ROS Humble from a dated, signed snapshot so a rebuild resolves the same
# packages: official depthai_ros_driver, MAVROS and rosbag2 with MCAP storage.
FROM ${UBUNTU_BASE} AS base
ARG ROS_SNAPSHOT=2026-08-07
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY docker/ros-snapshots.asc /usr/share/keyrings/ros-snapshots.asc
# The snapshot host serves HTTP only; apt verifies the signed InRelease.
RUN echo "deb [signed-by=/usr/share/keyrings/ros-snapshots.asc] http://snapshots.ros.org/humble/${ROS_SNAPSHOT}/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros-snapshot.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-depthai-ros-driver \
      ros-humble-rosbag2-storage-mcap \
      ros-humble-mavros ros-humble-mavros-extras \
      bzip2 ca-certificates git python3-pip python3-venv usbutils procps \
    && rm -rf /var/lib/apt/lists/*
# MAVROS refuses to start without its geoid/gravity/magnetic models.
RUN geographiclib-get-geoids egm96-5 \
    && geographiclib-get-gravity egm96 \
    && geographiclib-get-magnetic emm2015
# The checkout is bind-mounted and usually owned by a different UID.
RUN git config --system --add safe.directory '*'

# Python dependencies only; the package itself comes from the mounted checkout
# via PYTHONPATH, so code changes do not require an image rebuild.
COPY pyproject.toml /tmp/wr-deps/pyproject.toml
RUN python3 -m venv --system-site-packages /opt/wr-venv \
    && /opt/wr-venv/bin/python -m pip install --no-cache-dir 'pip==26.2.1' \
    && mkdir -p /tmp/wr-deps/src/wallering_mapping \
    && touch /tmp/wr-deps/src/wallering_mapping/__init__.py \
    && /opt/wr-venv/bin/python -m pip install --no-cache-dir \
      '/tmp/wr-deps[bags,oak,terrain,diagnostics,dev]' \
    && /opt/wr-venv/bin/python -m pip uninstall -y -q wallering-mapping \
    && rm -rf /tmp/wr-deps
ENV PATH=/opt/wr-venv/bin:$PATH \
    WR_MAPPING_PYTHON=/opt/wr-venv/bin/python \
    PYTHONNOUSERSITE=1 \
    HOME=/tmp

# ---------------------------------------------------------------------------
FROM base AS runtime
COPY --from=pycolmap-build /out/ /tmp/pycolmap/
RUN /opt/wr-venv/bin/python -m pip install --no-cache-dir --no-deps /tmp/pycolmap/*.whl \
    && rm -rf /tmp/pycolmap \
    && /opt/wr-venv/bin/python -c 'import pycolmap; assert pycolmap.__version__ == "3.12.6"; print(pycolmap.__ceres_version__)'
RUN mkdir -p /opt/wallering/manifest \
    && dpkg-query -W -f='${Package} ${Version} ${Architecture}\n' > /opt/wallering/manifest/dpkg.txt \
    && /opt/wr-venv/bin/python -m pip freeze --all > /opt/wallering/manifest/pip.txt \
    && find /usr/share/GeographicLib -type f -exec sha256sum {} + | sort -k2 \
      > /opt/wallering/manifest/geographiclib.sha256
LABEL org.opencontainers.image.title="wallering-mapping" \
      org.opencontainers.image.source="https://github.com/WalleringRobotics/mapping_functionality"
WORKDIR /tmp
CMD ["/bin/bash"]
