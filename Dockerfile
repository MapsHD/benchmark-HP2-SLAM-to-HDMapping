FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ── Base tools and libraries ─────────────────────────────────────────────────
# Eigen, SuiteSparse (BLAS/LAPACK for CHOLMOD) and TBB as in HP2-SLAM's README;
# the X/GL libraries are for open3d, opencv and the Open3D viewer (Mesa DRI is
# the software-GL fallback).
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    patch \
    ca-certificates \
    libeigen3-dev \
    libsuitesparse-dev \
    libtbb-dev \
    libgl1 \
    libegl1 \
    libgomp1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libx11-6 \
    libxi6 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libgl1-mesa-dri \
    libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# ── Python build tools and runtime libraries ─────────────────────────────────
# cmake >= 3.28: the vendored build fetches g2o and SuiteSparse with
# FetchContent SYSTEM/EXCLUDE_FROM_ALL (HP2-SLAM's README asks for >= 3.27);
# < 4 because CMake 4 refuses the old cmake_minimum_required of some fetched
# dependencies. scikit-build-core < 1: HP2-SLAM's pyproject files use the
# cmake.minimum-version key, which 1.x rejects.
# rosbags: KISS-ICP's rosbag dataloader. laspy + lazrs: the LAZ writer of the
# HDMapping converter. matplotlib: the pipeline's closure plot.
RUN pip3 install --no-cache-dir \
    "cmake>=3.28,<4" ninja "scikit-build-core<1" pybind11 \
    "rosbags==0.9.22" laspy lazrs matplotlib

WORKDIR /hp2_ws
COPY ./src/hp2slam ./hp2slam

# ── Benchmark patch: debugging leftovers ─────────────────────────────────────
# See overlay/patches/hp2slam_no_debug_leftovers.patch: the per-iteration
# terminal clear and the "dump a PLY and wait for ENTER" block in the ICP loop
# are switched off (kDebugOutput = false), and the viewer's screenshot of every
# frame is removed; the estimate is unchanged. The build fails if the patch no
# longer applies or a leftover is still active.
COPY ./overlay/patches/ ./patches/
RUN patch -p1 --batch -d hp2slam < patches/hp2slam_no_debug_leftovers.patch && \
    R=hp2slam/hybrid-icp/cpp/hybrid_icp/core/Registration.cpp && \
    grep -q "constexpr bool kDebugOutput = false;" $R && \
    grep -q "if (kDebugOutput) VisualizeStatus(corr" $R && \
    grep -q "if (kDebugOutput && last_alpha" $R && \
    test "$(grep -c "VisualizeStatus(corr\|DumpHybridCorrespondencesPLY(kDumpDir" $R)" = 2 && \
    ! grep -q "capture_screen_image" hp2slam/hp2_slam/tools/visualizer.py

# ── Vendored hybrid-icp still imports the kiss_icp package in places ─────────
# (datasets/rosbag.py, mcap.py, generic.py, kitti*.py, tools/cmd.py). The
# benchmark uses the rosbag dataloader, so these are redirected to the vendored
# hybrid_icp package, which holds the same modules.
RUN cd hp2slam/hybrid-icp/python/hybrid_icp && \
    sed -i 's/from kiss_icp\.pybind import kiss_icp_pybind/from hybrid_icp.pybind import hybrid_icp_pybind/; \
            s/kiss_icp_pybind\./hybrid_icp_pybind./g; \
            s/from kiss_icp\./from hybrid_icp./; \
            s/^\(\s*\)import kiss_icp$/\1import hybrid_icp as kiss_icp/' \
        datasets/*.py tools/*.py && \
    ! grep -rn "from kiss_icp\|import kiss_icp$" --include=*.py . && \
    test -f tools/point_cloud2.py

# ── hybrid-icp (KISS-ICP core with the hybrid registration) ──────────────────
RUN pip3 install --no-cache-dir --no-build-isolation ./hp2slam/hybrid-icp/python

# ── HP2-SLAM (local maps, MapClosures loop closing, g2o pose graph) ──────────
# g2o and SuiteSparse are fetched and built by the vendored CMake (the apt g2o
# lacks the targets it needs); Eigen comes from apt. The define is passed
# through scikit-build-core's environment variable: Ubuntu 22.04's pip has no
# --config-settings.
RUN SKBUILD_CMAKE_DEFINE="USE_SYSTEM_G2O=OFF" \
    pip3 install --no-cache-dir --no-build-isolation ./hp2slam

# libusb: the open3d wheel links libusb-1.0.so.0 (its RealSense support) and
# fails to import without it. Kept as a late layer so the heavy g2o and
# SuiteSparse layers above stay cached.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY ./overlay/scripts/ ./scripts/

# The final guards fail the image build loudly: first every shared library the
# CPU module of open3d needs (all missing ones listed at once; the optional
# CUDA build and PyTorch/TensorFlow ops are not loaded and not checked), then
# the imports.
RUN O3D=$(python3 -c "import importlib.util, os; print(os.path.dirname(importlib.util.find_spec('open3d').origin))") && \
    missing=$(find "$O3D/cpu" -maxdepth 1 -name "pybind*.so" -exec ldd {} \; 2>/dev/null | grep "not found" | sort -u) && \
    if [ -n "$missing" ]; then echo "[build] open3d needs missing libraries:"; echo "$missing"; exit 1; fi && \
    python3 -c "import hybrid_icp, hp2_slam, map_closures, open3d, cv2, laspy, lazrs, rosbags, matplotlib; \
from hybrid_icp.datasets import dataset_factory; from hp2_slam.pipeline import SlamPipeline; \
from hybrid_icp.datasets.rosbag import RosbagDataset; print('[build] HP2-SLAM, dataloaders and converter deps OK')" && \
    test -f /hp2_ws/scripts/hp2_slam_bench.py && test -f /hp2_ws/scripts/hp2slam_to_hdmapping.py

CMD ["bash"]
