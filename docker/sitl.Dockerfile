# Development-only PX4 runtime. No ROS, camera drivers or physical devices.
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3 python3-pip libstdc++6 libatomic1 bc \
    && python3 -m pip install --no-cache-dir pymavlink==2.4.50 \
    && rm -rf /var/lib/apt/lists/*
ENTRYPOINT ["python3"]
