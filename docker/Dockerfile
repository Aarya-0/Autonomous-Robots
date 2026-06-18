FROM fbe-dockerreg.rwu.de/doz-iki/staehle-vls/amr-tb3:latest

# Install dependencies
RUN apt-get update && \
    apt-get install -y curl sudo && \
    rm -rf /var/lib/apt/lists/*

# Default user
USER root
WORKDIR /root/ros2_ws

CMD ["bash"]

