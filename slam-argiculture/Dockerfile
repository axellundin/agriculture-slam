FROM ros:rolling-ros-base

RUN apt-get update && apt-get install -y curl python3-colcon-common-extensions

RUN echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc

COPY ./src/data_files.txt /data_files.txt

COPY start.sh /start.sh
RUN chmod +x /start.sh

# Set the start script as the entry point
ENTRYPOINT ["./start.sh"]    