FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y curl ros-noetic-catkin

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

COPY ./src/data_files.txt /data_files.txt

COPY start.sh /start.sh
RUN chmod +x /start.sh

# Set the start script as the entry point
ENTRYPOINT ["./start.sh"]    