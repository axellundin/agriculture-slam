FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y ros-noetic-desktop-full

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR /ros

COPY start.sh ./start.sh
RUN chmod +x ./start.sh

# Set the start script as the entry point
ENTRYPOINT ["./start.sh"]    