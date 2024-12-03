# SLAM Agriculture Project

This project aims to implement Simultaneous Localization and Mapping (SLAM) techniques for agricultural applications.

## Project Goals

### 1. Data Processing
- [x] Configure ROS Noetic environment
- [x] Set up Docker containerization
- [ ] Sensor data collection pipeline
  - [ ] LiDAR data processing
  - [ ] GPS data processing
- [ ] Data filtering and preprocessing
  - [ ] Noise reduction ? 
  - [ ] Outlier removal

### 2. SLAM Implementation
- [ ] Create process and measurement models
- [ ] Core SLAM functionality
  - [ ] Point cloud registration
  - [ ] Pose estimation
  - [ ] Loop closure detection
- [ ] Agricultural optimization
  - [ ] Row detection algorithm
  - [ ] Field boundary mapping
  - [ ] Dynamic object handling

### 3. Visualization
- [ ] Real-time mapping display
- [ ] Path trajectory visualization
- [ ] Field coverage monitoring
- [ ] Performance metrics dashboard

## Getting Started

### Prerequisites
- Docker
- ROS Noetic
- (Add other prerequisites)

### Installation
1. Clone the repository
```
git clone https://github.com/axlundin/slam-argiculture.git
```
2. Build the Docker container
```
cd slam-argiculture
docker-compose build
```
3. Run the Docker container
```
docker-compose up
```

### Downloading datasets
The desired dataset files from the page https://www.ipb.uni-bonn.de/datasets_IJRR2017/rosbags/160420/ are listed in the `data_files.txt` file.
