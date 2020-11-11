# ICRA20 视觉SLAM论文调研
## 1.特征点法
### Towards Noise Resilient SLAM
针对ORB-SLAM2框架中depth噪声引起的问题，提出两种应对，一是对每个landmark维护一组3D观测，以其中心作为估计值，以其一致性判断landmark好坏。二是针对depth噪声特点设计自适应的虚拟相机位置来抑制噪声影响。这些方法不仅能提高RGBD/stereo特征点法SLAM的精度，还由于排除了outlier而减少计算量，提高运行速度。

### Voxel Map for Visual SLAM
基于关键帧的SLAM在tracking时一般用共视关键帧中的关键点与当前帧做匹配，这是一种启发式算法，查到的关键点未必符合当前视野，也没考虑遮挡关系。本文提出用voxel hashing来索引特征点，对每个预测视角，用预先采样的点做raycast，在每根ray上查询最近的特征点。keyframe还是存在的，用于Bundle Adjustment。本文的hashing方法与keyframe同步更新，仅用于查找当前视野下的特征点。

### Redesigning SLAM for Arbitrary Multi-Camera Systems
为了让SLAM能更好适应任意多相机系统（更少的参数），做了三个改进：1. 自适应初始化，计算任意两个相机frustum之间的重叠区域（通过采样）判断是否形成stereo，来决定采用双目初始化还是5点法。2. 基于熵自动选择keyframe，通过一帧图像带来的信息增量。3. 把特征点同步存储在voxel map中（用hash table），这样每帧做PnP的时候就可以在voxel map里索引特征点，比在keyframe中索引得全面。

### Hybrid Camera Pose Estimation with Online Partitioning for SLAM
对ORB-SLAM的改进，是在tracking阶段，使用重叠区域进行Local BA。改进了Levenberg-Marquardt求解器，从而进一步提高了局部优化的效率。

## 2.结构特征
### Linear RGB-D SLAM for Atlanta World
设计了一种完全利用平面信息的基于滤波的SLAM算法。

### Lidar-Monocular Visual Odometry using Point and Line Features
基于线特征的SLAM还存在一些问题，由于线特征的三角化非常不稳定，从而导致视觉SLAM系统不稳定。直接应用线特征到激光-视觉里程计中不是一件简单的事情。为此，本文提出了一种鲁棒和高效的基于点和线特征的激光视觉里程计方法。具体地，该系统提供一种鲁棒的方法从LiDAR点运数据中提取点和线的深度，避免了可能出现奇异性的三角化。深度先验也被构建为深度因子来提高位姿估计的准确性。此外，为了克服帧帧里程计的尺度漂移，我们还在每个关键帧中恢复尺度，通过新的尺度修正优化方案。

### StructVIO: Visual-Inertial Odometry With Structural Regularity of Man-Made Environments
基于点和线特征，利用了Atlanta World假设的VIO系统设计。

## 3.特殊视觉特征
### Ultra-High-Accuracy Visual Marker for Indoor Precise Positioning
设计了一种新的视觉marker，做室内定位，还开发了一种计算方法，该方法通过使用良好的姿态精度来最小化重投影误差，从而将标记位置误差最小化，精度可以达到10cm。

### Ground Texture Based Localization Using Compact Binary Descriptors
针对地面纹理，设计一种特征提取和匹配的方式用于定位和全局重定位。基于一种新颖的匹配策略，称之为身份匹配，它基于紧凑的二进制特征描述符。

## 4.VIO
### Inertial-Only Optimization for Visual-Inertial Initialization
提出一种成功率更高、尺度估计更准的VIO初始化方法。V和I分别优化（VO所给轨迹误差对IMU来说可以忽略），IMU优化过程中考虑了噪声分布，是MAP估计。已开源 ORB_SLAM3。

### Uncertainty-Based Adaptive Sensor Fusion for Visual-Inertial Odometry under Various Motion Characteristics
针对IMU测量存在退化的场景，设计了一种方法在纯图像模式和两种VIO模式（只用陀螺仪和陀螺仪+加速度计）之间自适应切换，避免低信噪比的IMU测量影响定位精度。

### Hierarchical Quadtree Feature Optical Flow Tracking Based Sparse Pose-Graph Visual-Inertial SLAM
基于VINS-Mono开发了新的稀疏姿态图视觉惯性SLAM（SPVIS），前端提出分层四叉树方法提高光流特征跟踪的性能，大幅降低需要的特征点数量。后端采用固定时长的滑动窗口做全局优化，窗口之外仅保留位姿图。整个系统效率很高，从而支持长期运行。

### OpenVINS: A Research Platform for Visual-Inertial Estimation
提出了一个VIO的综合性框架，用于学术界和工业界的视觉惯性估计研究。该代码库对通常需要的视觉惯性估计特性具有开箱即用的支持，这些特性包括：1.在流形滑动窗口上的Kalman滤波器；2.在线摄像机的内、外部校准；3.摄像机到惯性传感器的时间偏移校准；4.具有不同表示和一致的首次估计 Jacobian（FEJ）处理的SLAM地标；5.用于状态管理的模块式系统；6.可扩展视觉惯性系统模拟器；7.用于算法评估的扩展工具箱。

## 5.直接法和稠密SLAM
### FlowNorm: A Learning-based Method for Increasing Convergence Range of Direct Alignment
直接法优化中，当初始估计不好时很容易陷入局部最优。本文提出一种像素加权优化方法，通过比对每个像素的梯度方向和（CNN估计得到的）光流方向的一致性得到权重。在以DSO和BA-Net为基准的试验中能有效提升效果，可以容许更多的跳帧。

### Robust RGB-D Camera Tracking using Optimal Key-frame Selection
基于RGBD的稠密 SLAM系统设计。前端有一个frame-to-model的tracking线程，自适应融合纹理定位（DVO）和结构定位（ICP）的结果输出初始位姿；还有一个frame-to-frame的回环检测线程，用SIFT特征匹配和BOW检索得到当前帧与历史帧的关联输出给后端。后端在所有帧形成的位姿图中寻找一个子集进行优化（先当作集合覆盖问题，找到邻域可以覆盖全部节点的最小节点子集，再用BFS添加节点使子集连通）。在TUM上定位精度不错。

### Voxgraph: Globally Consistent, Volumetric Mapping using Signed Distance Function Submaps
关注建图和全局位姿优化，不涉及前端。固定N帧建一个子图（每帧的odom都存着，因为LCD不一定给哪一帧的结果），重合度高的子图之间做配准，得到除了odom和回环之外的第三类位姿约束，共同做位姿图优化。子图用TSDF地图，融合后输出ESDF全局地图。
























