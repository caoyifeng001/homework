1. 本次的作业为实现一个里程计去除激光雷达运动畸变的模块
2. 本次的作业里面有两个工程:champion_nav_msgs和LaserUndistortion；大家需要首先编译安装champion_nav_msgs，按照champion_nav_msgs的readme文件执行即可，注意根据自己ubuntu的不同版本做修改。

本次程序的运行过程为：
1. 实现LidarMotionCalibration函数，并进行编译
2. 在LaserUndistortion下，进行source：source devel/setup.bash
3. 运行launch文件，执行本条指令的时候，必须保证没有任何ros节点在运行，roscore也要关闭。
4. 进入到 /bag目录下，运行指令：rosbag play –clock odom.bag。
5. 如果一切正常，则会看到pcl的可视化界面，当可视化界面中存在数据的时候，按R键即可看到结果。红色为畸变矫正前，绿色为畸变矫正后。

