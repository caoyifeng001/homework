1. 本次的作业为实现一个基于已知定位的建图算法，使用PPT中介绍的Occupany Mapping算法
2. 给出的代码内部包含了可视化界面，大家可以用RVIZ的map查看生成的地图

3. 本次给出的代码中可以通过readfile.h中的宏定义:READ_DATA_NUMBER来确定要读取的激光数据的帧数，建议前期调试的时候稍微取小一些(例如100)，调试完毕之后再取大，最大不要超过3000.

本次程序的运行过程为：
1. 文件occupany_mapping.cpp中的main函数中，有文件路径设置，需要根据自己电脑的路径进行修改。

2. 实现代码中要求的实现的内容，并且编译通过

3. 进入到OccupanyMappingProject目录下，运行:source devel/setup.bash

4. 运行指令：rosrun occupany_mapping    occupany_mapping即可

5. 在rviz中查看生成的地图

