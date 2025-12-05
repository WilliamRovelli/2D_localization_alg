# 2d定位系统
## 使用前：
1，localization_alg/agv_urdf放urdf文件（以车编号命名，如Y001.urdf） <br>
2，localization_alg/map放pcd文件（如test.pcd）<br>
3，修改localization_alg/src/localization_ndt/launch/abtr_localization.launch： <br>
    a,填地图名（如test.pcd，填test）<br>
    b,填urdf名（如Y001.urdf，填Y001）<br>
    c,填雷达话题、里程计话题、imu话题、base_frame_id。<br>
4，质量评估参数和NDT参数建议得到足够多数据再做确定。<br>
## agv在新场景下首次启动 
建议先录制bag，因为重定位需要打开rviz，目前没有web端。<br>
1，localization_alg目录下：<br>
source devel/setup.bash <br>
roslaunch localization_ndt localization.launch <br>
./relocalization.sh <br>
2，播放bag <br>
3，在rviz中使用2D estimate pose将agv拖拽到地图对应位置。 <br>
4，定位成功后，找一个固定位置（目标:后期agv维护或者关机移动处理，再次开机时可以自动重定位）,比如充电桩等。<br>
在固定位置，终端输入：rostopic echo /ndt_odom <br>
将对应的x、y、yaw填入到localization_alg/src/localization_ndt/config目录下的对应yaml文件内，比如YOO1.urdf和test_pcd会生成一个test_Y001.yaml。填到fixed_pose下。
