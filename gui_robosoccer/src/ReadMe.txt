文件说明
所有文件共同组成一个GUI节点（内含若干小节点）

main.cpp为整个程序的启动文件
mainwindow.cpp为主界面文件

rclcommx.cpp为ROS2节点文件，开启新线程运行（使用了1、3、4）

globalVar.cpp/globalFun.cpp为全局变量/函数

HSIslider.cpp、ImageLabelControl.cpp、ImageLabelEnlarge.cpp、ImageLabelColor.cpp、ImageLabelDrawI.cpp为界面部件的提升文件

额外的文件 rclcomm.cpp 和 rclcomm2.cpp 为弃用或测试文件