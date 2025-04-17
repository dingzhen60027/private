文件说明
//主程序
vision.cpp为图像采集节点-彩色相机
vision_processing.cpp为图像处理节点
//测试程序
vision_control_test.cpp为相机控制测试
vision_display_test.cpp为传输图像显示测试


图像采集节点采用大恒相机库进行相机控制，采集图像格式为RGB //若使用opencv控制相机，采集图像格式为BGR
//大恒相机采集图像尺寸为 1920*1200
//具体开发或api的使用可以参考大恒图像的官方案例example
下载地址https://www.daheng-imaging.com/downloads/

图像处理节点采用opencv进行图像处理
opencv彩色图片的格式为 BGR ！！！
注意节点间通信不要搞错了格式：vision_processing.cpp中图片处理统一采用 BGR 格式 //若接收图像为RGB，处理前要先转成BGR
//现在将传输的图片格式统一为 BGR
