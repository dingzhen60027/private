#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "message_interface_robosoccer/msg/velocity_robots.hpp"

#include "hidapi.h"

#define ROBOTNUM 5

//发射器基频2400, 频率范围2400-2525, 黄方频率402<2402>，蓝方频率418<2418>，发送修改频率值分别为2和18
#define YELLOW 2
#define BLUE 18

class Communication : public rclcpp::Node
{
public:
    Communication(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s已创建!", name.c_str());
        
        isOpenUSB = false;
        if(initUSB())
        {
            RCLCPP_INFO(this->get_logger(), "USB初始化失败!!如果插上USB仍显示此错误请设置USB权限:命令窗口输入sudo chmod -R 777 /dev/bus/usb/");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "USB初始化成功!");
        }
        
        subscription_velocity_ = this->create_subscription<message_interface_robosoccer::msg::VelocityRobots>("robots_velocity", 1, std::bind(&Communication::velocity_sub_callback, this, std::placeholders::_1));
    }
    
    ~Communication()
    {
        if (dev_handle)
        {
            hid_close(dev_handle);
            hid_exit();
        }
    }

private:
    rclcpp::Subscription<message_interface_robosoccer::msg::VelocityRobots>::SharedPtr subscription_velocity_;
    
    unsigned char USB_Buffer[33];//需要比实际发送字节数多1个字节，多一个标头
    
    struct hid_device_info *devs, *cur_dev;
    hid_device* dev_handle;
    bool isOpenUSB;
    
    void velocity_sub_callback(const message_interface_robosoccer::msg::VelocityRobots::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "| 1号车速度设定：L:%f R:%f|", 
                                        msg->velocity[0].left_wheel, msg->velocity[0].right_wheel);
        
        if (isOpenUSB)
        {
            int r;
            // Init the command buffer.
	        memset(USB_Buffer,0x00,sizeof(USB_Buffer));
	        USB_Buffer[0] = 0x0; //hid_write标头Report ID
	        USB_Buffer[1] = 0xfd;
	        USB_Buffer[2] = 11;
            for (int CarNum = 1; CarNum<=ROBOTNUM; CarNum++)
            {
                USB_Buffer[CarNum*2+1] = msg->velocity[CarNum-1].left_wheel/2;//L_v / 2
                USB_Buffer[CarNum*2+2] = msg->velocity[CarNum-1].right_wheel/2;//R_v / 2
            }
            
            r = hid_write(dev_handle,USB_Buffer,33);
            if(r<0)
            {
                std::cout << "发送指令失败：" << hid_error(dev_handle) << std::endl;
            }
        }
    }
    
    int initUSB()
    {
        if (hid_init())
		    return -1;
        
        //-------------Check the useful drives---------------//
        /*
        devs = hid_enumerate(0x0, 0x0);
	    cur_dev = devs;
        while (cur_dev) 
        {
		    printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
		    printf("\n");
		    printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
		    printf("  Product:      %ls\n", cur_dev->product_string);
		    printf("  Release:      %hx\n", cur_dev->release_number);
		    printf("  Interface:    %d\n",  cur_dev->interface_number);
		    printf("\n");
		    cur_dev = cur_dev->next;
	    }
	    hid_free_enumeration(devs);
	    */
        //----------------------------------------------------//

        // Open the device using the VID, PID,
	    dev_handle = hid_open(0xffff, 0x2, NULL);
	    if (!dev_handle) 
	    {
		    printf("unable to open device!!\n");
     		hid_exit();
     		return 1;
	    }
	    else
	    {
	        // Init the launcher frequency.
	        memset(USB_Buffer,0x00,sizeof(USB_Buffer));
	        USB_Buffer[0] = 0x0; //hid_write标头Report ID
	        USB_Buffer[1] = 0xfc;
	        USB_Buffer[2] = 1;
	        //USB_Buffer[3] = YELLOW;
	        USB_Buffer[3] = BLUE;
	        int r = hid_write(dev_handle,USB_Buffer,33);
            if(r<0)
            {
                std::cout << "修改频率失败：" << hid_error(dev_handle) << std::endl;
                return 1;
            }
            else
            {
                isOpenUSB = true;
            }
	    }
        
        return 0;
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communication>("communication");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
