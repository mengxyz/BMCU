# BMCU

#### 介绍

BMCU以四通道为一个单位，目前以CH32单片机为主控设计。其设计所需资料均参考于网络公开资料及个人测试，程序基于Platform IO平台下CH32单片机的Arduino支持库设计，调用了robtillaart的CRC库。
 **注意:本项目遵循GPL2.0开源协议，但需要额外补充的是本项目禁止商业用途。** 

#### 使用说明及安装教程

 **1.  制造所需资料见BMCU整合打包文件** 
BMCU打包文件目前发布在群内，或可以从[https://oshwhub.com/bamboo-shoot-xmcu-pcb-team/bmcu](https://oshwhub.com/bamboo-shoot-xmcu-pcb-team/bmcu)的附件部分，找到V1.1版打包文件。

 **2.  所需刷写的固件见release** 

 **3.  安装教程暂无，后续推出** 

#### 软件架构
主要文件：

main.cpp/h：               负责调度各个模块

many_soft_AS5600.cpp/h：   IO模拟的，可同时和多个AS5600通讯的驱动

Motion_control.cpp/h：     负责硬件和运动状态调度

Flash_saves.cpp/h：        用于保存数据到flash

time64.cpp/h：             将Arduino的32位时基转为64位，防止连续运行几个月后溢出

Debug_log.cpp/h：          用于DMA发送DEBUG数据到串口3

BambuBus.cpp/h：           用于支持拓竹打印机的通讯，使用了串口0

Klipper.cpp/h：            （未完成，新建文件夹）用于支持klipper的通讯

ws2812.cpp/h：             IO模拟的WS2812驱动

调用的库：

robtillaart/CRC@^1.0.3     用于计算CRC校验

#### 参与贡献

设计师：

4061N-程序员，优化机械部分，设计PCB

括号-参与BMCU组件框架及BMCU到打印机支架的设计

测试和数据提供者：

风雪-提供打印机与AMS通讯的数据，参与部分测试

二月喵-提供少量高质量的通讯数据，参与部分测试

其他成员：

婆老-负责在线摸鱼

其他未列出的群友-提供了宝贵的测试数据和建议

