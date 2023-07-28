# STM32

-   ST公司开发的32位单片机 

![image-20230727163935701](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727163935701.png)

-   本次使用STM32F1

![image-20230727164153633](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727164153633.png)

-   本次使用ARM Cortex M3

![image-20230727164718163](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727164718163.png)

-   Peripheral外设

![image-20230727164909390](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727164909390.png)

-   深色是在内核里面的外设，其他在外面

![image-20230727170133631](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727170133631.png)![image-20230727170153824](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727170153824.png)![image-20230727170222864](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727170222864.png)

#### 系统结构

![image-20230727170759341](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727170759341.png)

-   ICode 指令总线，加载程序指令
-   DCode 数据总线，加载
-   System 系统总线，SRAM储存运行时的数据
-   Flash 内储存编写的程序
-   AHB系统总线，挂载主要外设
-   APB先进外设总线，连接一般的外设
-   AHB > APB2 > APB1 性能上
-   DMA内核的小秘书，搬运大量数据
    -   发送请求，DMA获得总线控制权，访问并且转移数据

#### 引脚定义

![](C:/Users/24962/Desktop/STM32%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B%E8%B5%84%E6%96%99/STM32F103C8T6%E5%BC%95%E8%84%9A%E5%AE%9A%E4%B9%89.png)

-   红色：电源祥光
-   蓝色：最小系统相关
-   绿色：IO口功能口
-   正常能容忍3.3V电压（需要电平转换），有FT的可以容忍5V电压
-   主功能是默认功能，默认复用（用于IO口）
-   重定义，两个要用的都在一个端口上，可以映射到其他端口上

1.   VBAT 备用电池可以给RTC时钟和备份寄存器供电

2.   IO、侵入检测、RTC

     -   侵入检测引脚断开可以删除数据
     -   RTC输出RTC校准时钟，闹钟脉冲，秒脉冲

3 - 4.  IO、32.768KHz的RTC晶振

5 - 6. 主晶振，8MHz，锁相环电路可以倍频率，产生72MHz的主时钟频率

7. NRST 低电平复位引脚

8 - 9. 内部模拟部 的电源， ADC，RC震荡

10 - 19. IO口

-   PA0兼具了WakeUp的功能，唤醒待机的STM

20.   PB2 配置启动模式

21 - 22. IO口

23 - 24. 系统主电源口，分区供电，其他几个也是

34 和 37 - 40. IO口或调试端口，调试和下载程序

	- 支持SWD和JTAG两种调试方式
	- SWD两根线 SWDIO和SWDCLK
	- JTAG五条线 JTMS，JTCK， JTDI，JTDO，NJTRST

#### 启动配置

-   指定程序开始的位置，一般在**FLASH程序储存器**开始执行

![image-20230727173549230](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727173549230.png)

-   系统存储器：做串口下载的，BootLoader接收串口数据然后刷新到主闪存内（调试端口被占用时）
-   BOOT只在上电一瞬间有效，四个始终后就是其他功能了

#### 最小系统电路

![image-20230727175927667](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727175927667.png)

-   VCC和GND之间的电容叫做滤波电容
-   保证供电电压的稳定

![image-20230727180224833](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727180224833.png)

-   通过内部锁向环倍频得到72MHz的主频率
-   电容是起振电容
-   32.768KHz同理，32768是2的15次方，内部RTC经过2的十五次分频可以得到一秒的时间信号

![image-20230727180741657](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727180741657.png)

-   电容开始时充电引导GND,上电一段时间以后NRST变高
-   上电直接复位

![image-20230727190052425](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727190052425.png)

-   跳线帽可以改变选择BOOT是3.3还是GND
-   用于配置Boot电平

![image-20230727190216078](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727190216078.png)

-   下载用的

![image-20230727190318042](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727190318042.png)

-   上面是电源指示灯
-   下面是测试IO口的

![image-20230727190401186](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727190401186.png)

-   XC6204，USB5V转换3.3V，稳压芯片

![image-20230727190503087](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727190503087.png)

-   供电的滤波电容

![image-20230727190514360](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727190514360.png)

-   引出接口，方便使用

![image-20230727190536444](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727190536444.png)

-   USB接口，接入USB引脚，另外可以提供5V的供电

![image-20230727190619196](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727190619196.png)

-   两个晶振

#### 型号分类和缩写

![image-20230727222106130](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727222106130.png)

-   D： Density
-   VL：ValueLine
-   CL：Connectivity Line

#### 新建工程

![image-20230727222348203](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727222348203.png)

工程建立
添加工程必要文件 
Libraries->CMSIS -> CM3-> DeviceSupport-> ST-> STM32F10x-> startup-> arm->是启动文件，复制到工程模板（新建Start），回到STM32F10x（复制那三个文件stmxxxh、systemxxx.c、systemxxx.h）复制到Start,打开CM3->CoreSupport(将两个文件复制到Start)回到Keil将文件添加到工程->点击Target1,将Source Group1单击改名Start->右键选择添加已存在文件，打开Start文件夹，打开筛选器All files,添加启动文件（视频所用型号添加后缀md.s文件）和剩下所有.c.h文件->在工程选项添加该文件夹头文件路径（魔术棒按钮，C/C++,Include Paths栏点右边三个点按钮新建路径再点右三个点按钮添加Start路径，最后点ok）->打开新建工程文件夹添加新文件夹User（放main函数），接着keil里Target右键新建（组别）改名User添加文件选.c名叫main（注意路径选择Userwenj夹），在main里右键插入头文件stm32f10x.h(若用寄存器开发32，到此就完成工程建立) 
扳手里Encoding->UTF-8可以防止中文乱码

![image-20230727223028463](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230727223028463.png)
