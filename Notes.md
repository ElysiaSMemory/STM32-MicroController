# STM32

## 简介

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

## GPIO(General Purpose Input Output)

![image-20230728165456887](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728165456887.png)

-   所有的GPIO都是挂接在APB2外设总线上的

![image-20230728165623250](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728165623250.png)

![image-20230728165630656](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728165630656.png)

-   命名方式从GPIOA，GPIOB，GPIOC以此类推（PA，PB，PC）
-   每个外设有16个引脚
-   寄存器有32位，但是端口只有16位置，所以只有低16位寄存器有效

### GPIO位结构

![image-20230728194340852](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728194340852.png)

-   寄存器 - 驱动器 - IO口引脚

![image-20230728201936661](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728201936661.png)

-   上面是输入部分，下面是输出部分

![image-20230728205443019](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728205443019.png)

-   保护二极管对输入电压进行限V
-   如果输入电压大于3.3V，上面的二极管就会导通，输入电压就会流入VDD
-   如果输入电压小于-0V，下面的二极管就会导通，输入电压就会流入VSS

![image-20230728211242297](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728211242297.png)

-   上拉电阻到VDD，下拉电阻到VSS
-   上面导通下面断开：上拉输入模式
-   下面导通上面断开：下拉输入模式
-    两个都断开: 浮空输入模式
-   如果引脚没有连接，端口处于浮空状态（未知），容易混乱。上下拉保证电平高低
-   弱上下拉，不影响输入操作

![image-20230728212013176](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728212013176.png)

-   施密特触发器：对输入电压整形。
    -   输入电压**大于**阈值，输出瞬间高电平
    -   输入电压**小于**阈值，输出瞬间底电平
    -   应对失真信号，超微的抖动不会超过阈值，所以不会有影响

![image-20230728212056212](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728212056212.png)

-   现在可以写入输入寄存器
-   模拟输入链接得到ADC
-   复用功能输入：用于其他外设（接收数电）

![image-20230728212300747](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728212300747.png)

-   二选一进入输出控制
-   输出寄存器：IO口输出
    -   =& =|
    -   只能整体读写，用**位设置/清楚**寄存器改单位置
    -   读写“位带”（位寻址）

![image-20230728213311492](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728213311492.png)

-   两个MOS管（电子开关）
-   开关把IO口接到VDD或者VSS
-   **推挽输出**
    -   P和N-MOS都有效
    -   1时上面导通下面断开接到VDD
    -   0时下面导通上面断开接到VSS
    -   **强驱动能力**，对IO口有绝对控制权
-   **开漏模式**
    -   只有N-MOS在工作
    -   1时下面断开，高电阻模式（无驱动能力）
    -   0时下面导通，VSS低电平（有驱动能力）
    -   作为通信协议的引脚（驱动方式）。可以避免多个设备之间的干扰
    -   可以接上拉电阻。在高电平的时候直接被上拉到5V
    ![image-20230728213945204](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728213945204.png)
-   **关闭**（输入模式）
    -   两个MOS都无效，输出关闭，端口信号由外部控制

### 八种模式

![image-20230728214141944](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728214141944.png)![image-20230728214307471](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728214307471.png)

-   模拟输入是ADC专属

![image-20230728230925330](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728230925330.png)

![image-20230728231016419](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728231016419.png)

![image-20230728231115608](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728231115608.png)

### 寄存器

-   每一个端口用四位配置，16个端口需要64位
-   端口配置低寄存器，端口配置高寄存器
-   端口输入数据寄存器低16位置有效，对应16个引脚
-   端口输出数据寄存器低16位置有效，对应16个引脚
-   端口位设置，清除寄存器（高16位清除，低16位设置）（对一个端口同时设置和清除，同步性高）
-   端口位清除寄存器 = 清除寄存器(低16位有效)
-   端口配置锁定寄存器：防止意外更改

### GPIO输出

-   使用RCC开启GPIO的时钟
-   使用GPIO_Init初始化GPIO
-   使用输入或者输出函数控制GPIO口
-   默认开机输出为0

#### RCC的库函数

```c
// AHB外设时钟控制
/*
@brief  Enables or disables the AHB peripheral clock.
@param  RCC_AHBPeriph: specifies the AHB peripheral to gates its clock.
@param  NewState: new state of the specified peripheral clock.
This parameter can be: ENABLE or DISABLE.
*/
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
// APB2外设时钟控制
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
// APB1外设时钟控制
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);

```

#### GPIO库函数

```c
// Reset the GPIO
// PARAM: GPIOA, GPIOB...
void GPIO_DeInit(GPIO_TypeDef* GPIOx);

// Reset the AFIO
void GPIO_AFIODeInit(void);

// Using Struct to Initialize GPIO
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);

// Give Struct a default value
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);

// !!!! 按位或可以选择多个设备

//// WRITING
// @brief Set selected IO to be high
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
// @brief Set selected IO to be low
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
// @brief Set selected IO to be BitVal
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
// @brief Set all 16 IOs to be PortVal
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);


// MODE
// Analog Input
GPIO_Mode_AIN = 0x0,
// Floating Input
GPIO_Mode_IN_FLOATING = 0x04,
// In Pull Down
GPIO_Mode_IPD = 0x28,
// In Pull Up
GPIO_Mode_IPU = 0x48,
// Out Open Drain 开漏输出
GPIO_Mode_Out_OD = 0x14,
// Out Push Pull 推挽输出
GPIO_Mode_Out_PP = 0x10,
// Atl Open Drain 复用开漏
GPIO_Mode_AF_OD = 0x1C,
// Atl Push Pull 复用推挽
GPIO_Mode_AF_PP = 0x18

```

### LED, Buzzer

![image-20230728234946547](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728234946547.png)

-   三极管开关进行蜂鸣器驱动（0响1关闭，低电平触发）

![image-20230728235446930](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728235446930.png)

![image-20230728235512581](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728235512581.png)

![image-20230728235556971](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728235556971.png)

-   两种不同的输出方式，按照驱动能力去设计电路（一般高弱底强）

![image-20230728235652821](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230728235652821.png)

-   三极管开关驱动
-   三极管：基极，发射极，集电极
-   上：PNP三极管（0）
-   下：NPN三极管（1）
-   负载位置很重要，因为需要启动电压

### Bread Board

![image-20230729001053685](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230729001053685.png)

### 按键

![image-20230731142134147](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731142134147.png)

-   注意需要过滤按键抖动
-   最简单的方法是加一段延时

### 传感器模块

-   光敏电阻，热敏电阻，对射红外，反射红外

![image-20230731142348878](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731142348878.png)

-   电阻的变化不容易被察觉，所以与定值电阻串联分压得到模拟输出

![image-20230731142611212](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731142611212.png)

-   N1是可变电阻，与R1进行分压
-   C2是滤波电容，保证输出平滑（一端接在电路中，一端接地）分析时可以抹掉
-   **当N1的阻值变小，下拉作用增强， AO端口电压被拉低（极端0， GND）**
-   **当N1的阻值变大，上拉作用增强， AO端口电压被升高（极端1， VCC）**
-   类比弹簧，阻值越小拉力越强

![image-20230731143915243](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731143915243.png)

-   中间是VCC/2
-   AO就是模拟电压的输出

![image-20230731144123328](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731144123328.png)

-   同时支持数字输出，二值化后的输出，由LM393芯片完成（电压比较器芯片）

![image-20230731144237874](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731144237874.png)

-   电压比较器，哪边电压大瞬间到那边的极端（VCC， GND）

![image-20230731144443332](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731144443332.png)

-   电位器可以输出IN-的可调阈值电压
-   这个电压和上面的比较

![image-20230731144516169](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731144516169.png)

-   比较结果是DO，然后引到数字电压输出

![image-20230731144553505](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731144553505.png)

-   左边是电源指示灯，右边是输出指示灯

![image-20230731144634085](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731144634085.png)

-   R5上拉电阻保证默认高电平

![image-20230731145245833](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731145245833.png)

-   图1，默认按下是0，松开是悬空(上拉输入模式)
-   图2，外部上拉电阻，按下是无穷大（无电阻）的力下拉引脚GND (浮空，上拉输入模式)

![image-20230731145258655](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731145258655.png)

-   按下为1
-   要求配置为下拉输入模式

![image-20230731145439813](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731145439813.png)

-   光敏传感器：亮的情况下，输出低电平，指示灯亮

### C语言相关参考

![image-20230731145738685](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731145738685.png)

![image-20230731150331878](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731150331878.png)

![image-20230731150542921](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731150542921.png)

![image-20230731150728392](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731150728392.png)

-   可以用 Typedef来简化定义

![image-20230731151956279](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731151956279.png)

![image-20230731152115650](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731152115650.png)

-   顺序的数字编译器会自动填上

### GPIO输入

```c
//// READING
// @brief 读取一位
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
// @brief 读取整个寄存器
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
// @brief 读取输出输出寄存器的某一位，不是端口输入，用来看输出的是什么
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
// @brief 读取输出整个输出寄存器，不是端口输入，用来看输出的是什么
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
// @brief 锁定引脚配置
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
```

![image-20230731163707272](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731163707272.png)

## OLED

![image-20230731205101518](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731205101518.png)

![image-20230731221616642](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731221616642.png)

-   每一个像素都是可以自发光的发光二极管，不像LCD需要背光，所以省电，响应快，有更快的刷新频率
-   四针脚I2C通讯协议， 七针脚SPI通讯协议

![image-20230731224026361](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731224026361.png)

![image-20230731225539368](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731225539368.png)

![image-20230731225553755](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731225553755.png)

-   行-列

## 中断系统

### 简介

 ![image-20230801160643013](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801160643013.png)

![image-20230801161246670](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801161246670.png)

![image-20230801161437408](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801161437408.png)

-   EXTI 外部中断

-   TIM 定时器

-   ADC 模数转换

-   USART 串口

-   SPI 通信

-   I2C 通信

-   RTC 实时时钟

NVIC是STM32内部用于管理中断，分配优先级的

### 中断类型

![image-20230801161713837](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801161713837.png)

灰色的是内核中断（高深，不常用），下面的是外设中断

![image-20230801161954762](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801161954762.png)![image-20230801162000344](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801162000344.png)

0：窗口看门狗，如果没有及时喂狗，看门狗就会申请中断，可以进行错误检查
1：PVD电源中断，如果供电不足会申请中断，可以在中断里赶紧保存数据
。。。。。。

外设电路检测到异常就可以申请中断

EXTI0到EXTI4 && EXTI9_5 && EXTI15_10是**外部中断**

程序中的中断函数地址是由编译器分配的，不固定。中断跳转因为硬件限制只能到固定地方执行程序。只有在那个固定地址里再一次引导才能找到不固定的函数位置。

中断发生后跳转到固定位置（**中断向量表**），在固定位置由变迁一起再加上跳转到中断函数的代码

![image-20230801162713690](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801162713690.png)

### NVIC基本结构

统一分配中断优先级和管理中断
内核外设，CPU助手
![image-20230801163643371](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801163643371.png)

-   n代表一个设备可能占用多个中断
-   可以类比医院有个区号大厅分配病人登记，叫号系统判断紧急程度，分配给医生处置

### 优先级分组

-   **响应优先级**：这个人直接插队在外面排队的人，正在看病的看完马上轮到他
-   **抢占优先级**：这个人直接插队挤开正在看病的人，靠边站（优先级高的是可以嵌套的）。这个人看完了，之前的人继续看病。

每个中断有16个优先级，所以我们进行**分组**

-   四位刚好可以表示0~16，也就是16个优先级
-   **数字越小优先级越高**， 如果响应等级相同，按照图表排列处理

![image-20230801164737900](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801164737900.png)

我们会选好其中的分组方式，注意取值范围可能不同

### EXTI外部中断（Extern Interrupt）

引脚电平发生变化，触发中断

![image-20230801165004572](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801165004572.png)

-   相同的PIN指的是比如PA0, PB0, PC0...
-   外部中断可以从低功耗模式的停止模式下唤醒STM32（用于唤醒停止模式）
-   事件响应：可以选择不触发中断（不通向CPU）通向其他外设

### EXTI基本结构

-   AFIO在3 * 16个引脚中选择一个引脚连接到EXTI的通道里（所以不能重复）
-   EXTI所以有20个输入信号
-   ST把外部中断的5~9， 9~15分配到了一个通道里，减少压力

![image-20230801165830205](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801165830205.png)

![image-20230801165905938](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801165905938.png)

**AFIO内部结构，MUX数据选择器**

![image-20230801170022043](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801170022043.png)

注意也可以换**复用引脚功能的定义**

![image-20230801170328369](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801170328369.png)

**EXTI内部框图**

-   上路是触发中断，下路触发事件

![image-20230801170422186](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801170422186.png)

挂起寄存器相当于中断标志位，读取这个寄存器可以知道是哪一个通道触发

![image-20230801170517883](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801170517883.png)

看是否屏蔽中断，然后才去中断

![image-20230801170620122](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801170620122.png)

另一条路的走法

![image-20230801170639461](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801170639461.png)

可以通过总线访问这些寄存器

### 红外对管传感器

-   遮住中间输出高电平，输出指示灯灭掉
-   灭掉再亮起会产生下降沿

### 旋转编码器

![image-20230801195213794](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801195213794.png)

-   图一：光栅式：光栅编码盘转动的时候，红外传感器的红外光会接收到透过，不透，透过，不透的信号，也就是方波，个数是角度，频率是转速。外部中断可以捕捉方波边沿来实现操作。（**单相输出**：正反没法区别）
-   图二图三：机械触点：中间有个按键。类比光栅，在转盘上有金属触点。依次接通和断开两边的触点。两侧触点的通断是90度的相位差（**两相正交输出**）

![image-20230801195657509](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801195657509.png)![image-20230801195948030](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801195948030.png)

正向旋转时，B相输出是滞后90度

![image-20230801200036666](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801200036666.png)

反向旋转时，B相输出是提前90度

![image-20230801232458925](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801232458925.png)

-   在A的下降沿读取B的电平，高是正转，低是反转（不爽，第一种情况一转，一格，就中断，第二种情况转完进入中断）

![image-20230801232806882](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801232806882.png)

-   **都触发中断**
    -   **在B下降沿&&A底电平判断正转**
    -   **在A下降沿&&B底电平判断反转**
    -   转到位了才执行加减操作

-   图四：霍尔传感器类型：内部有一个磁体，边上有两个位置错开的霍尔传感器，当磁铁旋转，霍尔传感器可以输出正交方波信号

#### 硬件电路

![image-20230801200634405](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801200634405.png)

-   默认上拉，导通后就和中间GND直接拉低，通过R3（输出限流电阻）后输出低电平A
-   C1是滤波电容

![image-20230801201257837](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801201257837.png)

### 手册相关

-   NVIC寄存器相关参考手册``STM43F10xxx Cortex-M3编程手册``
-   中断分组相关的寄存器在SCB里面
-   通过改变寄存器可以AFIO，GPIO的映射关系

### 代码相关（AFIO, EXTI），配置步骤

![image-20230801205759992](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801205759992.png)

**打通信号电路**

1.   RCC启用所有用到外设的时钟，**AFIO和GPIO都是APB2，EXTI和NVIC（内核内的外设）默认一直打开**
2.   配置GPIO为输入模式
     -   注意不知道什么模式参考手册![image-20230801210859662](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801210859662.png)

3.   配置**AFIO（和GPIO在一个Library文件内）**，选择我们用的GPIO，连接到后面的EXTI

     ```c
     // @brief 复位AFIO，配置清除
     void GPIO_AFIODeInit(void);
     
     // @brief 配置AFIO的事件输出功能
     void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
     void GPIO_EventOutputCmd(FunctionalState NewState);
     
     // @brief 对引脚重新映射
     void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
     
     // @brief 配置AFIO数据选择器来选择中断引脚
     void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
     
     // @brief 以太网相关
     void GPIO_ETH_MediaInterfaceConfig(uint32_t GPIO_ETH_MediaInterface);
     
     ```

     ![image-20230801212057596](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801212057596.png)

     **注意配置好后EXTI是默认和端口数据的数字一样**

4.   配置EXTI配置边沿触发模式（上升下降，双边），**中断响应**或事件响应

     ```c
     // @brief 恢复上电默认状态
     void EXTI_DeInit(void);
     
     // @brief 根据结构体初始化EXTI！！常用 
     void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
     
     // @brief 根据默认结构体初始化EXTI
     void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
     
     // @brief 软件触发外部中断
     void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);
     
     // @brief 状态标志位（挂起寄存器）获取是否指定标志位被置1
     FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
     
     // @brief 清除指定标志位，置0
     void EXTI_ClearFlag(uint32_t EXTI_Line);
     
     // @brief （置标志位后会中断。如果需要在中断函数里查看和清除标志位用这两个。）获取中断标志位是否被置1
     ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
     
     // @brief 清除中断挂起标志位
     void EXTI_ClearITPendingBit(uint32_t EXTI_Line);
     
     ```

     

5.   配置**NVIC(Library里在misc)**，选择合适的优先级，从这里进入CPU

```c
// @brief 中断分组设置
void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
// pre-emption priority and subpriority
//先占优先级（急）/响应优先级（不急）

// @brief 根据结构体初始化
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
// NVIC_IRQChannel： 跳转后选择STM32F10X_MD 本芯片型号， EXTERN 1-4 是通用的，查看文件186行


// @brief 设置中断向量表
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);

// @brief 系统低功耗配置
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);

```

6.   写**中断函数**

查看启动文件![image-20230801224531014](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230801224531014.png)

``IRQHandler `` 就是中断相关的东西（函数名称）

```c
// EXAMPLE
void EXTI15_10_IRQHandler(void)
{
	// Check Flag is right?
	if (EXTI_GetITStatus(EXTI_Line14) == SET)
	{
		// Clear Flag
		EXTI_ClearITPendingBit(EXTI_Line14);
		
		// Code Here
	}
}
```

## TIM定时器

### 简介

![image-20230802162841732](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802162841732.png)
比如在STM32中，基准时钟频率在72MHz，如果我记72个数字那就是1MHz，也就是1us的时间:
72,000,000 #/sec  => 1/72,000,000 sec/#
1/72,000,000 * 72 = 1/1,000,000 sec/#(72)
1/1,000,000 sec = 0.000001 sec = 1us
72 * 1,000 = 7,2000 => 1ms

```c
Counts = Desired Delay (us) * Timer Frequency (Hz)
```

![image-20230802162905297](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802162905297.png)

-   **时基单元**（16位）
    -   计数器：用来执行计数定时的寄存器，每来一个时钟，计数器++
    -   预分频器：用来对计数器的时钟进行分频，让计数更加灵活
    -   自动重装计时器：计数的目标值
-   最大的情况，2^16^ = 65535；72MHz/65535/65535 = 中断频率；1/中断频率 = 59.65s (除以两次是因为分频器会让频率变慢，比如两次脉冲算一次脉冲)

-   如果话不够可以用级联的方式，一个计时器的输出当作另一个计时器的输入

![image-20230802165852588](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802165852588.png)

### 定时器类型

![image-20230802170001635](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802170001635.png)

主要学习通用定时器
高级定时器主要用于三相无刷电机

### 基本定时器结构

#### 定时中断

![image-20230802210846386](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802210846386.png)

-   预分频器连接到基准时钟的输入（基本型只能选择内部时钟）理解为连接到内部时钟（72MHz）
-   关于分频：实际分频系数 = 预分频器的数值 + 1。比如，1分频就是输入72/**2** = 36 MHz。
-   自动重装寄存器给予重装目标值。当计数和目标值一样就是时间到了 - **产生中断信号** - 清零计数器。
-   一般这种中断叫做**更新中断** ``UI`` 之后通往NVIC
-   另一个中断信号是事件中断，用于触发其他内部电路的工作，但是不触发中断

#### 主（从）模式触发DAC

让内部的硬件在不受程序控制下实现自动运行

每隔一段时间就要触发DAC，让他输出下一个电压点

-   正常思路：定时器产生中断，中断中调用代码手动触发DAC转换，然后输出（频繁中断不好）
-   定时器设计了主模式，映射更新事件到触发输出TRGO，用TRGO触发DAC即可

![image-20230802215548892](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802215548892.png)

### 通用计时器结构

![image-20230802215716737](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802215716737.png)

![image-20230802215731759](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802215731759.png)

-   时基单元是一样的，和基本计时器，流程也是一样的
-   通用计时器不仅仅有向上计数。同时支持向下计数和中央对齐模式
    -   向下计数：重装值开始，向下自减，减0之后，恢复重装并且中断
    -   中央对齐模式：0开始到增加到重装值触发中断，然后开始往下减少到0又一次触发中断

![image-20230802220152026](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802220152026.png)

通用定时器除了使用内部时钟（72MHz），也可以使用外部时钟 

-   来自TIMx_ETR引脚上的外部时钟

![image-20230802220459914](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802220459914.png)

![image-20230802220526688](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802220526688.png)

-   一路从 ETRF进入触发器，作为时钟给时基单元
-   这叫做**“外部时钟模式2”**

接下来还可以有TRGI（Trigger IN）出发输入，可以触发**从模式**，后面讲。。。“ **外部时钟模式1”**

从这一路进入的有以下引脚

-   ETRP外部时钟
-   ITR信号，来自**其他**定时器的时钟信号，分别来自其他四个的TRGO输出（通过这个可以串联计时器，第一个信号从更新事件的映射TRGO流出）
![image-20230802221146594](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802221146594.png)
![image-20230802221233318](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802221233318.png)![image-20230802221459531](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802221459531.png)

-   TI1F_ED（上升沿下降沿均有效），连接的是输入单元的捕获引脚

    ![image-20230802221611428](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802221611428.png)![image-20230802221713801](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802221713801.png)

定时器编码器接口可以读取正交编码器的输出波形

![image-20230802222012424](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802222012424.png)

TRGO可以映射内部事件，不仅限于

![image-20230802222123648](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802222123648.png)

输出比较电路，四通道

![image-20230802222134851](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802222134851.png)

-   可以用于输出PWM波形驱动电机

输入捕获电路，四通道

![image-20230802222208940](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802222208940.png)

-   可以用于测量输入方波的频率

捕获/比较寄存器

![image-20230802222301091](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802222301091.png)

-   被两边共用（左右不能同时使用）

### 高级定时器

![image-20230802222350427](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802222350427.png)

![image-20230802222400192](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802222400192.png)

-   主要改动是右边部分，其他差别不大

重复次数寄存器

<img src="C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802222445310.png" alt="image-20230802222445310" style="zoom:150%;" />

-   用这个计数器可以实现每过几个计数周期才发生一次中断或者事件（原来是每个周期都更新，又做了一次分频）

死区生成电路DTG Dead Time Generator

![image-20230802230928566](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802230928566.png)

-   防止硬件导致的直通现象
-   桥臂的上下管全部关闭

互补的输出引脚

![image-20230802232337770](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802232337770.png)

-   可以输出互补的PWM
-   用于驱动三相无刷电机（六个大功率开关管）
-   第四路没有变化

刹车输入

![image-20230802234039338](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802234039338.png)

-   切断电机输出防止意外

![image-20230804012928424](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804012928424.png)

![image-20230804162232327](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804162232327.png)

#### 滤波器的参数

在固定的时钟频率下采样（采样频率f）如果**N个采样都是相同的电平，输出这个东西**，如果不是，输出上一个电平或者默认低电平

``TIM_CKD_DIV1``

### 简化版本

![image-20230802234132010](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802234132010.png)

-   运行控制：控制启动停止，向上向下

![image-20230802234434093](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802234434093.png)

-   中断输出控制判断是需要什么中断

### 预分频器时序

![image-20230803162851040](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803162851040.png)

![image-20230803162905050](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803162905050.png)

-   CK_PSC是内部时间输入
-   CNT_EN是计数器使能
-   CK_CNT是分屏器输出，也是计数器的输入

![image-20230803163241270](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803163241270.png)

使能后千半段预分频器的系数为1，输出和CK_CNT一样的信号。后半段预分频器的系数为2，每两次算一个，时钟变成一半

![image-20230803164208012](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803164208012.png)

在计数器时钟的驱动下计数器也在上升。推测重新装载值ARR是FC，同时在下一个时钟沿触发触发更新寄存器和更新事件。

![image-20230803164616767](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803164616767.png)

设计严谨，**不允许计数周期没有完成就改变分频系数**。如果在周期内改变了分频系数，这个值会通过**预分频缓冲器**（影子）在本计数周期结束时的下一个时钟沿生效。

![image-20230803164656149](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803164656149.png)

预分频器内部也是计数器，在分频系数为0时一直输出；在分频系数1时，010101，每次回到0输出，**从而输出输入频率的二分频**

![image-20230803164830252](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803164830252.png)

### 计数器时序

![image-20230803170122997](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803170122997.png)

-   UIF是中断的标志位，至一后需要**手动清零**
-   ARR有缓冲寄存器

#### 溢出频率

![image-20230803170209497](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803170209497.png)

-   CK_PSC 内部时钟输入
-   PSC 分屏系数
-   ARR 重新装载值

#### 计数器影子（预装）寄存器是否开启

![image-20230803170624898](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803170624898.png)![image-20230803170637404](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803170637404.png)

-   通过更改ARPE位
-   同理如果启用预装（影子），更改在下一个技术周期才有效

### RCC时钟树

![image-20230803171811397](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803171811397.png)

STM32用来产生和配置时钟，并且把配置好的时钟发送到外设的系统

-   AHB往左都是时钟的产生电路，往右都是时钟的分配电路

![image-20230803172603464](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803172603464.png)

SYSCLK是系统时钟，由两个高速晶振提供， AHB，APB1，APB2均来自这两个晶振。

-   外部的石英振荡器比内部的RC振荡器要更加稳定

![image-20230803173325935](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803173325935.png)

在SytemInit里，（1）首先启动**内部**时钟8MHz运行，（2）然后启动外部时钟，配置外部时钟进入PLL（3）进行8MHz**倍频9**倍，得到72MHz；等到锁相环稳定后再更换选择**锁相环输出为系统时钟**

-   **CSS是时钟安全系统**，一旦外部时钟失效就会自动切换回内部时钟，防止卡死事故

![image-20230803174809149](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803174809149.png)

### 程序相关

![image-20230803232608885](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230803232608885.png)

1.   RCC开启时钟
2.   选择时基单元的时钟源（定时中断我们使用内部时钟源）

```c
// @brief 恢复缺省配置（上电配置）
void TIM_DeInit(TIM_TypeDef* TIMx);
// @brief 时基单元初始化
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
// @brief 时基单元结构体赋默认值
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
// @brief 使能计数器（对应运行控制）
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
// @brief 使能中断（对应中断输出控制）
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);


// @brief 选择内部时钟（走内部时钟模式）
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
// @brief 选择ITRx其他定时器的时钟（ITRx其他定时器 - 外部时钟模式1）
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
// @brief 选择TIx捕获通道的时钟（GPIO - Tix捕获通道 - 外部时钟模式1）
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource, uint16_t TIM_ICPolarity, uint16_t ICFilter);
// @brief 选择ETR通过外部时钟模式1输入（GPIO - 外部时钟ETR - 外部时钟模式1）
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);
// @brief 选择ETR通过外部时钟模式2输入（GPIO - 外部时钟ETR - 外部时钟模式2）（不要触发输入的功能的话是等效的）
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);
// @brief 单独配置ETR引脚的预分频器，极性，滤波器这些参数
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);

// @brief 单独写预分屏值的，可以选择（更新事件生效或者手动主动产生更新事件生效）
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
// @brief 自动重装器预装功能配置（要不要影子）
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
// @brief 给计数器写入一个值
void TIM_SetCounter(TIM_TypeDef* TIMx, uint16_t Counter);
// @brief 给自动重装器写一个值
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint16_t Autoreload);
// @brief 获取计数器的值
uint16_t TIM_GetCounter(TIM_TypeDef* TIMx);
// @brief 获取预分频器的值
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
// @brief 获取标志位和清除标志位的
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);

void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);
```

1.   配置时基单元（用一个结构体即可）
2.   配置中断输出控制，允许更新中断输出到NVIC
3.   配置NVIC，打开定时器中断的通道打开定时器中断的通道，并且分配优先级
4.   写一个中断函数

## PWM and OC (Output Compare)

### 简介

<img src="C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804193135850.png" alt="image-20230804193135850" style="zoom:150%;" />![image-20230804193254880](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804193254880.png)

-   CCR就是捕获/比较寄存器（被捕获和比较公用）
-   CNT被所有的通路公用
-   （我们设置好CCR）当CNT大于，小于，或者等于CCR时，输出就会是**方波PWM波形**

![image-20230804194044052](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804194044052.png)

高级定时器由死区生成和取反输出的能力（用于三相无刷电机）

### PWM简介

**天下武功，唯快不破**，闪的够快，就看不出来（用于惯性系统）

![image-20230804194539659](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804194539659.png)

-   站空比和实际数值是线性关系
-   分辨率是占空比变化的精细度

### 输出比较通道（通用定时器）

![image-20230804195350769](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804195350769.png)

-   当CNT和CCR满足大于和等于的情况，输出模式控制器会根据**模式**反转REF信号，此信号通过选择器选择是否取反，然后输出到GPIO上
-   OC1也是CH1

![image-20230804195457547](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804195457547.png)

-   冻结可以用于：正在输出PWM，突然想要暂停一下
-   匹配时电平翻转可以输出50%PWM；**输出波形的频率 = 更新频率 / 2**![image-20230804202949802](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804202949802.png)

-   PWM模式2就是PWM模式1输出的取反，只改变了REF的极性而已（后面还有一个取反就是了）

![image-20230804203455203](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804203455203.png)

![image-20230804203817316](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804203817316.png)

-   可见CCR设置的越高，输出的站空比越大

### 参数计算

![image-20230804204330974](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804204330974.png)

-   Reso越小越好

### 输出比较电路（高级）

![image-20230804205252490](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804205252490.png)

两个推挽电路合成**H桥电路**，可以控制正反转，如果有三个即可驱动三相无刷。 因为两极必须互补所以内部两个输出端口就是相反的

![image-20230804205419474](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804205419474.png)

考虑到硬件本身的限制，如果直接切换，有可能导致短路现象。所以**死区生成电路**会在上面关闭时延时一小段时间，再导通下管，反之同理。

### 舵机

![image-20230804210313820](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804210313820.png)

-   SG90型号
-   50Hz，每个上升沿之间间隔20ms
-   根据高电平时间来调整角度，线性分配

![image-20230804210322710](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804210322710.png)

-   PWM信号输入到控制板，给控制板一个指定的目标角度
-   然后电位器检测当前输出轴的角度
-   如果大于目标或者小于目标电机会对应的正反转调整角度

![image-20230804211020258](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804211020258.png)

### 直流电机和驱动电路

-   130直流电机

![image-20230804211522702](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804211522702.png)

-   H桥可以控制电流流过的方向，所以可以控制正反转

![image-20230804211642704](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230804211642704.png)

-   VM电机驱动模块驱动电源。一般和电机电源同样
-   VCC逻辑电平输入端。和控制器一致
-   三个GND随便选一个即可
-   灰色填充是对应关系
-   PWMA接PWM信号输出端
-   其他可以接任意接口(控制见表格)
    -   确认方向后，转还是制动就看PWM了
-   STBY是待机，GND不工作，VCC工作

手册14单元。

### 编程相关

1.   RCC时钟开启要使用的TIM外设和GPIO外设时钟
2.   配置时基单元
3.   配置输出比较单元
4.   配置GPIO，初始化为**复用推挽输出** ···· GPIO_Mode_AF_PP
5.   运行控制，启动计数器

```c
// @brief 配置输出比较
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
// @brief 输出比较的结构体默认赋值
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);

// @brief 配置强制输出模式：运行中想要暂停并且强制输出高低电平
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);

// @brief 配置CCR（当CNT大于，小于，或者等于CCR时）的影子寄存器（预装功能）
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);

// @brief 配置快速使能，用的不多
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);

// @brief 不用的
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);

// @brief 单独设置极性，结构体内也是一样的
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);

// @brief 单独修改输出使能参数
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);

// @brief 单独修改输出比较模式的函数
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);

// @brief 单独修改CCR寄存器(站控比)的函数（重要）
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint16_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare4);\

 // @brief 单独修改PSC寄存器(预分频)的函数（重要）
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
    
// @brief 高级定时器必用
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);

```

![image-20230805004219197](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230805004219197.png)

注意，如果要给不同电机不同的PWM只需要使用 不同的通道即可

#### 参数计算

-   舵机要求周期是20ms，频率就是1/20ms = 50Hz
-   可以选择合适的PSC和ARR，这里用72和20000

![image-20230806211340142](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230806211340142.png)

### 引脚重映射

![image-20230805164747161](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230805164747161.png)

**关闭本身的复用**

```c
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); // Change PA0 to PA15
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

// 参考引脚定义图确定用哪一个
GPIO_Remap_SWJ_NoJTRST
GPIO_Remap_SWJ_JTAGDisable
GPIO_Remap_SWJ_Disable // 小心
```

![image-20230805165157351](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230805165157351.png)

## 输入捕获 Input Capture

![image-20230807232309807](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230807232309807.png)

-   指定电平跳变上下沿，把CNT的值写入到CCR中
-   CCR就是捕获/比较寄存器（被捕获和比较公用）

![image-20230807232813877](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230807232813877.png)

-   类似于外部中断，但是这里执行的是控制后续电路

### 测频率的方法

#### 测频法

![image-20230808183518182](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808183518182.png)![image-20230808183638967](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808183638967.png)

-   一般我们使用周期为1S，每来一个上升沿就是一个周期，计数+1；结果直接单位Hz
-   适合**测量高频**信号
-   得到的是**平均频率**，结果更新慢
-   实现方法，美来一次计次加一，然后定时器设定一秒中断，每次取值然后清零

#### 测周法

![image-20230808201706485](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808201706485.png)![image-20230808201033139](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808201033139.png)

-   捕获两个上升沿，测量两个之间的时间。
-   测量时间的也是用计数器，使用已知频率f~c~（假设单位时间内发生一次）计次，记一次的时间是T = 1 / f~c~ ，记N个数的时间就是N / f~c~；这个时候取倒数就是频率
-   适合**测量低频信号**，更适合宝宝体质
-   出**结果速度快**，更新更快，受到噪声影响较大

#### 中界频率

-   前两种**N越大误差越小**，不一定每个周期信号都是完整的（正负一误差）
-   把两个方法的N相等，把f~x~解出来，就得到中界频率f~m~
    -   **如果待测信号 > 中界频率，用测频法**
    -   **如果待测信号 < 中界频率，用测周法**

### 基本电路

![image-20230808203741451](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808203741451.png)

-   输入引脚参考引脚定义表格
-   通过**异或**，三个输入引脚有任何一个翻转时。输出引脚产生一次电平翻转（**输入有单数1则为1，双数1为0**）
    -   本设计主要服务于**三相无刷电机**，三个霍尔传感器检测转子的位置，根据转子的位置进行换向
    -   作为无刷电机的接口定时器，驱动换向电路
-   输入滤波器过滤毛刺信号，边沿检测器和外部中断相似
    -   此处的输出可以选择交叉也可以选择各走各的（CH2给1，CH1给2）
    -   灵活**切换**后续输入，可以**一个引脚输入到两个捕获单元**
        -   上捕获周期（上升触发），下捕获站空比（下降触发）
-   捕获瞬间触发CNT写入CCR，同时至flag，可以开启**捕获中断**
-   **总结：上升沿触发捕获，CNT用于计数记时，存在CCR内N，驱动时钟就是f~c~**
    -   捕获后**自动清零用主从触发模式**

#### 细化结构

![image-20230808204757715](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808204757715.png)

-   f~DTS~ 是滤波器采样时钟来源，CCMR1的ICF位控制滤波器参数
    -   **固定频率采样：**连续N个都是高才是高，连续N个低才是低。高频抖动输出不变N越大滤波越强

![image-20230808204246987](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808204246987.png)

-   CC1S可以选择数据选择器，ICPS位配置分频器，CC1E控制使能

### 主从触发模式

![image-20230808211802179](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808211802179.png)

**主模式**：定时器内部信号映射到TRGO引脚，用于触发别的外设
**从模式**：接收其他外设或者自身外设的一些信号，用于控制自身定时器的运行（被别的信号控制）

-   **触发源选择**一个指定的信号，得到TRGI，TRG触发从模式，选择一个操作自动执行

![image-20230808211912895](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808211912895.png)![image-20230808211951059](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808211951059.png)

详细见手册

<img src="C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808212048924.png" alt="image-20230808212048924"  />![image-20230808212131467](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808212131467.png)![image-20230808212139377](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808212139377.png)

### 输入捕获基本逻辑结构图

#### 仅频率

![image-20230808212614816](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808212614816.png)

-   TI1FP1走两条路，**直连通道**和**主从模式触发**信号
-   先读取CNT的值再清零（或者非阻塞的同时）
-   CCR1内就是N，用于计算频率
-   ARR一般设置为65535，CNT同理，**注意信号频率太低会导致溢出**
-   触发源选择只有``TI1FP1``和``TI2FP2``**自动清零只能用通道一二**，其他需要捕获中断手动清零

#### PWMI

![image-20230808213821342](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808213821342.png)

-   **两个通道同时**捕获一个引脚
-   下降沿捕获的CNT是**高点平期间的计数值**，不触发清零
-   **CCR2 / CCR1 就是占空比**

![image-20230808213905399](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808213905399.png)

### 手册

-   14.3.5
-   14.3.6

![image-20230808214019978](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230808214019978.png)

-   14.3.15 主从模式

#### 编程相关

1.   RCC开启时钟，GPIO和TIM时钟打开
2.   GPIO初始化，把GPIO配置输入模式（上拉或者浮空）
3.   配置时基单元，让CNT自增
4.   配置输入捕获单元，滤波器，极性，直连/交叉，分频器
5.   选择从模式触发源TI1FP1
6.   选择触发后的操作RESET
7.   调用TIM_CMD启用定时器
8.   需要读取时，直接读取CCR寄存器，用f~c~ / N计算即可

```c
// @brief 结构体配置输入捕获单元
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);

// @brief 结构体配置输入捕获单元，配置两个通道，快速，配置成PWMI模式
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);

// @brief 给初始化结构体赋予初始值
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);

// @brief 选择输入触发源（从模式的触发源选择）
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);

// @brief 选择输出触发源TRGO（主模式输出的触发源）
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);


// @brief 选择从模式（从模式做什么）
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);

// @brief 分别：配置通道的分频器
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);

// @brief 分别：读取四个通道的CCR
uint16_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture4(TIM_TypeDef* TIMx);

```

#### 性能

-   标准频率 / 计数器最大值 = 最小可以测量的频率
    -   1M / 65535 ~ 15Hz
    -   加大预分频，标准频率变低，可以测量更低的频率
-   最大频率没有明确上线，误差只会越来越大
    -   不要超过标准频率
    -   看误差要求： 标准频率 / 误差要求的10次方值
    -   降低PSC，提高标准频率就可提高上限
    -   考虑测频法
-   误差分析
    -   实际测量还有晶振误差，误差积累会造成问题

## TIM编码器接口Encoder Interface

### 简介

![image-20230809201538701](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809201538701.png)

-   编码器两个输出，A和B，接入STM32的编码器接口，编码器接口自动控制时基单元中的CNT计数器进行++或者--
    -   比如初始化后，CNT为0，每次右转产生脉冲，CNT增加一次， 每次左转产生脉冲，CNT减少一次（可以加减的外部时钟）
-   每隔一段时间取出一次CNT，得到的值就是编码器的速度
    -   **测频法测正交脉冲的频率**

### 正交编码器

正交信号：精度更高，计次频率高，抗噪声

![image-20230809215939600](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809215939600.png)

-   正转时，A相提前B相90度
-   反转时，A相滞后B相90度

![image-20230809220402393](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809220402393.png)![image-20230809220621681](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809220621681.png)

**设计逻辑**就是

-   把A和B的所有边沿作为计数器的计数时钟
-   出现边缘信号时++或者--
-   计数方向由另一项的高低电平决定

### 硬件电路框图

![image-20230809220915513](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809220915513.png)

-   输入部分：要借用输入捕获的两个引脚，CH1和CH2的引脚
-   输出部分：相当于从模式控制器，控制CNT的计数时钟和方向（按照刚才的表）
-   计数时钟和方向都处于编码器接口托管的状态

![image-20230809221438442](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809221438442.png)

#### 框图

![image-20230809221614600](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809221614600.png)

-   编码器接口通过预分频器控制CNT计数器的时钟，同时编码器接口控制计数方向
-   一般设置ARR为65535，最大值
    -   因为65534， 65535， 0， 1， 2的顺序，会被补码直接转换成负数（方便）

### 编码器工作模式表格及案例

-   一开始讲的对应的是第三个模式
-   我们可以考虑忽略一些上升沿，减少增减的频率，仅在A或者B的边沿计数，**但是会损失精度**，计数频率低了
-   正转都是向上计数，反转都是向下计数

![image-20230809223348788](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809223348788.png)

#### 案例1

![image-20230809224057020](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809224057020.png)

-   注意毛刺部分，因为上下沿的逻辑，“一个腿走路，另一条腿不动的时候”，会重复加减加减来回摆动，**计数值不变**

![image-20230809224237716](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809224237716.png)

#### 案例2

-   TI1反向

![image-20230809224439721](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809224439721.png)

-   输入捕获模式下，极性选择，是选择上升沿有效还是下降沿有效的，这里变成了**控制极性了**
-   把TI1高低电平先取反，然后查表

![image-20230809224629788](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230809224629788.png)

### 手册

-   14.3.12

#### 编程相关

1.   开启RCC时钟，开启GPIO和定时器时钟
2.   配置GPIO输入模式
3.   配置时基单元，不分频，自动重装65535
4.   配置输入捕获单元，只有滤波器和极性有效，后面的与编码器无关
5.   配置编码器接口模式
6.   调用TIM_Cmd启动定时器
7.   读取CNT的值来测量位置
8.   每隔一段固定时间取出一次CNT然后清零，测频法测量速度

```c
// @brief 定时器编码器接口设置
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode, uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);

```

## ADC 模数转换

#### 简介

![image-20230810154329499](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810154329499.png)

-   分辨率: 几位：12位就是0 ~ 4095
-   转换频率：开始到结束需要1us，也就是1MHz
-   0-3.3V 线性映射到 0-4095
-   内部信号源是内部温度传感器（CPU温度）和内部参考电压（1.2V基准电压，不变化）
-   普通ADC是：启动，读值，再启动，读值
-   ST的ADC： 列一个组，一次性启动一个组，连续转换多个值
    -   规则组：常规时间
    -   注入组：突发事件
-   **模拟看门狗**可以设定阈值，当AD突破阈值就可以申请中断

### 逐次逼近ADC

ADC0809内部结构图

![image-20230810170424657](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810170424657.png)

-   **模拟信号线路选择器**：从ADDA，ADDB，ADDC选择线路，ALE给锁存信号，选择从IN的其中一路进行输入
-   STM32内部由18个输入通道，18路开关
-   **电路比较器**：发送数据给DAC（内部加权电阻网络，见51单片机），输出模拟电压
    -   DAC输出电压已知，外部输入电压未知
    -   同时输入电路比较器进行**大小判断**
    -   调整DAC知道和外面近似相等
    -   通常使用**二分法**，**逐次逼近**相当于从高位到低二进制位判断是1还是0的方式（判断#位次）
-   START是开始转换，EOC是指示结束，CLK是时钟推动过程
-   Vref是DAC参考电压，同时也决定了ADC的范围（255对应5V还是3.3V呢）
    -   一般VCC和GND都和这两个接在一起

### STM32ADC框图

![image-20230810171146569](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810171146569.png)

-   总共16 + 2个输入通道
-   模拟多路开关指定选择通道，然后进入模数转换器（逐次比较）
-   然后结果被放在数据寄存器内，读取即可
-   参考之前的信息，这里由两种模式的通道，**注入通道和规则通道**
    -   之前说的普通模式是点一个菜，这里说的是一次性点16个菜，老板按照顺序依次做好端上来
    -   **规则组**菜单可以一次性上16个菜（只有**一个**16位寄存器，桌子小，需要配合**DMA**实现（数据转运小帮手，转移菜））
    -   **注入组**比较高级，VIP作为可以全部上，4个寄存器

![image-20230810171615318](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810171615318.png)

-   接下来的是START开始信号相关
    -   可以由软件触发
    -   可以由**硬件触发**TIM的CH，TRGO，因为**ADC一般固定要每隔一段时间转换依次**
        -   比如选择TIM3的更新时间是TRGO输出，然后设置好ADC开始信号对应，就可以自动触发了

![image-20230810171835561](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810171835561.png)![image-20230810171854942](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810171854942.png)

-   ADC的电源和参考电压部分

![image-20230810172008995](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810172008995.png)![image-20230810172134294](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810172134294.png)

-   ADC的时钟部分来源是RCC
-   ADC分频只能选择6分频12M，和8分屏9M

-   DMA请求后面会说

![image-20230810172752308](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810172752308.png)

-   看门狗启动，并且指定看门通道。一旦超过阈值范围，就会开始乱叫，**触发中断**
-   同时EOC和JEOC是两个寄存器的完成flag，可以用使能控制是否申请中断

### 基本结构图

![image-20230810173019176](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810173019176.png)

### 输入通道对应的GPIO

![image-20230810204846280](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810204846280.png)

-   表格内查询``ADC12_INx``（ADC1和ADC2公用引脚）
    -   只有0~9十个外部输入通道
-   ADC2用于双ADC模式，一起工作，配合组成同步和交叉模式（交叉采样，提高频率，类比左右出拳）

### 转换模式（转换，扫描模式）

#### 单次转换，非扫描模式

-   仅**序列1**有效，简单的选择一个，在序列1的位置指定我们想要转换的通道
-   转换结果放在数据寄存器内，同时EOC标志位至一
-   如果判断EOC转换完了，就可以读结果了
-   然后**再次手动触发请求转换**

![image-20230810210418193](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810210418193.png)

#### 连续转换，非扫描模式

-   仍然只用第一个
-   不同在于在第一次转换后**不需要手动触发**，会自动触发下一轮转换
-   可以直接从数据寄存器读数

![image-20230810210905608](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810210905608.png)

#### 单词转换，扫描模式

-   也是单次转换，每触发一次结束后都会停下来，**需要手动触发下一次**
-   扫描模式，相当于允许菜单点菜了，通道可以任意指定（需要设定通道数目的参数）
-   每次触发后**依次**对N个位置进行转换
-   结果都在数据寄存器内
-   **需要DMA把数据挪走**
-   全部N个完成后产生EOC信号，转换结束
-   然后触发下一次

![image-20230810210927241](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810210927241.png)

#### 连续转换，扫描模式

-   和上面类似，只不过立刻开始下一次的转换
-   类似套路

![image-20230810211017139](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810211017139.png)

#### 间断模式

-   每隔几个扫描，暂停一下，需要再触发一次

### 触发控制（何时启动转换）

![image-20230810211154208](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810211154208.png)

### 数据对齐

-   因为是12位，我们有16位，考虑左右对齐

![image-20230810211238813](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810211238813.png)

-   一般使用右对齐，直接就是转换结果
-   左对齐直接大16倍
    -   如果**不需要高分辨率**可以直接拿高8位，编程8位ADC

### 转换时间

-   只有在需要高频率才会考虑
-   厨子做菜需要一段时间才能上菜
-   采样开关会打开，然后断开，进行AD转换，防止电压变化导致无法定位
    -   这就是**采样保持时间**

![image-20230810211708319](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810211708319.png)

### 校准

![image-20230810211813732](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810211813732.png)

-   固定的，加几条代码即可

### 外围电路

![image-20230810212439816](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230810212439816.png)

1.   电位器（滑动端），可以输出0 ~ 3.3V的电压
     -   一般要KOhm级别的电阻
2.   下面的是传感器电阻等等，**等效可变电阻**。电阻阻值没法测量，所以和固定电阻**串联分压**，反应电阻值电压的电路
     -   传感器阻值变小，下拉强，电压低
     -   传感器阻值变大，上拉强，电压高
     -   位置可换，一般要求固定电阻和可变电阻差不多
3.   简易电压转换电路：电阻分压，中间的电压是Vin/R1*R2

### 手册内容

-   11

### 编程相关

![image-20230811162539204](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230811162539204.png)

1.   开启RCC时钟，ADC和GPIO时钟
2.   ADCCLK分频器配置
3.   配置GPIO为**模拟输入模式**
4.   配置模拟开关，接入到规则组内
5.   配置ADC转换器，结构体完成
     1.   配置模拟看门狗，用ITConfig开启对应的中断输出
     2.   NVIC配置优先级
6.   开关控制，开启ADC_Cmd
7.   进行校准
8.   函数读取转换结果和触发转换

```c
// @biref 配置ADCCLK分频器的，选择2，4，6，8分频
void RCC_ADCCLKConfig(uint32_t RCC_PCLK2);

// @biref 回复缺省配置
void ADC_DeInit(ADC_TypeDef* ADCx);

// @biref 初始化
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);

// @biref 结构体初始化
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);

// @biref 开关控制
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);

// @biref 开启DMA转运数据，需要转运就要调用这个 
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);

// @biref 中断输出控制，是否通往NVIC
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);

// @biref 复位校准
void ADC_ResetCalibration(ADC_TypeDef* ADCx);

// @biref 获取复位校准状态
// 由软件设置硬件清除，软件置1，开始校准，校准完毕硬件自动清零
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx);

// @biref 开始校准
void ADC_StartCalibration(ADC_TypeDef* ADCx);

// @biref 获取开始校准状态
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx);

// @biref 软件开始转换控制，软件触发转换
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);

// @biref 返回SWSTART的状态，这个东西在转换开始后马上就清零了，和//转换是否结束毫无关系//
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);

// @biref 间断模式：每隔几个通道间断一次
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);

// @biref 间断模式：是否启用
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);

// @biref ADC规则组通道配置：给菜单填写菜品通道
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);

// @biref ADC外部触发转换控制：是否允许外部触发转换
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);

// @biref ADC获取转换值：获取数据寄存器内的结果
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);

// @biref ADC获取双模式转换值：获取数据寄存器内的结果
uint32_t ADC_GetDualModeConversionValue(void);

// @biref 模拟看门狗是否启动
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);

// @biref 配置看门狗阈值
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold, uint16_t LowThreshold);

// @biref 配置看门通道
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);

// @biref 开启内部的那两个通道
void ADC_TempSensorVrefintCmd(FunctionalState NewState);

// @biref 获取标志位状态：获取完成转换标志位：参数给EOC就可以判断EOC了，如果是1那就是结束了
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);

// @biref 清除标志位状态,1是结束，软件清除或者读取ADC_DR清除
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);

// @biref 获取中断状态
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);

// @biref 清除中断挂起位
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);

```

### 其他信息

-   在阈值这方面为了防止抖动可以设计一个上下错开阈值，中间是需要的值（施密特触发器，迟滞比较）
    -   低于下阈值开灯，高于上阈值关灯

![image-20230811202608577](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230811202608577.png)

-   也可以滤波，均值滤波，裁剪分辨率去掉尾数

## DMA 直接储存器获取

![image-20230812230343746](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230812230343746.png)

-   通道就是数据转运的路径，各个通道间互不干扰
-   如果是内部储存器之间的转运，DMA可以直接一股脑全部搞进来，**软件触发**
-   如果是和外设的话，需要遵守外设对应的时机**硬件触发**
    -   比如，ADC转换完成后硬件触发DMA转运

### 存储器映像

**程序储存器FLASH**：启示地址开始后，每个字节依次增长占领不同的地址。终止地址取决于容量。
**系统存储器**：在ROM最后面。Bootloader是出场写好的
**选项字节**：在ROM最后面。存储Flash的读写保护，看门狗等等

**运行内存**：程序中定义变量数组结构体等等，类比内存条
**外设寄存器**： 初始化读写的东西
**内核外设寄存器**： NVIC， SysTICK

![image-20230812230818004](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230812230818004.png)

-   存储器的内容和地址都很重要
-   ALU和控制都统称CPU
-   ROM掉电不丢失，RAM掉电丢失

32位有大约4GB的寻址空间，所以大部分都是灰色的保留区间

![image-20230812232014209](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230812232014209.png)

0地址处写的是“别名到Flash或者系统储存器，取决于BOOT的引脚”

-   所以我们需要把需要执行的程序映射到0地址去。看是从Flash或者系统储存器或SRAM启动

![image-20230812232156733](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230812232156733.png)

![image-20230812232218071](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230812232218071.png)

![image-20230812232236567](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230812232236567.png)

### DMA框图

![image-20230812234534279](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230812234534279.png)

-   除了CPU都可以看成外设
-   寄存器可以被读取和写入，同时可以用于改变硬件接线状态（开关，数据选择器，计数器，数据寄存器，桥梁）
-   **总线矩阵的左边是主动单元**，可以访问寄存器；被动单元只能被读写
    -   DCode是访问Flash（默认只读）的
    -   系统总线访问其他
-   主动单元除了内核CPU就是**DMA总线**（有三根）
-   DMA内部的通道可以分别设置源地址和目标地址
-   **DMA仲裁器**，本质要求分时复用，如果产生冲突，仲裁器根据通道优先级决定谁先
-   **总线仲裁器**（总线矩阵内）DMA和CPU冲突时，暂停CPU（仅一半带宽）
-   **AHB**从设备：DMA自己的寄存器
-   DMA请求：DMA硬件触发源

### DMA基本结构总结

![image-20230812235908413](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230812235908413.png)

-   注意不可以SRAM到Flash和Flash到Flash，因为Flash是只读的
-   外设和存储器的**起始地址**决定数据从哪里来哪里去
    -   如果要存储器到存储器，要把一个存储器的地址放在**外设的起始地址**站点
-   **数据宽度**：指定依次转运多大的数据宽度
    -   Byte			uint8_t
    -   HalfWord    uint16_t
    -   Word          uint32_t
-   **地址自增**：是否去下一个寄存器
    -   外设不要
    -   存储器，需要，**需要往后挪坑**
-   **传输计数器**：需要转运计次？自减寄存器
    -   完成后自增地址会回到**起始地址**，方便下一轮
-   **自动重装器**：决定转运模式，残次还是循环（ADC连续模式）
-   **M2M**：选择是用硬件触发还是软件触发
    -   软件触发：最快速度连续不断触发DMA，直到传输计数器清零（不可以和自动重装复用）**存储器到存储器使用**
    -   硬件触发：ADC串口定时器， 与外设有关，需要特殊时机，达到时机触发DMA
-   DMA_Cmd开启转运
    1.   开关控制
    2.   传输计数器大于0
    3.   触发源头有信号
-   传输寄存器=0而且没有自动重装时，**需要关闭DMA** DIABLE，再填充传输寄存器

### DMA请求

![image-20230813001255779](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230813001255779.png)

-   硬件对应自己不同的 通道；软件随意
-   用硬件库内的ADC_CMD开启。一般只开启一个
-   通道号越小优先级越高

### DMA 数据对齐

![image-20230813001720006](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230813001720006.png)

-   目标宽度大，在前面补0
-   目标宽度小，舍弃多出来的高位

![image-20230813001727983](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230813001727983.png)

### 例：数据转运+DMA

![image-20230813002608160](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230813002608160.png)

-   外设站点地址填写：数组A的首地址
-   存储器站点地址填写：数组B的首地址
-   宽度都是8，**字节传输**
-   方向：**外设到存储器**
-   **所有站点地址要自增**，传输计数器设置7，不要自动重装
-   软件触发
-   使能DMA
-   相当于复制

### 例：ADC扫描模式+DMA



-   我们需要在每个单独的通道完整转换后，进行DMA数据转运，并且目的地地址自增
-   外设地址：ADC_DR
-   存储器地址：SRAM内定义数组ADValue作为地址
-   宽度都是16，半字节传输
-   外设地址不自增，存储器自增
-   外设地址向存储器地址
-   计数器为7
-   如果是ADC连续扫描，DMA自动重装
-   触发为硬件触发，**ADC硬件触发**

### 手册

-   10
-   2-存储器和总线架构

### 编程相关

1.   开启DMA时钟(AHB总线)
2.   调用···DMA_Init初始化
     -   起始地址
     -   数据宽度
     -   地址是否自增
     -   方向
     -   传输计数器
     -   是否重装
     -   选择触发源M2M
     -   通道优先级
3.   开关控制DMA_Cmd给指定通道
4.   **如果是硬件触发的话**，记得调用一下xxx_DMACmd开启触发信号的输出
5.   **如果需要中断**可以开启DMA_ITConfig，在NVIC内配置通道写函数即可
6.   **如果需要在转运完成后手动给传输计数器赋值的话**，给DMA失能，赋值计数器，再使能即可

```c
// @brief 恢复缺省配置
void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);

// @brief 初始化
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);

// @brief 结构体初始化
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);

// @brief 使能
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);

// @brief 中断输出使能
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);

// @brief 给传输计数器写数据
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber); 

// @brief 获取传输寄存器的值
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);

// @brief 获取标志位状态
FlagStatus DMA_GetFlagStatus(uint32_t DMAy_FLAG);

// @brief 清除标志位状态
void DMA_ClearFlag(uint32_t DMAy_FLAG);

// @brief 获取中断状态
ITStatus DMA_GetITStatus(uint32_t DMAy_IT);

// @brief 清除中断挂起位
void DMA_ClearITPendingBit(uint32_t DMAy_IT);

// @brief 开启ADC DMA通道
void ADC_DMACmd()

```

## 通信协议

![image-20230814170542114](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230814170542114.png)

-   通信协议类比作弊时事先约定好，咳嗽一声，然后特定动作表示特定的意思

![image-20230814162628285](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230814162628285.png)

-   TX数据发送脚；RX数据接收脚
-   SCL：时钟；SDA：数据
-   SCLK时钟，MOSI主机输出数据脚，MISC主机输入数据脚，CS片选通信对象
-   CAN_H和CAN_L差分数据脚
-   DP，DM差分数据脚

**全双工**：可以同时进行双向通信，一般有两根通讯线，比如一根TX发送一根RX接收（MOSI，MISO）
**半双工**：可以不同时双向通信
**单工**：单向通讯

**同步**：接收方在时钟的指引下进行采样
**异步**：需要约定采样频率，和额外的帧头帧尾进行采样位置对齐

**单端**：引脚要共地
**双端**：依靠差分引脚的电压差传输信号（不需要GND）极大的提高抗干扰特性，性能优异

**点对点**：老师找你去办公室1v1
**多设备**：老师在教室里说话，有一个寻址的过程

### 串口通信

-   I2C和SPI一般用于芯片之间的通信，不会接到电脑上

![image-20230814230433924](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230814230433924.png)

![image-20230814230749657](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230814230749657.png)

-   TX（发送高低电平）和RX（接收高低电平）的电平都是相对于GND的
-   TX，RX，GND必须要接上
-   VCC如果设备都有独立供电就可以的
-   如果需要单工可以只接一对TX，RX

#### 电平标准

![image-20230814231027018](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230814231027018.png)

-   RS232一般用于大型机械，可以有一些波动误差
-   RS485差分信号抗干扰能力强可以通信长达千米

### 串口参数及协议

发送一字节，每一个字节都装在一个数据帧内；每个数据帧包含起始位，数据位，校验位，可奇偶校验位

-   **一共9位代表一个字节，前8位是有效荷载**

**波特率**：双方遵守的通讯速率(码元每秒Baud)；**比特率**：每秒传输的比特数（bps）
二进制中，波特率 = 比特率。
假设规定波特率为1000bps，一秒钟发1000位，每一位的时间就是1ms。

1.   **空闲状态**：默认高电平
2.   **起始位**：**低电平**打破高电平空闲状态，下降沿表示要开始了
3.   **数据位**：低位先行
4.   **校验位**：如果数据出错可以丢弃或者重传
     -   无校验
     -   奇校验：校验位+数据位 一共奇数个1
     -   偶校验：校验位+数据位 一共偶数个1
5.   **停止位**：固定回归**高电平**，为下一次做准备

![image-20230814232257507](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230814232257507.png)

例：发送0x0F = 0000 1111

-   可以考虑CRC校验，更好
-   如果想要软件模拟，设置定时器定时反转端口即可
    -   接收的时候需要外部中断，起始位下降沿触发，对齐采样时钟，采样8/9次

#### 串口时序例

波特率 = 9600； 每一位大约1/9600 = 104us
注意**低位先行**

![image-20230814232812322](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230814232812322.png)
![image-20230814233228251](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230814233228251.png)

![image-20230814233249527](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230814233249527.png)

-   串口的停止位可以设置为1位，1.5位，2位

## USART 外设

![image-20230815105830559](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815105830559.png)

-   同步模式多一个时钟输出，为了兼容别的协议，一般还是异步

**常用配置**：波特率9600/115200，数据位8位，停止位1位，不校验

-   硬件流控制：如果A向B发送，而且发送太快，如果没有硬件流控制B接收不过来就会损失数据；如果有硬件流控制，B可以在准备好时置低电平（没准备好置高电平），A只会在B准备好的时候才会发送数据

### USART框图

-   TX，RX是输入输出的引脚
-   SW_RX，IRDA_OUT，IRDA_IN是智能卡和IrDA的引脚
-   TDR（只写）和RDR（只读）在**程序**上表现为一个寄存器，数据寄存器DR，实际上硬件有两个

![image-20230815111658249](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815111658249.png)

-   **发送移位寄存器**：把数据一位一位移出去，对应串口协议的波形数据位
    -   比如如果写入TDR - 0x55（0101 0101）
    -   硬件检测到写入数据，检查移位寄存器是否在工作，如果没有，立刻把TDR里的送到移位寄存器里准备发送；置**标志位TXE**，发送位空
        -   如果标志位TXE是1，可以在TDR写入下一个数据
    -   发送器控制会控制移位寄存器**向右移位**，把数据输出到TX引脚
    -   数据帧之间可以连续发送

![image-20230815112329867](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815112329867.png)

-   **接收数据寄存器**
    -   数据从RX引脚进入接收移位寄存器，在接收器控制的控制下，一位一位读取RX电平
    -   先放在最高位，然后右移
    -   当一个字节完成转移后，会一下吧被丢尽RDR中（开始接收下一帧数据）
    -   置RXNE表示位，接收数据寄存器非空
    -   检测到即可读走

-   发送加上帧头帧尾，接收剔除帧头帧尾默认电路自动执行

![image-20230815120022939](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815120022939.png)

-   **硬件流控制**
    -   引脚同样是交叉连接
    -   nRTS请求发送，输出脚“我当前能不能接收”
        -   和其他设备连接后
        -   RTS输出低电平：可以接收
        -   RTX输出高电平：不可接收，暂停
    -   nCTS准许发送，接收别人nRTS的信号

![image-20230815120715842](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815120715842.png)

-   **SCLK**用于产生同步的时间信号

    -   配合发送位移寄存器输出
    -   每移位一位，时钟走一个周期，告诉对方我移出一位数据了，是否需要指导接收？
    -   只能输出，单向
    -   用途
        -   兼容别的协议，比如SPI
        -   自适应波特率，接收设备不确定发送设备是什么波特率，软件计算

    ![image-20230815121037838](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815121037838.png)

-   **唤醒单元**：实现串口挂载多设备
    -   一条总线上多个设备，需要寻址
    -   给每个设备设置USART地址
    -   收到对应地址的设备的唤醒单元会工作，否则沉默

![image-20230815121221314](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815121221314.png)

-   **中断输出控制**

    -   重要标志位
        -   TXE发送寄存器空
        -   RXNE接收寄存器非空
    -   配置中断是否能通往NVIC

    ![image-20230815121602584](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815121602584.png)

-   **波特率发生器**
    -   USART1挂载在APB2，使用PCLK2的时钟，72M
    -   其他在APB1， 一般36M
    -   除以USARTDIV分频系数，支持小数点后四位（更加精准）
    -   分频完成还要/16，得到发送器接收器时钟

![image-20230815121648594](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815121648594.png)

引脚要查看定义

### 基本结构图总结

![image-20230815121959413](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815121959413.png)

-   两个标志位都可以申请中断

### 数据帧

![image-20230815131743726](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815131743726.png)

-   每个数据的中间是时钟的上升沿，时钟的频率和数据的速率是一样的
-   接收端可以在**时钟上升沿采样**
-   空闲帧和断开帧都是局域网使用的

![image-20230815132501224](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815132501224.png)

-   可以配置停止位的时长不一样

### 输入的检测和匹配

-   输入的采样率和波特率要一致，而且输入采样的位置要在每一位的正中间，这样才可靠；还要有能力判断噪声，根据噪声置标志位

![image-20230815133618302](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815133618302.png)

-   以波特率的16倍进行采样，一位时间内采样16次
-   最开始空闲状态采样一直是1，某个位置突然收到下降沿，如果没有噪声，马上跟着就是起始位
-   在起始位进行连续16次采样（没有噪声那么都是0）
-   在下降沿后的第3，5，7次进行采样
-   在下降沿后的第8，9，10次进行采样
    -   要求每3位中要有2个0
    -   如果不是都是0，在状态寄存器会置NE（Noise Error）
    -   不合规，不算检测到起始位，忽略前面的数据重新开始捕获下降沿
-   如果通过检测，接收状态从空闲转换为**接收起始位**
-   第8，9，10次正好是**中间**，后面的采样也要在这里

![image-20230815133801884](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815133801884.png)

-   在中间采样
-   如果没有噪声三次都是一样的，如果有噪声按照2：1的规则来，多数获胜，NE也会置1

### 波特率发生器

-   波特率发生器就是分频器
-   分为整数和小数部分

![image-20230815134013817](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815134013817.png)

-   注意是2或1，然后要除以16，作为上一段的**采样时钟**，才说过的
-   所以输入时钟/DIV = 16倍的波特率

![image-20230815134207887](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815134207887.png)

假设计算BRR的值，考虑波特率9600

-   整数前面多的补0，小数后面补0

### 手册（CH340G模块）

![image-20230815135108588](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815135108588.png)

-   USB端口
    -   GND， VCC（5V）
    -   D+，D-（通信线）
    -   USB协议
-   CH340芯片转换成串口协议（TXD RXD）

![image-20230815135141980](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815135141980.png)

-   分别有5V和3.3V的输出，从稳压电路输出
-   第五脚通向CH340的电源输入脚，条线帽插在**45**或者56脚上
-   跳线帽选择通信电平+供电

![image-20230815135254459](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815135254459.png)

![image-20230815135506032](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815135506032.png)

-   底下都是指示灯，有数据对应的指示灯会亮

### 手册

-   25

### 编程相关

![image-20230815121959413](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815121959413.png)

1.   开始时钟，UART和GPIO
2.   GPIO初始化（不配置下拉输入），TX配置复用输出，RX配置输入
3.   配置USART，结构体
4.   配置接收中断
     -   ITConfig和NVIC
5.   调用发送和接收的函数，或者获取标志位的函数

```c
// @brief 恢复缺省配置
void USART_DeInit(USART_TypeDef* USARTx);

// @brief 初始化
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);

// @brief 结构体初始化
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);

// @brief 配置同步时钟输出
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);

// @brief 时钟输出结构体初始化
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);

// @brief 启用
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);

// @brief 配置中断
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);


void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);

// @brief 地址，唤醒，LIN不用
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);

// @brief 发送数据
// No need to clear TXE flag, next call flag is unset
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);

// @brief 接收数据
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);

// @brief 标志位函数
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);

```

#### 数据模式

![image-20230815225355814](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230815225355814.png)

-   取数字某一位
    -   数字/10^x^ % 10

## 串口数据包

### HEX数据包

 数据包把一个个数据打包起来，方便我们进行多字节的数据通信

-   比如：陀螺仪的数据，x轴一个字节，y一个字节，z一个字节，不断连续循环发送
-   进行**数据分割**
-   可以自己发挥想象力设计，比如把最高位当成标志位
-   我们额外**提供包头包尾，固定包长/可变包长**

![image-20230831150527385](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230831150527385.png)

**问题**：数据和包头包尾重复

-   限制载荷数据的范围，限符
-   如果不能，使用固定长度的数据包，经过几个数据对齐后就不会有问题了（不判断在和载荷数据是不是包头包尾，仅再接受包头包尾时判断包头包尾）
-   增加包头包尾的数量，使其呈现出载荷数据出现不了的状态
    -   比如：FFFE - 数据包 - FDFC

可以只要一个包头

**问题**：固定和可变包长的选择问题

-   如果数据会和包头包尾重复，尽量选择固定包长
-   只要出现包头就开始，只要包尾就结束，非常灵活

**问题**：各种类型的数据转换成数据流

-   指针指向，当字节发送即可

**传输直接，适合模块发送的原始数据（陀螺仪传感器），灵活性不足，同意包头包尾和数据重复冲突**

### 文本数据包

![image-20230831151418256](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230831151418256.png)

**输入输出直观容易理解，比较适合输入指令和人机交互，但是解析效率低**（Hex就是一个字节100， 文本就得是‘1’‘0’‘0’）

### 数据包发送

HEX数据包：直接丢进数组一发完事
字符数据包：用String直接发就完事了

### 数据包接收

**固定包长**

-   每收到一个字节会进入一次中断，在中断内我们可以拿到那一个字节，拿到之后推出
-   因为包头，包尾，数据的处理逻辑不同，程序内需要一个能够**记住不同状态的机制**
-   由此引出**状态机**

![image-20230831152734600](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230831152734600.png)

-   每个变量需要一个变量来标志一下，这里用S
-   如果数据和包头重复，导致判断错误，包尾位置可能不是FE，进入重复等待包尾的状态，直到接收到真正的包尾

**状态机思路**

1.   根据需求确认状态，画圈
2.   考虑状态会在什么时候转移，如何进行转移，画好线和转移条件
3.   根据图进行编程

比如在制作菜单的时候，芯片内部逻辑，什么时候工作

**可变包长**

![image-20230831153104473](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230831153104473.png)
