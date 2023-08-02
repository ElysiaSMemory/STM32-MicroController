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

### 简化版本

![image-20230802234132010](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802234132010.png)

-   运行控制：控制启动停止，向上向下

![image-20230802234434093](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230802234434093.png)

-   中断输出控制判断是需要什么中断
