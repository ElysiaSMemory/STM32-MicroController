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
```

![image-20230731163707272](C:/Users/24962/AppData/Roaming/Typora/typora-user-images/image-20230731163707272.png)

