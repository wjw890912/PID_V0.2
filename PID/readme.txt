


QQ:925295580
e-mail:925295580@qq.com
Time:201512
author:王均伟



       SW

   beta---(V0.1)


1、TCP/IP基础协议栈
.支持UDP
.支持TCP
.支持ICMP

2、超轻量级efs文件系统
.支持unix标准文件API
.支持SDHC标准。兼容V1.0和V2.0大容量SDK卡-16G卡无压力。（驱动部分参考开源 :-)）
.超低内存占用，仅用一个512字节的高速缓存来对文集遍历。

3、支持1-write DS18B20 	温度传感器
.支持单总线严格时序
.支持ROM搜索，遍历树叶子，允许一条总线挂接多个温度传感器
.数据自动转换为HTML文件

4、TCP/IP应用
.支持TFTP服务端，可以完成文件的下载任务。（此部分来自GITHUB，增加部分TIMEOUT 事件）tftp -i
.支持NETBIOS服务。
.支持一个TCP服务器，本地端口8080.	测试服务
.支持一个TCP客户端，本地端口40096	远端端口2301 远端IP192.168.0.163	用来做数据交互
.支持一个UDP广播，  本地端口02	    广播地址255.255.255.255	  用来把温度采集的数据发广播
.支持一个HTTP服务器 本地端口80  	http:ip  访问之	  关联2只18B20温度传感器显示在简单的SD卡的网页上

5、系统编译后
Program Size: Code=51264 RO-data=28056 RW-data=1712 ZI-data=55048  

6、网络配置

ipaddr：192, 168, 0, 253-209
netmask：255, 255, 0, 0
gw：192, 168, 0, 20
macaddress：xxxxxxxxxx


tks for GitHUB

	    HW

 stm32f107+DP83848CVV+ds18B20*2+SDHC card (4GB)

7、修改了一个SDcrad的BUG，有一个判断语句写错了，fix后可以支持2GB和4GB以上的两种卡片了。20160122
8、增加了一个宏在main.h中
#define MYSELFBOARD
如果定义了，那么表示使用李伟给我的开发板
如果不定义就选择我自己画的那块板子。
没有什么本质区别，框架一样，只是IO口稍有改动。20160122


 20160123
9、增加一个SMTP应用，可以通过定义USED_SMTP来使能 。
10、完成smtp.c的移植和测试。可以向我的邮箱发送邮件。邮箱需要设置关闭SSL。并且需要修改一下源码 的几个宏定义。
11、把采集的温度数据20分钟发一封邮件到我自己的邮箱中。完成。  必须用通过认真的IP否则过不去防火墙

12、调整了一下发邮箱的时间为5分钟一封邮件，@163邮箱的限制大约是300封邮件


20160402

13、测试中发现有内存泄露的情况,通过增加内存的信息通过TCP输出的 2301端口debug后发现
  异常的如下
 总内存数（字节）：6144
 已用内存（字节）：5816
 剩余内存数（字节）：328
 使用标示：1
 正常的如下
 总内存数（字节）：6144
 已用内存（字节）：20
 剩余内存数（字节）：6124
 使用标示：1
 显然memalloc使内存溢出查找代码因为除了SMTP应用程序使用malloc外其他不具有使用的情况。
 所以肯定是SMTP出问题
 进一步分析代码为SMTP的smtp_send_mail（）中
 smtp_send_mail_alloced(struct smtp_session *s)
 函数使用的
 s = (struct smtp_session *)SMTP_STATE_MALLOC((mem_size_t)mem_len);
 分配了一块内存没有事正常的释放。
 这样反复
 几次最终导致这块应用代码不能正常返回一块完整的 mem_le大小的内存块而一直保留了328字节的剩余内存。
 这最终导致了所有依赖mem的应用程序全部获取不到足够的内存块。而出现的内存溢出。
 继续分析 释放的内存句柄  (struct smtp_session *) s
 发现几处问题

 1）非正常中止	“风险”
   if (smtp_verify(s->to, s->to_len, 0) != ERR_OK) {
    return ERR_ARG;
  }
  if (smtp_verify(s->from, s->from_len, 0) != ERR_OK) {
    return ERR_ARG;
  }
  if (smtp_verify(s->subject, s->subject_len, 0) != ERR_OK) {
    return ERR_ARG;
  }
  由于没有对  smtp_send_mail_alloced 函数进行判断所以如果此处返回会造成函数不能正常中止
  也就会导致 (struct smtp_session *) s	没有机会释放（因为在不正常中止时是在后面处理的）
  但是考虑到源数据是固定的从片上flash中取得的，这种几率几乎没有。但是存在风险。所以统一改为
  if (smtp_verify(s->to, s->to_len, 0) != ERR_OK) {
    	 err = ERR_ARG;
     goto leave;
  }
  if (smtp_verify(s->from, s->from_len, 0) != ERR_OK) {
    	 err = ERR_ARG;
     goto leave;
  }
  if (smtp_verify(s->subject, s->subject_len, 0) != ERR_OK) {
    	 err = ERR_ARG;
     goto leave;
  }

  2）、非正常TCP连接，主要原因。
  原来的函数为：
  if(tcp_bind(pcb, IP_ADDR_ANY, SMTP_PORT)!=ERR_OK)
	{
	return	ERR_USEl;
  	   
	}
  显然还是同样的会造成malloc 分配了但是没有被调用，修改为
  if(tcp_bind(pcb, IP_ADDR_ANY,SMTP_PORT)!=ERR_OK)
  {
	err = ERR_USE;
    goto leave;		   
  }

   这样	  leave中就会自动处理释放掉这个非正常中止的而造成的内存的溢出问题。
   leave:
  smtp_free_struct(s);
  return err;

 归根结底是一个问题。那就是必须保证malloc 和free 成对出现。



 14、NETBIOS 名字服务增加在lwipopts.h中增加
 #define NETBIOS_LWIP_NAME "WJW-BOARD"
 正确的名称
 这样可以使用如下格式找到板子的IP地址
 ping 	wjw-board 
 而不用指定IP地址
 20160410



 /*测试中发现长时间运行后SMTP还有停止不发的情况，内存的问题上面已经解决，下面尝试修改进行解决，并继续测试-见17条*/
  20160427

 15、修改SNMTP的timout超时时间统一为2分钟，因为我的邮件重发时间为3分钟。默认的10分钟太长。先修改之。不是他影响的。fix

 /** TCP poll timeout while sending message body, reset after every
 * successful write. 3 minutes def:(3 * 60 * SMTP_POLL_INTERVAL / 2)*/
#define SMTP_TIMEOUT_DATABLOCK  ( 1 * 60 * SMTP_POLL_INTERVAL / 2)
/** TCP poll timeout while waiting for confirmation after sending the body.
 * 10 minutes def:(10 * 60 * SMTP_POLL_INTERVAL / 2)*/
#define SMTP_TIMEOUT_DATATERM   (1 * 60 * SMTP_POLL_INTERVAL / 2)
/** TCP poll timeout while not sending the body.
 * This is somewhat lower than the RFC states (5 minutes for initial, MAIL
 * and RCPT) but still OK for us here.
 * 2 minutes def:( 2 * 60 * SMTP_POLL_INTERVAL / 2)*/
#define SMTP_TIMEOUT            ( 1 * 60 * SMTP_POLL_INTERVAL / 2)
 20160427
 
  16、增加监控SMTP TCP 部分的变量数组
   smtp_Tcp_count[0]//tcp new count 
    smtp_Tcp_count[1]//bind count
	 smtp_Tcp_count[2]connect count 
	 smtp_Tcp_count[3]bind fail save the all pcb list index number
	 that all use debug long time running on smtp .
 20160427

17、发现不是SMTP的问题似乎邮箱出问题了，重新修改以上15条参数全部为2分钟 ，30*4*0.5S=1MIN *2=2min 

/** TCP poll interval. Unit is 0.5 sec. */
#define SMTP_POLL_INTERVAL      4
/** TCP poll timeout while sending message body, reset after every
 * successful write. 3 minutes def:(3 * 60 * SMTP_POLL_INTERVAL / 2)*/
#define SMTP_TIMEOUT_DATABLOCK  30*2
/** TCP poll timeout while waiting for confirmation after sending the body.
 * 10 minutes def:(10 * 60 * SMTP_POLL_INTERVAL / 2)*/
#define SMTP_TIMEOUT_DATATERM   30*2
/** TCP poll timeout while not sending the body.
 * This is somewhat lower than the RFC states (5 minutes for initial, MAIL
 * and RCPT) but still OK for us here.
 * 2 minutes def:( 2 * 60 * SMTP_POLL_INTERVAL / 2)*/
#define SMTP_TIMEOUT
20160429            30*2
18、加长了KEEPALIVBE时间为
	pcb->so_options |= SOF_KEEPALIVE;
   pcb->keep_idle = 1500+150;// ms
    pcb->keep_intvl = 1500+150;// ms
   pcb->keep_cnt = 2;// report error after 2 KA without response
 20160429


19、增加几个SMTP结果的变量	smtp_Tcp_count[10]	upsize 10 dword

20、增加监控SMTP TCP 部分的变量数组
   smtp_Tcp_count[0]//tcp new count 
    smtp_Tcp_count[1]//bind count
	 smtp_Tcp_count[2]connect count 
	 smtp_Tcp_count[3]bind fail save the all pcb list index number
add smtp send result 
              smtp_Tcp_count[4]|= (smtp_result);  
			  smtp_Tcp_count[4]|= (srv_err<<8);
			  smtp_Tcp_count[4]|= (err<<24);

			  if(err==ERR_OK){smtp_Tcp_count[5]++;}	//smtp成功次数统计

			   if(arg!=(void*)0)
			   {
				smtp_Tcp_count[6]=0xAAAAAAAA ;	 //有参数
			   }
			   else
			   {
			   
			   smtp_Tcp_count[6]=0x55555555 ;	//没有参数
			   }
20160430
21、

 以上测试中发现运行到9天左右就会不再执行SMTP代码返回数据如下： 低字节在前--高字节在后
 【Receive from 192.168.0.253 : 40096】：
5D 11 00 00     5D 11 00 00    58 11 00 00    00 00 00 00     04 00 00 F6  48 11 00 00  55 55 55 55 

上面的数据可知：
tcp new count=0x115d
bind count=0x115d
connect count=0x1158
bind fail  number=0
smtp_result=  4（SMTP_RESULT_ERR_CLOSED）
srv_err=00
tcp err=0xf6是负数需要NOT +1=(-10) 错误代码为  ERR_ABRT	  Connection aborted

以上数据定格，不在变化，说明这个和TCP baind 没有关系，是TCP new和之前的调用问题，所以继续锁定这个问题找。

20160513

22、把速度调快，10秒钟一次SMTP 连接。修改SMTP 应用程序的超时时间为8秒钟同时增加
smtp_Tcp_count[8]++;来计数总的调用SMTP的次数
smtp_Tcp_count[7]++;来计数SMTP 用的TCP new之前的次数。排除一下TCP new的问题！
如果这个变量一直变化而后面的没有变化这证明TCP —new出错。反之再向前推，直到调用它的地方一点点排除。

继续追踪这个停止TCP 连接的问题。
20160513


 /********为了接口陈新的485而做*******************/

23、增加modbus RTU 主机部分底层TXRX的代码，打算使用RTU 和TCP 做成透传485.这边不处理，只转发。
定义了一个宏

//定义了则使用MODBUS RTU TX/RX底层收发 (注意应用层没有使用。因为应用层打算交给服务器做，这边仅仅做RTU透传)
#define USED_MODBUS

18:52调试TX通过。更换了TIM3和USART2的Remap

20160613

24、更新STM32的固件库，使用2011版本的，原因是原来的2009版本的CL系列的串口驱动有问题。波特率不正常。换为2011的正常了，
    MODBUS RTU的流程做了修改。发送屏蔽掉CRC校验的产生，。直接透传。
	注意是MODBUS 这个串口从软件上看也是半双工的	。
20160614

25、上面的24条问题最终结果是晶振问题导致的，和固件库没有关系。


#if !defined  HSE_VALUE
 #ifdef STM32F10X_CL   
  #define HSE_VALUE    ((uint32_t)8000000) /*!< Value of the External oscillator in Hz */
 #else 
  #define HSE_VALUE    ((uint32_t)8000000) /*!< Value of the External oscillator in Hz */
 #endif /* STM32F10X_CL */
#endif /* HSE_VALUE */


  这里	 #define HSE_VALUE    ((uint32_t)8000000) /*!< Value of the External oscillator in Hz */
  要定义为你自己的外部晶振值。
20160615

26、增加了服务器接口和陈新，这一版要准备用在大鹏数据采集上做网管，所以定了简答的协议，这边直观转发。

 IP4_ADDR(&ip_addr,114,215,155,179 );//陈新服务器

 /*服务器发下的协议类型 
协议帧格式
帧头+类型+数据域+\r\n
帧头：
一帧网关和服务器交互的开始同时还担负判读数据上传还是下发的任务。
【XFF+0X55】：表示数据上传到服务器
【0XFF+0XAA】: 表示是数据下发到网关
类型：
0x01:表示土壤温湿度传感器
0x02表示光照传感器
0x03 表示PH 值传感器
0x04 表示氧气含量传感器
数据域
不同的类型的传感器数据域，数据域就是厂家提供的MODBUS-RTU的协议直接搭载上。

 服务器发送：FF AA + 类型 +【modbus RTU 数据域 】+\r\n
 网关回复  ：FF 55 + 类型 +【modbus RTU 数据域 】+\r\n

*/

 27、根据服务器要求修改网关的上报数据

 getid 命令改为

 AB +00 +[三字节ID]+CD +'$'+'\r'+'\n'   9个字节

 修改上报数据位

 FF 55 + 类型（1字节）+ID（3字节）+[modbus-RTU数据域] + '$'+'\r'+'\n'

20180710

28、开启DHCP 
20160710

29、增加PID运算程序和BTA16的过零型SRR/可控硅的驱动，只开启PID--并建立PID分支 V.0
20160712

30、开启TCP debug port 2301 set PID value Kp Ki Kd 和 SP d的值，并打印出温度输入的值和控制输出的百分比
要求发送数据为：【pidset:kp=XX.XX,ki=XX.XX,kd=XX.X,sp=XX.XX,】逗号必须加。 
				全部带小数点，不限制小数位数
20160714

31、增加KEY.C文件增加DS12864.C文件。以支持一个简单的显示界面。

 sample UI and Functions like this ...

1、主界面

	 [整体界面居中]

	当前温度：xxx.xx℃	 （实时显示当前传感器采集的温度）
	设定温度：xxx℃		（显示当前的恒温温度）
	恒温时间：xxx分钟 （实时显示当前所剩余的时间）
	加热状态：（停止）、（运行）


2、设置温度界面	（按SET——temp键进入）



    恒温温度设置

	设置温度：XXX℃	  [居中显示]

	使用SET_TMEMP键进入

	使用  UP、DOWN 键进行上下调整。默认温度为40℃  调整范围为1-99℃

	使用  OK 键确认并退出显示主界面。



3、设置恒温时间界面（按SET-time键进入）


	恒温时间设置
	
	恒温时间：xxx分钟 
	
	使用SET_TIME键进入
	
	使用 UP\DOWN 键进行上下调整。	 默认为20分钟。调整范围为1-200分钟
	
	使用OK键确认并退出显示主界面

 4、开启\关闭  控制
  
   在主界面使用 OK 按键控制系统执行，点击一次运行（一切从初始开始），再点击一次停止运行。


5、按键功能
  
	SET_TMEMP  进入恒温值设定界面
	SET_TIME   进入定时时间设置界面
	UP		   增加数值
	DOWN 	   减少数值
	OK		   确认（子界面）/运行/停止（主界面）


 20160726


 实现菜单程序和显示界面。
 20160729


 增加硬件对应的映射引脚以及配置
20160807


