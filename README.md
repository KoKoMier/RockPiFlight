# Flight-Control
## 基本介绍
这是一个基于c++语言写的飞控程序，主控是用RcokPI，现如今可以实现自平衡，以及室外的gps和气压计定位与定高。
##使用方法
###安装openwrt的sdk
updating......
###Rockpi安装openwrt镜像
updating......
###遥控器的配置
updating......
###下载完整代码
首先是获取源代码
```
git clone https://github.com/KoKoMier/RockPiFlight.git
```
由于我用了子模块，因此在RockPiFlight的文件下，使用以下指令来获取我写的第三方库
```
git submodule update --init --recursive
```
随后用openwrt的编译器编译源码即可在build中生成RockPiFlight的二进制文件，把二进制文件，以及APMconfig.json通过scp放入到Rockpi中
###代码的使用
首先是校准电机，执行指令
```
./RockPiFlight -E 5inchDefault
```
随后寻找四个电机的ID号，通过执行下面的代码，然后输入电机ID号(0-15)以及pwm的值(1000-2000),用穷举法的方式找到，并把值写入到APMconfig.json中的A1,A2,B1,B2，
其中A1是左上的电机，A2是右上的电机，B1是左下的电机，B2是右下的电机
```
./RockPiFlight -e 5inchDefault
```
然后再执行陀螺仪校准的代码
```
./RockPiFlight -a 5inchDefault
```
最后就可以开始执行无人机飞行的代码了,若是发现无人机抖动，很有可能是MPU6500放的方向与作者不一样，需要修改APMconfig.json中的Sensor里面_flag_MPU_Flip的参数，
从而通过旋转坐标系来实现稳定，最后就可以安装上桨叶实现飞行
```
./RockPiFlight -r 5inchDefault
```
##The initial
https://github.com/TSKangetsu

