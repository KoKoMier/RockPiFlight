# Flight-Control
## 基本介绍
这是一个基于c++语言写的飞控程序，主控是用RcokPI，现如今可以实现自平衡，以及室外的gps和气压计定位与定高。
##使用方法
###安装openwrt的sdk
updating......
###Rockpi安装openwrt镜像
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
###


The initial:https://github.com/TSKangetsu
## update
```
git reset --hard origin/master
```
## submodule
```
git submodule update --init --recursive
```
```
git submodule update --remote
```
