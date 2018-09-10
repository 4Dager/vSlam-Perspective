## 4Dager-vSlam-Perspective

<a href="https://v.youku.com/v_show/id_XMzgxMDgzNDQyMA==.html?spm=a2h0j.11185381.listitem_page1.5~A
" target="_blank"><img src="https://vthumb.ykimg.com/054104085B8E65CE000001591301CB29" 
alt="IMAGE ALT TEXT HERE" width="560" height="315" border="0" /></a>  
(Click the image to redirect to the video)

## 安装
### 1. 源码编译
该代码已在macOS Sierra上使用Xcode 8.3.1进行编译，并在iPhone7 Plus上使用iOS 10.2.1进行了测试。

1.1 为macOS安装boost
```
$ ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
$ brew install boost
```
1.2请下载 **opencv2.framework** [点击这里下载](http://uav.ust.hk/storage/opencv2.framework.zip)，然后解压缩到**VINS_ThirdPartyLib/opencv2.framework(请确保您没有为OSX安装opencv)**
 
1.3 在你的Xcode里，选择 **Product**-> **Scheme**-> **Edit Scheme**-> **Run**-> **Info**，set **Build Configuration** to **Release** (not debug)

1.4 **在左上角选择您的设备**，然后在**Main.storyboard中选择您的设备尺寸**，构建并运行

1.5兼容设备和iOS版本要求

	iPhone7 Plus, iPhone7, iPhone6s Plus, iPhone6s, iPad Pro
	iOS 10.2.1 and above


### 2. APP下载
#### 2.1 扫描二维码或通过下列网址进行下载
<img src="https://raw.githubusercontent.com/wiki/4Dager/vSlam-Perspective/2018-09-06 16-59-16屏幕截图.png" 
width="100" height="100" border="0" />
#### 下载地址: <a>https://fir.im/vtyz</a>
#### 2.2 ios_slam app添加信任步骤
##### 第一步，取出苹果手机，打开【设置】-【通用】，如下图所示。
<img src="https://raw.githubusercontent.com/wiki/4Dager/vSlam-Perspective/1.png" 
width="200" height="300" border="0" />
##### 第二步，当我们打开通用菜单后，向下拉，找到【设备管理】，单击进入，如下图所示。
<img src="https://raw.githubusercontent.com/wiki/4Dager/vSlam-Perspective/2.png" 
width="200" height="300" border="0" />
##### 第三步，在进入设备管理菜单后，选择“企业级应用”APP，如下图所示。
<img src="https://raw.githubusercontent.com/wiki/4Dager/vSlam-Perspective/3.png" 
width="200" height="300" border="0" />
##### 第四步，在选择企业级应用APP后，在菜单中间选择信任该APP，如下图所示。
<img src="https://raw.githubusercontent.com/wiki/4Dager/vSlam-Perspective/4.png" 
width="200" height="300" border="0" />
##### 第五步，当我们选择信任企业级APP后，系统便弹出一个对话框，单击“信任”按钮，如下图所示。
<img src="https://raw.githubusercontent.com/wiki/4Dager/vSlam-Perspective/5.png" 
width="200" height="300" border="0" />
##### 第六步，完成设置后，我们可重新打开该APP，查证是否成功设置，如可进入即可，如下图所示。
<img src="https://raw.githubusercontent.com/wiki/4Dager/vSlam-Perspective/6.png" 
width="200" height="300" border="0" />

## 特别鸣谢
[HKUST Aerial Robotics Group](http://uav.ust.hk/)
