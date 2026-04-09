# motor_inspection3.1
# 使用过程中务必保证自身与周围人的安全

## 1.环境配置

运行环境Ubuntu22.04

```
sudo apt update
sudo apt install gcc
sudo apt install cmake
```

打开"requirements.txt"文件所在目录
执行

```
pip install -r requirements.txt
```

## 2.编译

在"CMakelists.txt"所在目录中 执行

```
mkdir build
cd build
cmake ..
make
```

## 3.运行

usb to can连接电脑和电机

细红线连接H，细黑线连接L

<img width="1097" height="383" alt="image" src="https://github.com/user-attachments/assets/8ae99aec-3bfb-4142-aa84-1200fd154dd9" />



USB口插入电脑

```
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up
```

```
python3 motor_client.py
```

在新开一个终端

```
cd build
./bin/motor_test
```

## 4.软件使用

### 改变力臂

摆臂末端的小轴可根据需求上下移动，每移动一格力臂增加/减少50mm，力臂最长为500mm。

在程序中设置力臂:

修改"motor_client.py"文件中第59行 (后续该功能会添加到UI中 :cold_sweat: )

<img width="468" height="202" alt="image" src="https://github.com/user-attachments/assets/739e6ae5-3574-4a5b-8a07-57856d276a12" />


将哑铃盘放入力臂末端小轴中，**随后放入限位环，拧紧限位环的螺丝!!!**

**该设备为大负载设备，且稳定性有待验证，在点击"连接"按钮前请确保G型夹锁紧，台架被牢牢固定在桌子上，
摆臂的前方与两侧附近人员已清空，手放在电源关机键，随时准备断电!!!**

**敬畏力学，尊重牛顿!!!**

点击"连接"按钮

打开电源

根据实际放入的哑铃片输入哑铃重量

<img width="269" height="44" alt="image" src="https://github.com/user-attachments/assets/40013166-edab-450c-b725-d1c7c97a5262" />

点击"一键测试"按钮，开始测试，电机将会正转一圈随后反转一圈，**请勿在程序结束前靠近设备或干扰摆臂运行!!!**

**完成测试后若不再使用，务必关闭电源!!!**

"电机反馈扭矩最大值"参数会记录自程序运行后或上次清零后的最大值，若要在下一轮重新记录需手动点击"电机反馈扭矩最大值清零"按钮\
\
\
\
\
有任何问题与建议可与我联系，**使用过程中务必保证自身与周围人的安全**


