# pylvxreader
A tool to read .lvx format files for better use of livox lidar

## 简介

pylvxreader是一个用于读取Livox公司旗下激光雷达的特有文件形式.lvx的工具，可以用于读取所有的datatype类型，输入所需处理的文件路径，返回值为读取的点数据（仅包括x, y, z）与IMU数据（包括三个方向的角速度与加速度）。

## 文件

文件包括doc文件夹，其中包含.lvx的文件编码类型；包括image文件夹，内含README.md中的图片；pylvxreader.py为代码文件；sample.py为示例文件，包括了一个使用的简单示例供参考。

## 参考资源

Livox_lvx_parser：[silug109/Livox_lvx_parser: Small repo for someone who need parsed lvx file (github.com)](https://github.com/silug109/Livox_lvx_parser)

pylvx：[Jaesirky/pylvx: lvx文件解析工具 (github.com)](https://github.com/Jaesirky/pylvx)

## 开发人员
[<img src="https://avatars.githubusercontent.com/u/62091780?s=400&u=f754d6233a76cfa8e9d07e3b6f4a495abaf15fd6&v=4" width=100px align="center">](https://github.com/TwSphinx54) 