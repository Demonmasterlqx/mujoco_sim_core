# mujoco_sim_core

## 构建方式

使用 colcon 作为构建工具，支持 python/c++ 等语言

## 架构设计

以 mujoco_core 为核心，mujoco掌管整个仿真进程，包括仿真进度（包括开始仿真/结束仿真），时间 clock 发布。

使用 eCAL 作为ICP方案，~~支持多机分散仿真（幻想）~~

外部控制器（ros2/RTOS/裸机等） 通过 adaptor 接入。

mujoco 会加载全部的模型，然后通过 adaptor 与各个控制器通信。

启动参数传递使用统一的 yaml 文件传参 + 命令行设置。

# todo

+ 增加构建方式介绍/增加依赖安装部分