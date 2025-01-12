# rm_vision
<img src="image/rm_vision.svg" alt="rm_vision" width="200" height="200">

## Overview

rm_vision 项目旨在为 RoboMaster 队伍提供一个规范、易用、鲁棒、高性能的视觉框架方案，为 RM 开源生态的建设添砖加瓦

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

[![Build Status](https://github.com/chenjunnn/rm_vision/actions/workflows/ci.yml/badge.svg)](https://github.com/chenjunnn/rm_vision/actions/workflows/ci.yml)


该项目为 [rm_vision](https://github.com/chenjunnn/rm_vision) 适配的弹道解算模块

### License

The source code is released under a [MIT license](rm_auto_aim/LICENSE).

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Author: Chen Jun

# SolveTrajectory

自瞄装甲板解算电控端代码(哨兵版)
包含弹道模型及其解算

### autoSolveTrajectory函数
根据最优决策得出被击打装甲板 自动解算弹道

装甲板id顺序，以四块装甲板为例，逆时针编号
| ![](/image/1.png) |
| :---------------: |
|    装甲板编号     |

参数
- param pitch:rad  传出pitch
- param yaw:rad    传出yaw
- param aim_x:传出aim_x  打击目标的x
- param aim_y:传出aim_y  打击目标的y
- param aim_z:传出aim_z  打击目标的z

传出的aim坐标用于视觉端进行可视化击打点坐标

# ArmorSolve.cpp为另一版解算以及火控代码

