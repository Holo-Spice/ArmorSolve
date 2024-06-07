# SolveTrajectory

自瞄装甲板解算电控端代码(哨兵版)
包含弹道模型及其解算

### autoSolveTrajectory函数
根据最优决策得出被击打装甲板 自动解算弹道

装甲板id顺序，以四块装甲板为例，逆时针编号
$~~~~~$2
3$~~~~~~~~$1
$~~~~~$0

参数
- param pitch:rad  传出pitch
- param yaw:rad    传出yaw
- param aim_x:传出aim_x  打击目标的x
- param aim_y:传出aim_y  打击目标的y
- param aim_z:传出aim_z  打击目标的z

传出的aim坐标用于视觉端进行可视化击打点坐标

# ArmorSolve.cpp为另一版解算以及火控代码
此版为2024超级对抗赛生死战前一夜改的，未经过实战测试，但感觉更合理

建议在考虑延迟时加上数据的处理、传输、解算延迟
此版为上位机版本，故应用在电控端时注意修改