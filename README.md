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