/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算


#include <math.h>
#include <stdio.h>

#include "SolveTrajectory.h"
#include "GafProjectileSolver.h"

struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.12f; // 飞行时间


void st_Data_Iint(void)
{
    
    //定义参数
    st.k = 0.092;//0.092
    st.bullet_type =  BULLET_17;
    st.current_v = 28;
    st.current_pitch = 0;
    st.current_yaw = 0;
    st.xw = 3.0;
    // st.yw = 0.0159;
    st.yw = 0;
    // st.zw = -0.2898;
    st.zw = 1.5;

    st.vxw = 0;
    st.vyw = 0;
    st.vzw = 0;
    st.v_yaw = 0;
    st.tar_yaw = 0.09131;
    st.r1 = 0.5;
    st.r2 = 0.5;
    st.dz = 0.1;
    st.bias_time = 100;//100
    st.s_bias = 0.19133;
    st.z_bias = 0.21265;
    st.armor_id = ARMOR_INFANTRY3;
    st.armor_num = ARMOR_NUM_NORMAL;
}

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
    if(t < 0)
    {
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
//        printf("[WRAN]: Exceeding the maximum range!\n");
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
//    printf("model %f %f\n", t, z);
    return z;
}


/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float completeAirResistanceModel(float s, float v, float angle)
{

    return 0;

}



/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
//        printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
//            i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
    float timeDelay = 0;
    static float angle,s;

    s = sqrt((st.xw) * (st.xw) + (st.yw) * (st.yw)) - st.s_bias;
    angle = atan2(st.zw + st.z_bias, s);
    t = (float)((exp(st.k * s )  - 1) / (st.k * st.current_v * cos(angle)));

    timeDelay = st.bias_time/1000.0f + t;
    st.tar_yaw += st.v_yaw * timeDelay;

    //计算车辆中心yaw
    float center_yaw = (float)(atan2(st.yw,st.xw));

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 0;
	int i = 0;
    int idx = 0; // 选择的装甲板
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) 
        {
            float tmp_yaw = st.tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //2.计算距离最近的装甲板
        float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        int idx = 0;
        for (i = 1; i<2; i++)
        {
        	float temp_dis_diff = sqrt(tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y);
        	if (temp_dis_diff < dis_diff_min)
        	{
        		dis_diff_min = temp_dis_diff;
        		idx = i;
        	}
        }

        *aim_z = tar_position[idx].z + st.vzw * timeDelay;
        *aim_x = tar_position[idx].x + st.vxw * timeDelay;
        *aim_y = tar_position[idx].y + st.vyw * timeDelay;
            
    }
    else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }
       //计算距离最近的装甲板
        float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        int idx = 0;
        for (i = 1; i<3; i++)
        {
        	float temp_dis_diff = sqrt(tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y);
        	if (temp_dis_diff < dis_diff_min)
        	{
        		dis_diff_min = temp_dis_diff;
        		idx = i;
        	}
        }

        *aim_z = tar_position[idx].z + st.vzw * timeDelay;
        *aim_x = tar_position[idx].x + st.vxw * timeDelay;
        *aim_y = tar_position[idx].y + st.vyw * timeDelay;
    } 
    else {
        for (i = 0; i<4; i++) {
            float tmp_yaw = st.tar_yaw + i * PI/2.0;
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw - st.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }
        //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板
            //3.先排除被遮挡的装甲板 再选取yaw最小的装甲板

            //计算距离最近的装甲板
//        	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
//        	int idx = 0;
//        	for (i = 1; i<4; i++)
//        	{
//        		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
//        		if (temp_dis_diff < dis_diff_min)
//        		{
//        			dis_diff_min = temp_dis_diff;
//        			idx = i;
//        		}
//        	}
        

            //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//        for (i = 1; i<4; i++) {
//            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }
        //先排除被遮挡的装甲板 再选取yaw最小的装甲板
        float possible_dis = st.xw;        
        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
        for (i = 1; i<4; i++)
        {
            if( tar_position[i].x > possible_dis) continue;
            else{
                float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
                if (temp_yaw_diff < yaw_diff_min)
                {
                    yaw_diff_min = temp_yaw_diff;
                    idx = i;
                }
            }
        }
          *aim_z = tar_position[idx].z + st.vzw * timeDelay;
          *aim_x = tar_position[idx].x + st.vxw * timeDelay;
          *aim_y = tar_position[idx].y + st.vyw * timeDelay;
         //瞄准中心 判断开火时间
//        *aim_x = st.xw - cos(center_yaw);
//        *aim_y = st.yw - sin(center_yaw);
//        float aim_dis = sqrt(pow(*aim_x,2)+pow(*aim_y,2));
//        
//        float possible_dis = sqrt(st.xw*st.xw+st.yw*st.yw);        
//        float dis_diff_min = fabsf(aim_dis - sqrt(pow(tar_position[0].x,2)+pow(tar_position[0].y,2)));
//        for (i = 1; i<4; i++)
//        {
//            float temp_dis = sqrt(pow(tar_position[i].x,2)+pow(tar_position[i].y,2));
//            if( temp_dis > possible_dis) continue;
//            else{
//                float temp_dis_diff = fabsf(aim_dis - temp_dis);
//                if (temp_dis_diff < dis_diff_min)
//                {
//                    dis_diff_min = temp_dis_diff;
//                    idx = i;
//                }
//            }
//        }
//        *aim_z = tar_position[idx].z + st.vzw * timeDelay;
//        *aim_x += st.vxw * timeDelay;
//        *aim_y += st.vyw * timeDelay;
      }
        

    //这里符号给错了
    float temp_pitch = 0.0f;
//    temp_pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
//            *aim_z + st.z_bias, st.current_v);
    solver(DEFAULT_VEL, DEFALUT_COEFF, sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias\
            , *aim_z, &temp_pitch);
    if(temp_pitch)
        *pitch = - temp_pitch;
    if(*aim_x || *aim_y)
        *yaw = (float)(atan2(*aim_y, *aim_x));
}

// 从坐标轴正向看向原点，逆时针方向为正

//int main()
//{
//    float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 落点，传回上位机用于可视化
//    float pitch = 0; //输出控制量 pitch绝对角度 弧度
//    float yaw = 0;   //输出控制量 yaw绝对角度 弧度

//    //定义参数
//    st.k = 0.092;
//    st.bullet_type =  BULLET_17;
//    st.current_v = 18;
//    st.current_pitch = 0;
//    st.current_yaw = 0;
//    st.xw = 3.0;
//    // st.yw = 0.0159;
//    st.yw = 0;
//    // st.zw = -0.2898;
//    st.zw = 1.5;

//    st.vxw = 0;
//    st.vyw = 0;
//    st.vzw = 0;
//    st.v_yaw = 0;
//    st.tar_yaw = 0.09131;
//    st.r1 = 0.5;
//    st.r2 = 0.5;
//    st.dz = 0.1;
//    st.bias_time = 100;
//    st.s_bias = 0.19133;
//    st.z_bias = 0.21265;
//    st.armor_id = ARMOR_INFANTRY3;
//    st.armor_num = ARMOR_NUM_NORMAL;


//    autoSolveTrajectory(&pitch, &yaw, &aim_x, &aim_y, &aim_z);


//    printf("main pitch:%f° yaw:%f° ", pitch * 180 / PI, yaw * 180 / PI);
//    printf("\npitch:%frad yaw:%frad aim_x:%f aim_y:%f aim_z:%f", pitch, yaw, aim_x, aim_y, aim_z);

//    return 0;
//}

void SolveDataUnpack(received_packed_t* received_data)
{
    
//    memcpy(&(st.xw), &(received_data->x), sizeof(received_data) - sizeof(received_data->checksum) - sizeof(received_data->header) - 1);
    st.armor_id = received_data->id;
    st.armor_num = received_data->armors_num;
    
    st.xw = received_data->x;
    st.yw = received_data->y;
    st.zw = received_data->z;
    st.tar_yaw = received_data->yaw;
    st.vxw = received_data->vx;
    st.vyw = received_data->vy;
    st.vzw = received_data->vz;
    st.v_yaw = received_data->v_yaw;
    st.r1 = received_data->r1;
    st.r2 = received_data->r2;
    st.dz = received_data->dz;
}

/*
@brief 根据目标距离计算开火阈值
@param temp_range_p:rad  传出pitch阈值
@param temp_range_y:rad    传出yaw阈值
@param x:传入x  打击目标的x
@param y:传入y  打击目标的y
@param z:传入z  打击目标的z
*/
void count_autoshoot_range(float *temp_range_p, float *temp_range_y, float x, float y, float z)
{
    float angle_yaw = 0, angle_pitch = 0;
    float s = sqrt((x * x)+(y * y));
    //识别到大装甲板
    if(st.armor_id == 1 || st.armor_num == ARMOR_NUM_BALANCE)
    {
        angle_yaw = atan2(AM12_LENTH , s); //xz误差待测
        angle_pitch = atan2(AM12_WIDE , s);
    }
    //识别到小装甲板
    else
    {
        angle_yaw = atan2(AM02_LENTH , s);
        angle_pitch = atan2(AM02_WIDE , s);
    } 
      *temp_range_y = angle_yaw * KY + BASICY;
      *temp_range_p = angle_pitch * KP + BASICP;
      if(*temp_range_y < 0.004)
      {
        *temp_range_y= 0.004;
      }
      if(*temp_range_p < 0.005)
      {
        *temp_range_p= 0.005;
      }  
}

