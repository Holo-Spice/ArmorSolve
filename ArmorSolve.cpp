 #include<iostream>


  const double BULLET_SPEED = 25.0;
  const double TIME_OFFSET = 0.15;
  const double PITCH_OFFSET = 0.02;
  const double ARMOR_YAW_LIMIT = 0.6;
  const double ARMOR_YAW_LIMIT_OFFSET = 0.22;

  double ALLOW_ERROR_DISTANCE = 0.06;

  double gimbal_command_pitch_ = 0.0;
  double gimbal_command_yaw_ = 0.0;
  double control_status = 0.0;

  double gimbal_pitch_ = 0.0;
  double gimbal_yaw_ = 0.0;

 void ArmorSolve(){

      if (target_->tracking) {
        double xc = target_->position.x;
        double yc = target_->position.y;
        double z = target_->position.z;
        double armor_yaw = target_->yaw;
        double v_yaw = target_->v_yaw;
        double r1 = target_->radius_1;
        double r2 = target_->radius_2;

        // Resolve current tracking armor
        double xa = xc - r1 * cos(armor_yaw);
        double ya = yc - r1 * sin(armor_yaw);
        // Calculate latency
        double latency = (this->now() - target_->header.stamp).seconds();
        // Calculate bullet fly time
        double distance = sqrt(xa * xa + ya * ya);
        double horizontal_speed = BULLET_SPEED * cos(gimbal_pitch_);
        double fly_time = distance / horizontal_speed + latency + TIME_OFFSET;

        // Calculate target position after fly_time
        double pre_xc = xc + target_->velocity.x * fly_time;
        double pre_yc = yc + target_->velocity.y * fly_time;
        double pre_yaw = armor_yaw + target_->v_yaw * fly_time;

        double final_x = 0, final_y = 0, final_z = 0;
        control_status = 0;
        if (abs(v_yaw) < 1.5) {
          // 不选板，瞄准当前跟踪装甲板
          final_x = pre_xc - r1 * cos(pre_yaw);
          final_y = pre_yc - r1 * sin(pre_yaw);
          final_z = z;
          control_status = 2;
        } else {
          // 选择最优板
          double a_n = target_->armors_num;
          bool is_current_pair = true;
          double r = 0;
          double center_yaw = atan2(pre_yc, pre_xc);
          for (size_t i = 0; i < a_n; i++) {
            double tmp_yaw = pre_yaw + i * (2 * M_fPI / a_n);
            double yaw_diff =
                angles::shortest_angular_distance(tmp_yaw, center_yaw);
            double yaw_diff_offset = signbit(v_yaw) ? -ARMOR_YAW_LIMIT_OFFSET : ARMOR_YAW_LIMIT_OFFSET;
            if (-ARMOR_YAW_LIMIT + yaw_diff_offset < yaw_diff &&
                yaw_diff < ARMOR_YAW_LIMIT + yaw_diff_offset) {
              // Only 4 armors has 2 radius and height
              if (a_n == 4) {
                r = is_current_pair ? r1 : r2;
                final_z = z + (is_current_pair ? 0 : target_->dz);
              } else {
                r = r1;
                final_z = z;
              }
              final_x = pre_xc - r * cos(tmp_yaw);
              final_y = pre_yc - r * sin(tmp_yaw);
              control_status = 1;
              break;
            }
            is_current_pair = !is_current_pair;
          }
        }

        if (control_status != 0) {
          // Calculate gimbal command
          double pitch;
          solver_.solve(distance, final_z, pitch);
          gimbal_command_pitch_ = -pitch;
          gimbal_commandm_yaw_ = atan2(final_y, final_x);

          if (control_status == 1) {
            // Fire control
            ALLOW_ERROR_DISTANCE = (target_->armors_num == 2 || target_->id == "1") ? 0.04 : 0.01;
            ALLOW_ERROR_DISTANCE = target_->id == "outpost" ? 0.01 :  ALLOW_ERROR_DISTANCE;
            double allow_error_angle = ALLOW_ERROR_DISTANCE / distance;
            if (abs(angles::shortest_angular_distance(
                    gimbal_command_yaw_, gimbal_yaw_)) < allow_error_angle) {
              control_status = 2;
            }
          }
        }
        } else {
        control_status = 0;
      }
      }
 }