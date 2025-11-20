# FAST-LIO 高频率定位改进

## 概述
本修改为FAST-LIO系统添加了高频率（60Hz）定位功能，通过IMU积分在两次激光雷达测量之间进行状态预测，从而提高定位系统的输出频率。

## 主要改进

### 1. 新增功能
- **高频率IMU积分定时器**: 以60Hz频率运行的IMU积分定时器
- **状态预测算法**: 基于IMU测量值进行运动学积分的状态预测
- **高频率里程计发布**: 60Hz频率的里程计和TF发布
- **模式切换**: 可通过参数在传统雷达频率模式和高频率模式之间切换

### 2. 技术实现

#### 核心算法
```cpp
// IMU积分预测过程
for (每个IMU测量值) {
    // 补偿bias
    acc_unbias = acc_measurement - state.ba;
    gyr_unbias = gyr_measurement - state.bg;
    
    // 姿态更新 (SO3指数映射)
    state.rot = state.rot * SO3::exp(gyr_unbias * dt);
    
    // 速度更新 (世界坐标系)
    acc_world = state.rot * acc_unbias + state.grav;
    state.vel = state.vel + acc_world * dt;
    
    // 位置更新
    state.pos = state.pos + state.vel * dt + 0.5 * acc_world * dt * dt;
}
```

#### 系统架构
1. **主处理循环**: 保持100Hz，处理雷达数据和EKF更新
2. **IMU积分循环**: 60Hz，进行状态预测和高频率发布
3. **状态同步**: 雷达更新后同步IMU积分状态，确保一致性

### 3. 配置参数

在配置文件中添加了新的参数：
```yaml
# 高频率定位模式配置
high_frequency_odom_en: true     # true: 启用60Hz IMU积分定位
                                # false: 使用传统雷达频率定位
```

### 4. 使用方法

#### 启用高频率模式
```bash
# 使用高频率配置文件
ros2 launch fast_lio mapping.launch.py config_file:=mid360_high_freq.yaml

# 或者在现有配置文件中设置参数
high_frequency_odom_en: true
```

#### 禁用高频率模式
```yaml
high_frequency_odom_en: false
```

### 5. 性能特点

#### 优势
- **高频率输出**: 从雷达频率（通常10-20Hz）提升到60Hz
- **更好的动态响应**: 快速运动时的跟踪性能提升
- **平滑的轨迹**: 减少运动过程中的跳跃和不连续
- **兼容性**: 保持与原有系统的完全兼容

#### 注意事项
- **计算负载**: 60Hz的处理会增加一些计算负载
- **IMU质量**: 高频率模式对IMU数据质量要求更高
- **参数调优**: 可能需要根据具体IMU调整噪声参数

### 6. 技术细节

#### 新增类成员变量
```cpp
mutex imu_integration_mutex_;           // IMU积分互斥锁
state_ikfom last_state_point_;         // 上一次状态
double last_imu_integration_time_;     // 上一次IMU积分时间
bool imu_integration_enabled_;         // IMU积分是否启用
bool high_frequency_odom_en_;          // 是否启用高频率里程计模式
```

#### 新增回调函数
- `imu_integration_callback()`: 60Hz IMU积分回调
- `perform_imu_integration()`: IMU积分预测函数
- `publish_high_frequency_odometry()`: 高频率里程计发布

### 7. 配置文件

#### 标准配置
- `mid360.yaml`: 传统模式配置
- `avia.yaml`: 传统模式配置

#### 高频率配置
- `mid360_high_freq.yaml`: 专门为高频率模式优化的配置

### 8. 编译和部署

编译系统：
```bash
cd /path/to/your/workspace
colcon build --packages-select fast_lio
source install/setup.bash
```

### 9. 监控和调试

#### 查看发布频率
```bash
# 监控里程计发布频率
ros2 topic hz /Odometry

# 应该显示接近60Hz的频率
```

#### 性能监控
```bash
# 监控系统负载
top -p $(pgrep fastlio_mapping)
```

### 10. 参数调优建议

#### IMU噪声参数
```yaml
mapping:
    acc_cov: 0.1      # 根据IMU精度调整
    gyr_cov: 0.1      # 根据IMU精度调整
    b_acc_cov: 0.0001 # 加速度bias协方差
    b_gyr_cov: 0.0001 # 陀螺仪bias协方差
```

#### 时间同步
确保IMU和雷达之间的时间同步：
```yaml
common:
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0  # 根据实际校准结果调整
```

## 版本信息
- 基于FAST-LIO原版本进行修改
- 添加了ROS2兼容的高频率定位功能
- 保持向后兼容性

## 作者信息
本修改版本专门针对提高FAST-LIO定位频率的需求开发，适用于需要高频率状态估计的机器人应用场景。
