#!/usr/bin/env python3
"""
简化版手指模型调试脚本
使用Renderer代替viewer，避免macOS的mjpython要求
"""

import mujoco
import numpy as np
import time
import matplotlib.pyplot as plt

def main():
    # 加载模型
    try:
        model = mujoco.MjModel.from_xml_path('finger_model_levelB.xml')
        data = mujoco.MjData(model)
        print("✅ 模型加载成功")
        print(f"关节数量: {model.njnt}")
        print(f"执行器数量: {model.nu}")
        print(f"自由度数量: {model.nv}")
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return

    # 打印关节详细信息
    print("\n📋 关节详细信息:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name is None:
            joint_name = f"joint_{i}"
        joint_type = model.jnt_type[i]
        if model.jnt_limited[i]:
            joint_range = model.jnt_range[i]
            print(f"  {joint_name}: 类型={joint_type}, 范围=[{joint_range[0]:.3f}, {joint_range[1]:.3f}]")
        else:
            print(f"  {joint_name}: 类型={joint_type}, 范围=无限制")

    # 打印执行器详细信息
    print("\n🎛️ 执行器详细信息:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if actuator_name is None:
            actuator_name = f"actuator_{i}"
        if model.actuator_ctrllimited[i]:
            ctrl_range = model.actuator_ctrlrange[i]
            print(f"  {actuator_name}: 控制范围=[{ctrl_range[0]:.3f}, {ctrl_range[1]:.3f}]")
        else:
            print(f"  {actuator_name}: 控制范围=无限制")

    # 数据记录
    times = []
    positions = []
    velocities = []
    controls = []

    print("\n🔧 开始调试仿真...")

    # 阶段1: 零控制测试
    print("\n阶段1: 零控制测试（5秒）")
    data.ctrl[:] = 0.0
    
    for step in range(5000):  # 5秒，1ms步长
        mujoco.mj_step(model, data)
        
        if step % 100 == 0:  # 每0.1秒记录一次
            times.append(data.time)
            positions.append(data.qpos.copy())
            velocities.append(data.qvel.copy())
            controls.append(data.ctrl.copy())
            
            if step % 1000 == 0:  # 每1秒打印一次
                print(f"  时间: {data.time:.1f}s")
                for i in range(min(3, model.njnt)):
                    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                    if joint_name is None:
                        joint_name = f"joint_{i}"
                    print(f"    {joint_name}: pos={data.qpos[i]:.4f}, vel={data.qvel[i]:.4f}")

    # 检查零控制时的稳定性
    max_vel_zero = np.max(np.abs(np.array(velocities[-100:])))  # 最后1秒的最大速度
    print(f"\n零控制时最大速度: {max_vel_zero:.6f}")
    if max_vel_zero > 0.01:
        print("⚠️  零控制时仍有明显运动，可能是物理参数问题")
    else:
        print("✅ 零控制时系统稳定")

    # 阶段2: 阶跃测试
    print("\n阶段2: 阶跃测试（5秒）")
    step_start_time = len(times)
    
    for step in range(5000):
        current_time = data.time
        
        # 在2.5秒时给第一个执行器阶跃信号
        if current_time > 7.5:  # 2.5 + 5 (阶段1时间)
            data.ctrl[0] = 0.1
        else:
            data.ctrl[0] = 0.0
        data.ctrl[1:] = 0.0
        
        mujoco.mj_step(model, data)
        
        if step % 100 == 0:
            times.append(data.time)
            positions.append(data.qpos.copy())
            velocities.append(data.qvel.copy())
            controls.append(data.ctrl.copy())
            
            if step % 1000 == 0:
                print(f"  时间: {data.time:.1f}s, 控制信号: {data.ctrl[0]:.3f}")
                for i in range(min(3, model.njnt)):
                    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                    if joint_name is None:
                        joint_name = f"joint_{i}"
                    print(f"    {joint_name}: pos={data.qpos[i]:.4f}, vel={data.qvel[i]:.4f}")

    # 阶段3: 低频正弦波测试
    print("\n阶段3: 低频正弦波测试（10秒）")
    
    for step in range(10000):
        current_time = data.time
        
        # 低频正弦波
        data.ctrl[0] = 0.2 * np.sin(2 * np.pi * 0.1 * current_time)  # 0.1Hz
        data.ctrl[1] = 0.1 * np.sin(2 * np.pi * 0.15 * current_time)  # 0.15Hz
        data.ctrl[2:] = 0.0
        
        mujoco.mj_step(model, data)
        
        if step % 100 == 0:
            times.append(data.time)
            positions.append(data.qpos.copy())
            velocities.append(data.qvel.copy())
            controls.append(data.ctrl.copy())
            
            if step % 2000 == 0:
                print(f"  时间: {data.time:.1f}s")
                for i in range(min(3, model.njnt)):
                    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                    if joint_name is None:
                        joint_name = f"joint_{i}"
                    vel_abs = abs(data.qvel[i])
                    status = "⚠️" if vel_abs > 1.0 else "✅"
                    print(f"    {status} {joint_name}: pos={data.qpos[i]:.4f}, vel={vel_abs:.4f}")

    # 转换为numpy数组
    times = np.array(times)
    positions = np.array(positions)
    velocities = np.array(velocities)
    controls = np.array(controls)

    # 分析结果
    print("\n📊 分析结果:")
    
    # 找出指尖关节（通常是最后一个）
    tip_joint_idx = model.njnt - 1
    tip_joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, tip_joint_idx)
    if tip_joint_name is None:
        tip_joint_name = f"joint_{tip_joint_idx}"
    
    tip_velocities = velocities[:, tip_joint_idx]
    max_tip_vel = np.max(np.abs(tip_velocities))
    print(f"指尖关节 ({tip_joint_name}) 最大速度: {max_tip_vel:.4f}")
    
    if max_tip_vel > 2.0:
        print("❌ 指尖速度过大，存在抖动问题")
    elif max_tip_vel > 1.0:
        print("⚠️  指尖速度偏大，可能有轻微抖动")
    else:
        print("✅ 指尖速度正常")

    # 绘制图表
    print("\n📈 绘制分析图表...")
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # 位置图
    axes[0].plot(times, positions[:, tip_joint_idx], 'b-', label=f'{tip_joint_name} 位置')
    axes[0].set_ylabel('位置 (rad)')
    axes[0].set_title('指尖关节位置')
    axes[0].grid(True)
    axes[0].legend()
    
    # 速度图
    axes[1].plot(times, tip_velocities, 'r-', label=f'{tip_joint_name} 速度')
    axes[1].set_ylabel('速度 (rad/s)')
    axes[1].set_title('指尖关节速度')
    axes[1].grid(True)
    axes[1].legend()
    
    # 控制信号图
    axes[2].plot(times, controls[:, 0], 'g-', label='控制信号 0')
    if model.nu > 1:
        axes[2].plot(times, controls[:, 1], 'm-', label='控制信号 1')
    axes[2].set_ylabel('控制信号')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_title('控制信号')
    axes[2].grid(True)
    axes[2].legend()
    
    plt.tight_layout()
    plt.savefig('finger_debug_analysis.png', dpi=150, bbox_inches='tight')
    print("✅ 分析图表已保存为: finger_debug_analysis.png")
    
    # 显示图表
    plt.show()

    print("\n🎯 诊断建议:")
    if max_tip_vel > 2.0:
        print("1. 尝试增加关节阻尼 (damping)")
        print("2. 降低执行器增益 (kp)")
        print("3. 检查mesh文件是否有问题")
    elif max_tip_vel > 1.0:
        print("1. 适当增加阻尼或降低增益")
        print("2. 检查控制信号是否合理")
    else:
        print("1. 系统表现正常")
        print("2. 如果仍有视觉抖动，可能是渲染问题")

if __name__ == "__main__":
    main()