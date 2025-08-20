#!/usr/bin/env python3
"""
手指模型调试脚本
用于诊断和解决指尖抖动问题
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    # 加载模型
    try:
        model = mujoco.MjModel.from_xml_path('finger_model_levelB.xml')
        data = mujoco.MjData(model)
        print("✅ 模型加载成功")
        print(f"关节数量: {model.njnt}")
        print(f"执行器数量: {model.nu}")
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return

    # 打印关节和执行器详细信息
    print("\n📋 关节详细信息:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name is None:
            joint_name = f"joint_{i}"
        joint_type = model.jnt_type[i]
        joint_range = model.jnt_range[i] if model.jnt_limited[i] else [-np.inf, np.inf]
        joint_damping = model.dof_damping[i]
        joint_armature = model.dof_armature[i]
        print(f"  {joint_name}: 类型={joint_type}, 范围=[{joint_range[0]:.3f}, {joint_range[1]:.3f}], 阻尼={joint_damping:.4f}, 惯量={joint_armature:.6f}")

    print("\n🎛️ 执行器详细信息:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if actuator_name is None:
            actuator_name = f"actuator_{i}"
        if model.actuator_ctrllimited[i]:
            actuator_ctrlrange = model.actuator_ctrlrange[i]
        else:
            actuator_ctrlrange = np.array([-np.inf, np.inf])
        actuator_gain = model.actuator_gainprm[i, 0] if hasattr(model, 'actuator_gainprm') and model.actuator_gainprm[i, 0] > 0 else 50.0
        actuator_biastype = model.actuator_biastype[i]
        print(f"  {actuator_name}: 控制范围=[{actuator_ctrlrange[0]:.3f}, {actuator_ctrlrange[1]:.3f}], 增益={actuator_gain:.1f}, 偏置类型={actuator_biastype}")

    # 创建查看器
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        print("\n🔧 调试模式启动")
        print("阶段1: 零控制测试（所有控制信号设为0）")
        
        # 阶段1: 零控制测试
        start_time = time.time()
        phase1_duration = 5.0
        
        while viewer.is_running() and time.time() - start_time < phase1_duration:
            # 设置所有控制信号为0
            data.ctrl[:] = 0.0
            
            # 仿真步进
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # 打印指尖状态
            if int(data.time * 10) % 10 == 0:  # 每0.1秒打印一次
                tip_joint_idx = -1  # 假设最后一个关节是指尖
                if model.njnt > 0:
                    tip_pos = data.qpos[tip_joint_idx]
                    tip_vel = data.qvel[tip_joint_idx]
                    print(f"时间: {data.time:.1f}s, 指尖位置: {tip_pos:.4f}, 速度: {tip_vel:.4f}")
            
            time.sleep(0.001)

        print(f"\n阶段1完成，检查是否有自激振荡")
        
        # 阶段2: 简单阶跃测试
        print("\n阶段2: 简单阶跃测试")
        start_time = time.time()
        phase2_duration = 5.0
        
        while viewer.is_running() and time.time() - start_time < phase2_duration:
            # 只给第一个执行器一个小的阶跃信号
            data.ctrl[0] = 0.1 if data.time > 2.5 else 0.0
            data.ctrl[1:] = 0.0  # 其他执行器为0
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # 打印状态
            if int(data.time * 5) % 5 == 0:  # 每0.2秒打印一次
                for i in range(min(3, model.njnt)):
                    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                    if joint_name is None:
                        joint_name = f"joint_{i}"
                    print(f"  {joint_name}: pos={data.qpos[i]:.4f}, vel={data.qvel[i]:.4f}")
            
            time.sleep(0.001)

        print(f"\n阶段2完成，检查阶跃响应")
        
        # 阶段3: 降低增益测试
        print("\n阶段3: 降低增益测试")
        start_time = time.time()
        phase3_duration = 10.0
        
        # 临时降低执行器增益（如果支持）
        original_gains = []
        gain_adjusted = False
        if hasattr(model, 'actuator_gainprm'):
            gain_adjusted = True
            for i in range(model.nu):
                original_gains.append(model.actuator_gainprm[i, 0])
                model.actuator_gainprm[i, 0] = 10.0  # 降低到10
        else:
            print("  ⚠️  模型不支持动态调整增益，跳过增益测试")
        
        while viewer.is_running() and time.time() - start_time < phase3_duration:
            # 缓慢的正弦波测试
            current_time = data.time
            data.ctrl[0] = 0.2 * np.sin(2 * np.pi * 0.2 * current_time)  # 很慢的0.2Hz
            data.ctrl[1] = 0.1 * np.sin(2 * np.pi * 0.15 * current_time)
            data.ctrl[2:] = 0.0
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # 打印状态
            if int(current_time * 2) % 2 == 0:  # 每0.5秒打印一次
                print(f"时间: {current_time:.1f}s, 控制信号: [{data.ctrl[0]:.3f}, {data.ctrl[1]:.3f}]")
                for i in range(min(3, model.njnt)):
                    joint_vel = abs(data.qvel[i])
                    if joint_vel > 1.0:  # 速度过大警告
                        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                        print(f"  ⚠️  {joint_name} 速度过大: {joint_vel:.4f}")
            
            time.sleep(0.001)

        # 恢复原始增益
        if gain_adjusted:
            for i in range(model.nu):
                model.actuator_gainprm[i, 0] = original_gains[i]

        print(f"\n调试完成！")
        print("请观察以上三个阶段的表现：")
        print("1. 零控制时是否还有抖动？")
        print("2. 阶跃响应是否稳定？")
        print("3. 降低增益后是否改善？")

if __name__ == "__main__":
    main()