#!/usr/bin/env python3
"""
测试修复后的手指模型
"""

import mujoco
import numpy as np
import time

def main():
    # 加载修复后的模型
    try:
        model = mujoco.MjModel.from_xml_path('finger_model_fixed.xml')
        data = mujoco.MjData(model)
        print("✅ 修复模型加载成功")
        print(f"关节数量: {model.njnt}")
        print(f"执行器数量: {model.nu}")
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return

    # 打印修复后的关节参数
    print("\n📋 修复后的关节参数:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name is None:
            joint_name = f"joint_{i}"
        joint_damping = model.dof_damping[i]
        joint_armature = model.dof_armature[i]
        print(f"  {joint_name}: 阻尼={joint_damping:.3f}, 惯量={joint_armature:.6f}")

    print("\n🔧 开始稳定性测试...")

    # 阶段1: 零控制稳定性测试
    print("\n阶段1: 零控制稳定性测试（10秒）")
    data.ctrl[:] = 0.0
    
    max_velocities = []
    stability_check = []
    
    for step in range(10000):  # 10秒
        mujoco.mj_step(model, data)
        
        if step % 100 == 0:  # 每0.1秒记录
            max_vel = np.max(np.abs(data.qvel))
            max_velocities.append(max_vel)
            stability_check.append(data.qpos.copy())
            
            if step % 1000 == 0:  # 每1秒打印
                print(f"  时间: {data.time:.1f}s, 最大速度: {max_vel:.6f}")
                for i in range(min(3, model.njnt)):
                    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                    if joint_name is None:
                        joint_name = f"joint_{i}"
                    print(f"    {joint_name}: pos={data.qpos[i]:.4f}, vel={data.qvel[i]:.4f}")

    # 分析稳定性
    final_max_vel = np.max(max_velocities[-50:])  # 最后5秒的最大速度
    print(f"\n零控制稳定性分析:")
    print(f"  最终最大速度: {final_max_vel:.6f}")
    
    if final_max_vel < 0.01:
        print("  ✅ 零控制时系统稳定")
    elif final_max_vel < 0.1:
        print("  ⚠️  零控制时有轻微抖动")
    else:
        print("  ❌ 零控控制时仍有明显抖动")

    # 阶段2: 简单控制测试
    print("\n阶段2: 简单控制测试（10秒）")
    
    control_test_results = []
    for step in range(10000):
        current_time = data.time
        
        # 简单的正弦波控制，只控制舵机
        data.ctrl[0] = 0.3 * np.sin(2 * np.pi * 0.2 * current_time)  # 0.2Hz
        data.ctrl[1] = 0.2 * np.sin(2 * np.pi * 0.15 * current_time + np.pi/4)  # 0.15Hz
        data.ctrl[2] = 0.1 * np.sin(2 * np.pi * 0.1 * current_time)  # 0.1Hz
        
        mujoco.mj_step(model, data)
        
        if step % 200 == 0:  # 每0.2秒记录
            max_vel = np.max(np.abs(data.qvel))
            control_test_results.append({
                'time': current_time,
                'max_vel': max_vel,
                'positions': data.qpos.copy(),
                'velocities': data.qvel.copy(),
                'controls': data.ctrl.copy()
            })
            
            if step % 2000 == 0:  # 每2秒打印
                print(f"  时间: {current_time:.1f}s, 最大速度: {max_vel:.6f}")
                for i in range(min(3, model.njnt)):
                    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                    if joint_name is None:
                        joint_name = f"joint_{i}"
                    vel_abs = abs(data.qvel[i])
                    status = "⚠️" if vel_abs > 2.0 else "✅"
                    print(f"    {status} {joint_name}: pos={data.qpos[i]:.4f}, vel={vel_abs:.4f}")

    # 分析控制测试结果
    control_max_vels = [r['max_vel'] for r in control_test_results]
    avg_control_vel = np.mean(control_max_vels[-25:])  # 最后5秒平均
    max_control_vel = np.max(control_max_vels)
    
    print(f"\n控制测试分析:")
    print(f"  平均最大速度: {avg_control_vel:.6f}")
    print(f"  峰值最大速度: {max_control_vel:.6f}")
    
    if avg_control_vel < 1.0:
        print("  ✅ 控制响应良好，系统稳定")
    elif avg_control_vel < 2.0:
        print("  ⚠️  控制响应可接受，有轻微抖动")
    else:
        print("  ❌ 控制响应过快，系统不稳定")

    # 最终评估
    print(f"\n🎯 修复效果评估:")
    
    if final_max_vel < 0.01 and avg_control_vel < 1.0:
        print("  🎉 修复成功！系统完全稳定")
        print("  建议: 可以尝试逐步降低阻尼，提高响应速度")
    elif final_max_vel < 0.1 and avg_control_vel < 2.0:
        print("  ✅ 修复有效！系统基本稳定")
        print("  建议: 可以微调参数进一步优化")
    else:
        print("  ⚠️  修复部分有效，但仍需调整")
        print("  建议: 进一步增加阻尼或检查mesh文件")

    # 保存测试结果
    print(f"\n📊 测试完成！")
    print(f"  零控制稳定性: {'✅' if final_max_vel < 0.01 else '⚠️' if final_max_vel < 0.1 else '❌'}")
    print(f"  控制响应稳定性: {'✅' if avg_control_vel < 1.0 else '⚠️' if avg_control_vel < 2.0 else '❌'}")

if __name__ == "__main__":
    main()