#!/usr/bin/env python3
"""
手指模型查看器
用于查看和测试 finger_model_levelB.xml 模型
"""

import mujoco
import mujoco.viewer
import numpy as np
import time


def main():
    # 加载模型
    try:
        model = mujoco.MjModel.from_xml_path(filename='finger_model_levelB.xml')
        data = mujoco.MjData(model)
        print("✅ 模型加载成功")
        print(f"自由度数量: {model.nq}")
        print(f"关节数量: {model.njnt}")
        print(f"执行器数量: {model.nu}")
        print(f"传感器数量: {model.nsensor}")
        print(f"刚体数量: {model.nbody}")
        print(f"几何体数量: {model.ngeom}")
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return

    # 打印关节信息
    print("\n📋 关节信息:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]
        joint_range = model.jnt_range[i] if model.jnt_limited[i] else [-np.inf, np.inf]
        print(f"  {joint_name}: 类型={joint_type}, 范围={joint_range}")

    # 打印执行器信息
    print("\n🎛️ 执行器信息:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        actuator_ctrlrange = model.actuator_ctrlrange[i] if model.actuator_ctrllimited[i] else [-np.inf, np.inf]
        print(f"  {actuator_name}: 控制范围={actuator_ctrlrange}")

    # 仿真参数
    dt = model.opt.timestep
    print(f"\n⏱️ 仿真参数:")
    print(f"  时间步长: {dt}s")
    print(f"  重力: {model.opt.gravity}")

    print("\n🎮 启动交互式查看器...")
    print("  使用鼠标拖拽旋转视角")
    print("  滚轮缩放")
    print("  关闭窗口退出程序")

    # 使用交互式查看器（不需要 mjpython）
    try:
        # 定义控制函数
        def controller(model, data):
            """控制器函数"""
            current_time = data.time

            # 简单的正弦波控制测试
            if model.nu >= 6:  # 确保有足够的执行器
                data.ctrl[0] = 0.5 * np.sin(2 * np.pi * 0.5 * current_time)  # a_servo1
                data.ctrl[1] = 0.3 * np.sin(2 * np.pi * 0.3 * current_time + np.pi / 4)  # a_servo2
                data.ctrl[2] = 0.4 * np.sin(2 * np.pi * 0.4 * current_time + np.pi / 2)  # a_trans1
                data.ctrl[3] = 0.2 * np.sin(2 * np.pi * 0.6 * current_time)  # a_base
                data.ctrl[4] = 0.2 * np.sin(2 * np.pi * 0.7 * current_time + np.pi / 3)  # a_mid
                data.ctrl[5] = 0.1 * np.sin(2 * np.pi * 0.8 * current_time + np.pi / 6)  # a_tip

        # 启动交互式查看器
        mujoco.viewer.launch(model, data, show_left_ui=True, show_right_ui=True)

    except KeyboardInterrupt:
        print("\n🛑 用户中断，退出程序")
    except Exception as e:
        print(f"\n❌ 运行错误: {e}")
    finally:
        print("👋 程序结束")


if __name__ == "__main__":
    main()
