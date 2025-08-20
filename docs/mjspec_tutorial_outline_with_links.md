# MuJoCo mjSpec 教程目录（含跳转链接）

**教程主页：** https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb

## 1. Model Editing（模型编辑）
- **[All imports](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=All_imports)** - 所有必要库的导入
- **[Separate parsing and compiling](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Separate_parsing_and_compiling)** - 解析和编译的分离
  - **[Parse, compile, modify, compile:](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Parse_compile_modify_compile_)** - 解析、编译、修改、重新编译的流程

## 2. Procedural Models（程序化模型生成）

### 2.1 Tree（树模型）
- **[Utilities](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Utilities)** - 树模型生成的工具函数
- **[Tree creation](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Tree_creation)** - 递归创建树结构
- **[Make video](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Make_video)** - 生成树模型动画视频

### 2.2 Height Field（高度场）
- **[Utilities](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Utilities)** - 高度场生成工具
  - **[Perlin noise generator](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Perlin_noise_generator)** - Perlin噪声生成器
  - **[Soft edge slope](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Soft_edge_slope)** - 软边缘斜坡生成
- **[Textured height-field generator](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Textured_height_field_generator)** - 纹理化高度场生成器
- **[Video](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Video)** - 高度场演示视频

### 2.3 Mesh（网格）
- **[Add "rock" mesh](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Add_rock_mesh)** - 添加岩石网格
- **[Video](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Video)** - 网格演示视频

### 2.4 Terrain Generation（地形生成）
- **[Utilities](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Utilities)** - 地形生成工具函数
- **[Stairs](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Stairs)** - 楼梯生成
- **[Debris (Geoms)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Debris_Geoms_)** - 碎片生成（使用几何体）
- **[Debris (Mesh)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Debris_Mesh_)** - 碎片生成（使用网格）
- **[Boxy Terrain](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Boxy_Terrain)** - 方块地形
- **[Box (Extrusion | Cut)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Box_Extrusion_Cut_)** - 盒子（挤出|切割）
- **[Heightfield](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Heightfield)** - 高度场地形
- **[Floating platform](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Floating_platform)** - 悬浮平台
- **[Simple stairs](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Simple_stairs)** - 简单楼梯
- **[Sinusoidal stairs](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Sinusoidal_stairs)** - 正弦楼梯
- **[Floating platform for circular stair](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Floating_platform_for_circular_stair)** - 圆形楼梯的悬浮平台
- **[Circular stairs](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Circular_stairs)** - 圆形楼梯
- **[Tile Generator](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Tile_Generator)** - 地砖生成器
- **[Generate Terrain](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Generate_Terrain)** - 生成完整地形

## 3. Model Editing（模型编辑进阶）

### 3.1 基础操作
- **[Get resources](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Get_resources)** - 获取资源文件
- **[Traversing the spec](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Traversing_the_spec)** - 遍历规范结构
- **[Model re-compilation with state preservation](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Model_re_compilation_with_state_preservation)** - 状态保持的模型重新编译

### 3.2 Humanoid Model（人形模型）
- **[Humanoid model](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Humanoid_model)** - 人形模型基础
- **[Humanoid with arms replaced by legs](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Humanoid_with_arms_replaced_by_legs)** - 用腿替换手臂的人形模型
- **[Humanoid with Franka arm](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Humanoid_with_Franka_arm)** - 装备Franka机械臂的人形模型
- **[Imported actuators](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Imported_actuators)** - 导入的执行器
- **[Humanoid with randomized heads and arm poses](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Humanoid_with_randomized_heads_and_arm_poses)** - 随机化头部和手臂姿态的人形模型

### 3.3 Model Scaling（模型缩放）
- **[Uniformly scale humanoid](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Uniformly_scale_humanoid)** - 均匀缩放人形模型
- **[Scaling actuator forces](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Scaling_actuator_forces)** - 缩放执行器力
- **[Long-limbed humanoid](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Long_limbed_humanoid)** - 长肢人形模型
- **[Meshes](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Meshes)** - 网格缩放

## 4. dm_control example（dm_control示例）
- **[Six Creatures on a floor](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Six_Creatures_on_a_floor)** - 地板上的六种生物
- **[Video of the movement](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Video_of_the_movement)** - 运动视频
- **[Movement trajectories](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb#scrollTo=Movement_trajectories)** - 运动轨迹

---

## 相关资源链接：

### 官方文档
- **[MuJoCo Model Editing 文档](https://www.google.com/url?q=https%3A%2F%2Fmujoco.readthedocs.io%2Fen%2Flatest%2Fprogramming%2Fmodeledit.html)** - C API 模型编辑文档
- **[MuJoCo Python 文档](https://www.google.com/url?q=https%3A%2F%2Fmujoco.readthedocs.io%2Fen%2Flatest%2Fpython.html%23model-editing)** - Python API 模型编辑文档
- **[MuJoCo 入门教程](https://github.com/google-deepmind/mujoco?tab=readme-ov-file#getting-started)** - 基础入门教程

### 参考资源
- **[Perlin Noise 介绍](https://www.youtube.com/watch?v=9x6NvGkxXhU)** - Perlin噪声算法视频教程
- **[dm_control 示例论文](https://www.google.com/url?q=https%3A%2F%2Farxiv.org%2Fabs%2F2006.12983)** - 相关研究论文

---

## 主要技术要点：

### 核心概念
- **mjSpec API** - MuJoCo模型编辑的核心API
- **解析与编译分离** - 允许在解析后进行编辑步骤
- **程序化生成** - 通过代码动态创建复杂模型
- **模型组合** - 将多个模型组合成更大的模型

### 关键技术
- **递归结构创建** - 树、地形等复杂结构的生成
- **Perlin噪声** - 用于自然地形生成
- **模型缩放** - 均匀和非均匀缩放技术
- **状态保持** - 模型重新编译时保持物理状态
- **多模型组合** - 不同模型的拼接和整合

### 应用场景
- **机器人仿真** - 人形机器人、多足生物
- **环境生成** - 地形、障碍物、复杂场景
- **参数化设计** - 可配置的模型生成
- **领域随机化** - 训练数据的多样性生成