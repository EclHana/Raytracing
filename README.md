# Raytracing
基于微表面模型的光线追踪算法
### 项目描述

1. 通过蒙特卡洛随机采样的方法求解渲染方程，通过对光源采样而不是对相交点的半球空间采样方式加速有效光路的构建
2. BVH（包围体积层次结构）算法来构建二叉树包围盒求交的加速结构，三角形与光线求交算法，球面与光线求交算法
3. 微表面模型，通过对Schlick’s近似菲涅尔项F，法线的统计分布D，几何遮蔽项G的求解来完成微表面模型
4. 多线程并发渲染，对每一行的framebuffer分配线程进行渲染，通过互斥锁和原子变量控制渲染进度提交
5. 通过RussianRoulette方法使无限递归的光路构建过程依概率收敛，并根据期望得到正确的辐照度
