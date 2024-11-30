import numpy as np
from scipy.interpolate import CubicSpline
from crazyflie_py import Crazyswarm

# 飞行参数
Z = 1.15                    # 悬停高度
TAKEOFF_DURATION = 2.5    # 起飞时间
points_per_second = 100    # 每秒插值点数量（100Hz）
speed = 1.4              # 飞行速度1.5 m/s
GOTO_DURATION = 1 / points_per_second  # 每个点的飞行时间
# 定义路径关键点：悬停点、A点、B点、C点
hover_point = np.array([0.0, 0.0, 1.15])  # 悬停点
C = np.array([0.9, 1.9, 1.15])  # 控制点 C
A = np.array([2.35, 1.4, 1.15])  # A 点
D = np.array([2.5, 0.0, 1.40])  # 控制点 D
B = np.array([1.3, -0.45, 1.55])  # B 点
landing_hover_point = np.array([0.0, 0.0, 1.0])  # 降落悬停点

def generate_smooth_trajectory(hover_point, A, D, B, C, landing_hover_point, points_per_second, speed):
    """
    生成平滑曲线轨迹，经过新增控制点 C, 动态调整轨迹点数量。
    :param hover_point: 起点
    :param C: 控制点 C
    :param A: 必须经过的点 A
    :param B: 必须经过的点 B
    :param D: 控制点 D
    :param landing_hover_point: 终点
    :param points_per_second: 插值点密度
    :param speed: 飞行速度
    :return: 平滑路径点 (Mx3)
    """
    # 控制点，定义飞行顺序
    control_points = np.array([
        hover_point,
        C,
        A,
        D,
        B,
        landing_hover_point
    ])

    # 提取各个维度的控制点
    t = np.linspace(0, 1, len(control_points))  # 参数化时间
    x = control_points[:, 0]
    y = control_points[:, 1]
    z = control_points[:, 2]

    # 使用 CubicSpline 生成三维轨迹
    spline_x = CubicSpline(t, x, bc_type='clamped')  # Clamped 确保边界条件
    spline_y = CubicSpline(t, y, bc_type='clamped')
    spline_z = CubicSpline(t, z, bc_type='clamped')

    # 使用高密度采样计算路径总弧长
    t_dense = np.linspace(0, 1, 1000)
    dense_points = np.vstack((spline_x(t_dense), spline_y(t_dense), spline_z(t_dense))).T
    segment_lengths = np.linalg.norm(np.diff(dense_points, axis=0), axis=1)
    total_length = np.sum(segment_lengths)

    # 根据路径总弧长和飞行速度计算轨迹点数量
    num_points = int(total_length * points_per_second / speed)

    # 生成平滑轨迹点
    t_fine = np.linspace(0, 1, num_points)
    smooth_x = spline_x(t_fine)
    smooth_y = spline_y(t_fine)
    smooth_z = spline_z(t_fine)

    # 返回平滑轨迹点
    trajectory = np.vstack((smooth_x, smooth_y, smooth_z)).T
    return trajectory

# 生成完整的平滑轨迹
WAYPOINTS = generate_smooth_trajectory(hover_point, A, D, B, C, landing_hover_point, points_per_second, speed)

# 姿态约束
yaw = 0.0  # 保持固定的航向角


def calculate_pitch_roll(position, target, prev_target, dt):
    """
    基于目标点和当前位置计算精确的俯仰和横滚角速度。
    :param position: 当前位置 (x, y, z)
    :param target: 当前目标点 (x, y, z)
    :param prev_target: 前一个目标点，用于计算速度和曲率
    :param dt: 时间间隔（与更新频率一致）
    :return: pitch_rate, roll_rate (俯仰角速度, 横滚角速度)
    """
    # 计算当前位置与目标位置的差向量
    delta_pos = target - position

    # 计算速度向量
    velocity = (target - prev_target) / dt

    # 计算曲率（通过目标点前后的差值逼近）
    delta_tangent = velocity / np.linalg.norm(velocity)
    curvature = np.linalg.norm(delta_tangent)

    # 将速度分量转化为俯仰和横滚控制
    pitch_rate = curvature * velocity[0]  # 基于 X 分量调整俯仰角速度
    roll_rate = curvature * velocity[1]   # 基于 Y 分量调整横滚角速度

    # 返回姿态角速度
    return pitch_rate, roll_rate



def main():
    # 初始化 Crazyflie 群控制
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # 起飞
    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(2.5)

    # 悬停短暂停留
    cf.goTo(goal=cf.initialPosition + hover_point, yaw=0.0, duration=1.0, groupMask=0, relative=False)
    timeHelper.sleep(0.5)

    # 初始化前一目标点
    prev_target = WAYPOINTS[0]

    for i, target_position in enumerate(WAYPOINTS[1:], start=1):
        # 当前目标点
        target_position += cf.initialPosition

        # 计算俯仰角速度和横滚角速度
        dt = GOTO_DURATION
        pitch_rate, roll_rate = calculate_pitch_roll(cf.initialPosition, target_position, prev_target, dt)

        # 发送控制命令
        cf.cmdFullState(
            pos=target_position,
            vel=np.array([0.0, 0.0, 0.0]),
            acc=np.zeros(3),
            yaw=yaw,
            omega=np.array([roll_rate, pitch_rate, 0.0])
        )

        # 更新前一目标点
        prev_target = target_position
        timeHelper.sleep(GOTO_DURATION)

    # 发送控制命令
    cf.cmdFullState(
    pos=np.array([0.11, 0.03, 1.1]),
    vel=np.array([0.0, 0.0, 0.0]),
    acc=np.zeros(3),
    yaw=yaw,
    omega=np.array([roll_rate, pitch_rate, 0.0])
    )
    timeHelper.sleep(1)
    # 降落
    cf.land(targetHeight=0.0, duration=4.0)
    timeHelper.sleep(4.0)

if __name__ == "__main__":
    main()
