import pandas as pd
import matplotlib
matplotlib.use('TkAgg')  # 或者 'Qt5Agg'
import matplotlib.image as mpimg
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams['font.family'] = 'Times New Roman'
mpl.rcParams['font.size'] = 15
import math
from matplotlib.patches import Polygon
import time

def read_experiment_data(target_path, hunter_path, obstacle_path):
    """读取目标、围捕机器人和障碍物的位置数据"""
    # 读取目标数据（注意Target2可能存在缺失值）
    target_df = pd.read_csv(
        target_path,
        sep=r'\s+',  # 按空格分隔（支持多个空格）
        engine='python'
    )

    # 读取围捕机器人数据（3个机器人）
    hunter_df = pd.read_csv(
        hunter_path,
        sep=r'\s+',
        engine='python'
    )

    # 读取障碍物数据（3个障碍物）
    obstacle_df = pd.read_csv(
        obstacle_path,
        sep=r'\s+',
        engine='python'
    )

    # 验证数据完整性（确保有300个时间步）
    assert len(target_df) == 450, "目标数据应包含300个时间步"
    assert len(hunter_df) == 450, "围捕机器人数据应包含300个时间步"
    assert len(obstacle_df) == 450, "障碍物数据应包含300个时间步"

    return target_df, hunter_df, obstacle_df


RobotNum = 4  # 围捕机器人数量（3个围捕机器人 + 1个目标）
target_num = 1  # 目标数量
hunter_num = 3  # 围捕机器人数量
obstacle_num = 2  # 障碍物数量
TargetIndex = [0]  # 目标机器人编号
robotPosition = np.zeros((RobotNum, 2))  # 初始化机器人位置

cell_wid = 0.03
Trapping_R = 3  # 设定包围半径
Avoiding_R = 3  # 设定避碰半径
target_threshold = Avoiding_R / 5 
Size = 400  # 实际场地长宽
rt = Size / 3  # 实际场地长宽转换为GRN浓度场长宽参数=16
size = 1 #障碍物尺寸

def main():
    # 文件路径（根据实际文件位置修改）
    target_path = "target.txt"
    hunter_path = "hunter.txt"
    obstacle_path = "obstacle.txt"
    
    # 读取数据
    target_df, hunter_df, obstacle_df = read_experiment_data(
        target_path, hunter_path, obstacle_path
    )
    
    # 障碍物半径（依次对应Obstacle1-3）
    obstacle_radii = [0.2, 0.18, 0.22]

    # 记录开始时间
    start_time = time.time()

    #初始化列表记录每个时间步的运行时间
    time_array = np.zeros((450,1))
    
    agent_colors = ['blue', 'green', 'purple','red'] #围捕机器人颜色，最后一个为目标颜色
    # uav_icon = mpimg.imread('UAV.png') # 读取UAV图标
    L_sensor = 0.2  # 探测范围半径

    #历史位置列表
    history_positions = [[] for _ in range(target_num+hunter_num)]

    plt.figure(figsize=(4,4),dpi=100)


    strategy = Strategy(TargetIndex, RobotNum)
    #围捕模式划分
    pattern_num = 3
    Dv = np.zeros((450, 1))

    # 创建一个空列表来存储提取出的 坐标
    
    historyPos = []
    lastCurrentPoint = []
    lastlastCurrentPoint = []

    # 运行450个时间步（0-299）
    for time_step in range(450):
        # 提取当前时间步的目标位置（Target1）
        target1_x = target_df.loc[time_step, "Target1_X"]
        target1_y = target_df.loc[time_step, "Target1_Y"]
        robotPosition[0][0] = target_df.loc[time_step, "Target1_X"]
        robotPosition[0][1]= target_df.loc[time_step, "Target1_Y"]
        static_obstacles = []

        if time_step == 0:
            # 定义围捕机器人列表，提取当前时间步的围捕机器人位置（3个机器人）
            hunters = [
                (hunter_df.loc[time_step, "Hunter1_X"], hunter_df.loc[time_step, "Hunter1_Y"]),
                (hunter_df.loc[time_step, "Hunter2_X"], hunter_df.loc[time_step, "Hunter2_Y"]),
                (hunter_df.loc[time_step, "Hunter3_X"], hunter_df.loc[time_step, "Hunter3_Y"])
            ]
            robotPosition[1] = hunters[1]
            robotPosition[3] = hunters[2]
            robotPosition[2] = hunters[0]
            historyPos.append(robotPosition.copy())

        # 定义障碍物列表，提取当前时间步的障碍物位置（3个障碍物）
        obstacles = [
            (obstacle_df.loc[time_step, "Obstacle1_X"], obstacle_df.loc[time_step, "Obstacle1_Y"]),
            (obstacle_df.loc[time_step, "Obstacle2_X"], obstacle_df.loc[time_step, "Obstacle2_Y"])
        ]
        # obstacles = [
        #     (obstacle_df.loc[time_step, "Obstacle1_X"], obstacle_df.loc[time_step, "Obstacle1_Y"]),
        #     (obstacle_df.loc[time_step, "Obstacle2_X"], obstacle_df.loc[time_step, "Obstacle2_Y"]),
        #     (obstacle_df.loc[time_step, "Obstacle3_X"], obstacle_df.loc[time_step, "Obstacle3_Y"])
        # ]
        for obs in obstacles:
            static_obstacles.append(obs)  # 将障碍物位置添加到静态障碍物列表中

        plt.clf()  # 清除当前图形
        
        getAngle = [[] for _ in range(RobotNum)]
        getdistance = [[] for _ in range(RobotNum)]
        pattern_point = []
        robot_index = []
        currentpoint = [[] for _ in range(RobotNum)]
        con_line_list = [[] for _ in range(RobotNum)]
        target_index = [i for i in range(RobotNum) if i in TargetIndex]
        selected_indices = []  # 已经被选中的目标点的索引
        full_flag = False

        #当前时间步开始时间
        start_time_step = time.time()

        pattern_point, con_line = strategy.get_pattern_point_1(robotPosition, target_index, static_obstacles, size)
        con_line_list[0] = con_line

        # plt.plot(con_line[:, 0], con_line[:, 1], color='red', alpha=1, linestyle='--', label='Concentration Line')
        # plt.pause(0.01)  # 暂停以显示图形

        #固定位置分配目标地
        idx = 0
        for point in pattern_point:
            while idx in target_index:
                idx = idx + 1
            if time_step == 0:
                currentpoint[idx] = point
            else:
                currentpoint[idx] = (point + lastCurrentPoint[idx])/ 2  # 平均上次和本次的目标点
                
            idx = idx + 1


        # plt.scatter(currentpoint[1][0], currentpoint[1][1],color = 'black')
        # plt.scatter(currentpoint[2][0], currentpoint[2][1],color = 'black')
        # plt.scatter(currentpoint[3][0], currentpoint[3][1],color = 'black')
        # plt.pause(0.01)

        #  控制机器人
        for i in range(RobotNum):
            if i not in target_index:  # 障碍物机器人
                robot_index = int(i)  # 第i台小车
                v_move = strategy.go_to_point(robotPosition, i, currentpoint,
                                                static_obstacles,
                                                size,
                                                historyPos,
                                                lastCurrentPoint,
                                                lastlastCurrentPoint)  # 围捕机器人控制

                getdistance[robot_index] = v_move

        for spped in range(len(getdistance)):
            if spped not in target_index :
                robotPosition[spped][0] += getdistance[spped][0]
                robotPosition[spped][1] += getdistance[spped][1]
            else:
                continue

        #当前时间步结束时间
        end_time_step = time.time()
        # 计算当前时间步运行时间
        time_array[time_step] = end_time_step - start_time_step

        min_dis = 100*np.ones((hunter_num,1))
        trap_angle = np.zeros((hunter_num,1))
        angle_group = np.zeros((pattern_num,1))

        for i in range(1,hunter_num+1):
            trap_angle[i-1] = math.atan2(robotPosition[i][0]-robotPosition[0][0],robotPosition[i][1]-robotPosition[0][1])
            for angle in range(1,pattern_num+1):
                last_pattern = (angle-1)/pattern_num*2*np.pi-np.pi
                current_pattern = angle/pattern_num*2*np.pi-np.pi
                if trap_angle[i-1] >= last_pattern and trap_angle[i-1] <= current_pattern:
                    angle_group[angle-1] = 1
            for j in range(1,hunter_num+1):
                if j==i:
                    continue
                dis = np.linalg.norm(robotPosition[i]-robotPosition[j])
                if dis < min_dis[i-1]:
                    min_dis[i-1] = dis

        Dv[time_step] = 1 - sum(angle_group)/pattern_num + sum((min_dis - np.mean(min_dis))**2)/(hunter_num)

        # 将坐标保存为历史坐标
        historyPos.append(robotPosition.copy())
        if time_step == 0:
            lastCurrentPoint = currentpoint.copy()
        else :
            lastlastCurrentPoint = lastCurrentPoint.copy()
            lastCurrentPoint = currentpoint.copy()
 
        for i in range(hunter_num):
            pos = robotPosition[i+1]  # 因为robotPosition[0]是目标位置，所以从1开始

            history_positions[i+1].append(pos.copy())  # 保存围捕机器人的历史位置
            trajectory = np.array(history_positions[i+1])
            # plot trajectory
            plt.plot(trajectory[:, 0], trajectory[:, 1], color=agent_colors[i], alpha=0.3)
            # Calculate the angle of the velocity vector
            plt.scatter(pos[0], pos[1], color=agent_colors[i])
            # # UAV图标放在当前位置
            # icon_size = 0.1
            # plt.imshow(
            #     uav_icon,
            #     extent=(
            #         pos[0] - icon_size/2, pos[0] + icon_size/2,
            #         pos[1] - icon_size/2, pos[1] + icon_size/2
            #     )
            # )
            # 绘制围捕无人机的探测范围（虚线圆圈）
            detection_circle = plt.Circle(pos, L_sensor, color=agent_colors[i], fill=False, linestyle='--', alpha=0.5)
            # plt.gca().add_patch(detection_circle)

    

        plt.scatter(target1_x, target1_y, c=agent_colors[-1], label='Target')
        history_positions[0].append((target1_x, target1_y))
        trajectory = np.array(history_positions[0])
        plt.plot(trajectory[:, 0], trajectory[:, 1], color=agent_colors[-1], alpha=0.3)

        for i, obs in enumerate(obstacles):
            circle = plt.Circle(obs, obstacle_radii[i], color='gray', alpha=0.5)
            plt.gca().add_patch(circle)
        
        plt.xlim(-0.1, 3.1)
        plt.ylim(-0.1, 3.1)
        # plt.legend()
        plt.pause(0.01)  # 动画效果

        if time_step + 1 in [100, 200, 300, 450]:
            filename = f"GRN-SMC_{time_step + 1}.pdf"
            plt.savefig(filename, format='pdf', bbox_inches='tight', pad_inches=0)

        # 此处可添加对当前时间步数据的处理逻辑

    #  记录结束时间
    end_time = time.time()
    # 计算运行时间
    run_time = end_time - start_time
    print(f"运行时间: {run_time} 秒")
    #保存time_array为txt文件
    np.savetxt("GRN_SMC_time_array.txt", time_array)

    plt.show()

    
    # 绘制Dv图
    plt.figure(figsize=(5, 10))
    plt.plot(Dv)
    plt.xlabel('Time Step')
    plt.ylabel('Dv')
    plt.title('Dv over Time')
    plt.show()

    # #保存Dv图
    plt.savefig("GRN_SMC_Dv.pdf")
    np.savetxt("GRN-SMC_Dv.txt", Dv)



    # fig, ax = plt.subplots(figsize=(8, 8))

    # # 历史位置列表
    # history_positions = [[] for _ in range(target_num + hunter_num)]

    # def update(time_step):
    #     ax.clear()
    #     # 提取当前时间步的目标位置
    #     target1_x = historyPos[time_step][0][0]
    #     target1_y = historyPos[time_step][0][1]
    #     # 围捕机器人
    #     hunters = historyPos[time_step][1:4]  # 因为robotPosition[0]是目标位置，所以从1开始
    #     for i in range(hunter_num):
    #         pos = hunters[i]
    #         history_positions[i].append(pos)
    #         trajectory = np.array(history_positions[i])
    #         ax.plot(trajectory[:, 0], trajectory[:, 1], color=agent_colors[i], alpha=0.3)
    #         icon_size = 0.1
    #         ax.scatter(pos[0], pos[1], c=agent_colors[i])
    #         # ax.imshow(
    #         #     uav_icon,
    #         #     extent=(
    #         #         pos[0] - icon_size/2, pos[0] + icon_size/2,
    #         #         pos[1] - icon_size/2, pos[1] + icon_size/2
    #         #     )
    #         # )
    #         detection_circle = Circle(pos, L_sensor, color=agent_colors[i], fill=False, linestyle='--', alpha=0.5)
    #         ax.add_patch(detection_circle)
    #     # 障碍物
    #     obstacles = [
    #         (obstacle_df.loc[time_step, "Obstacle1_X"], obstacle_df.loc[time_step, "Obstacle1_Y"]),
    #         (obstacle_df.loc[time_step, "Obstacle2_X"], obstacle_df.loc[time_step, "Obstacle2_Y"]),
    #         (obstacle_df.loc[time_step, "Obstacle3_X"], obstacle_df.loc[time_step, "Obstacle3_Y"])
    #     ]
    #     ax.scatter(target1_x, target1_y, c=agent_colors[-1])
    #     history_positions[-1].append((target1_x, target1_y))
    #     trajectory = np.array(history_positions[-1])
    #     ax.plot(trajectory[:, 0], trajectory[:, 1], color=agent_colors[-1], alpha=0.3)
    #     for i, obs in enumerate(obstacles):
    #         circle = Circle(obs, obstacle_radii[i], color='gray', alpha=0.5)
    #         ax.add_patch(circle)
    #     ax.set_xlim(-0.1, 3.1)
    #     ax.set_ylim(-0.1, 3.1)
    #     # ax.legend()

    ani = animation.FuncAnimation(fig, update, frames=300, interval=50)
    ani.save('animation.gif', writer='pillow')  # 保存为GIF
    # # ani.save('animation.mp4', writer=animation.FFMpegWriter(fps=20))  # 保存为MP4（需安装ffmpeg）

#  控制线程
class Strategy:
    def __init__(self, target_index, robotnum):

        global Avoiding_R
        global cell_wid
        global target_threshold
        self.TARGET_THRESHOLD = target_threshold
        self.AVOID_RADIUS = Avoiding_R  # 包围半径
        self.robotnum = robotnum
        self.this_time_neighbor = []  # 当前时刻邻居
        self.this_time_target = []  # 当前时刻目标位置
        self.this_time_rigidbody = []  # 当前时刻、当前机器人的位置
        self.predict_rigidbody = [0., 0.]  # 下一时刻机器人位置
        self.this_time_obs = []  # 障碍物位置
        self.target_num = target_num  # 目标机器人数量
        self.target_index = target_index  # 目标机器人编号，最多2个目标
        self.v = 0.5  # 机器人速度
        self.v_max = 1  # 归一化后机器人最大速度的模值
        self.movePTs = np.zeros([robotnum, 2], np.double)  # 预计移动后的位置

    # 对于围捕机器人的运动控制
    def go_to_point(self, dict, mynum, point, static_obstacles, size, history_position,lastCurrentPoint,lastlastCurrentPoint):  # 所有机器人位置、当前机器人编号、pattern上的目标点 
        ## history_position 围捕机器人的历史位置，记录了其从开始的所有坐标
        ## lastCurrentPoint lastlastCurrentPoint 上次和上上次的围捕目标点
        self.this_time_rigidbody = np.array([dict[mynum][0], dict[mynum][1]])  # 将该编号的机器人位置赋值给当前时刻位置
        self.this_time_target = np.array([point[mynum][0], point[mynum][1]])  # 将pattern上目标点位置赋值给当前时刻目标位置
        
        C = 2
        alpha = 0.5
        k1 = 0.08
        ksi = 0.2
        dt = 0.1
        a_max = 0.1 # 最大加速度
        v_max = 1  # 最大速度

        if  len(history_position) == 0:
            last_P =  np.zeros(dict.shape)
            llast_P =  np.zeros(dict.shape)
        elif len(history_position) == 1:
            last_P = history_position[-1]
            llast_P = np.zeros(dict.shape)
        else :
            last_P = history_position[-1]
            llast_P = history_position[-2]

        if not lastCurrentPoint:
            last_C = np.zeros(dict.shape)
        else:
            last_C = lastCurrentPoint

        if not lastlastCurrentPoint:
            llast_C = np.zeros(dict.shape)
        else:
            llast_C = lastlastCurrentPoint
        
        x = dict[mynum][0]
        dx = dict[mynum][0] - last_P[mynum][0]
        ddx = last_P[mynum][0] - llast_P[mynum][0]

        x_d = point[mynum][0]
        dx_d = point[mynum][0] - last_C[mynum][0]
        ddx_d =llast_C[mynum][0] - llast_C[mynum][0]

        y = dict[mynum][1]
        dy = dict[mynum][1] - last_P[mynum][1]
        ddy = last_P[mynum][1] - llast_P[mynum][1]

        y_d = point[mynum][1]
        dy_d = point[mynum][1] - last_C[mynum][1]
        ddy_d =llast_C[mynum][1] - llast_C[mynum][1]
    
        e_x = x - x_d
        e_y = y - y_d
        de_x = dx - dx_d
        de_y = dy - dy_d

    
        # sliding plant
        s_x = de_x + C*e_x
        s_y = de_y + C*e_y
        
        # control input
        u_x = ddx_d - C*(dx - dx_d) - k1*s_x - ksi * np.abs(s_x) ** alpha * np.sign(s_x)
        u_y = ddy_d - C*(dy - dy_d) - k1*s_y - ksi * np.abs(s_y) ** alpha * np.sign(s_y)

        if u_x > a_max:
           u_x = a_max
        if u_y > a_max:
           u_y = a_max
        
        # ddx ddy
        ddx = u_x 
        ddy = u_y 
        
        # dx dy
        dx = dx+ddx *dt
        dy = dy+ddy *dt

        if dx > 0 and dx > v_max:
           dx = v_max
        elif dx < 0 and dx < -v_max:
           dx = -v_max

        if dy > 0 and dy > v_max:
           dy = v_max
        elif dy < 0 and dy < -v_max:
           dy = -v_max

        v_move = np.array([dx, dy])  # 计算位移增量

        return v_move  # 返回该编号机器人下一时刻的位置

    # 对于目标机器人生成围捕圈（如果只有一个机器人或者两者之间的距离小于阈值，当成一个机器人生成围捕圈）
    def get_pattern_point_1(self, dict, mynum, static_obstacles, size):  # 机器人位置列表、目标机器人列表

        self.this_time_rigidbody = [[] for _ in mynum]  # 初始化当前时刻机器人位置
        # 更新 self.this_time_rigidbody 列表，使其包含目标机器人列表对应的位置
        for index in range(len(mynum)):
            self.this_time_rigidbody[index] = dict[mynum[index]]  # 将目标机器人赋值为当前时刻机器人位置

        # 清空障碍物位置列表
        self.this_time_obs.clear()
        # 遍历所有目标机器人
        for index in range(len(self.this_time_rigidbody)):
            # 遍历所有障碍物
            for obs in range(len(static_obstacles)):

                # 转换障碍物位置为 NumPy 数组，用于比较
                static_obstacle_array = np.array(static_obstacles[obs])
                # 检查障碍物是否不在 self.this_time_obs 中
                if static_obstacle_array.tolist() not in self.this_time_obs:  # 此行代码防止障碍物重复加入障碍物列表中
                    # 添加障碍物位置到 self.this_time_obs
                    self.this_time_obs.append(static_obstacle_array.tolist())
        if self.this_time_obs == []:
            self.this_time_obs.append([30,30])
        neighborpattern = self.neighbor_pattern(self.this_time_obs, size)  # 计算障碍物的浓度场
        con_line = self.OwnPattern_1(self.this_time_rigidbody, neighborpattern, size)  # 根据当前目标机器人位置和障碍物浓度场计算综合浓度场

        # # 排除在障碍物范围内的点
        contour_coords = con_line.copy()
        # to_remove = []
        # for i in range(len(temp)):
        #     for obs in static_obstacles:
        #         distance_obs = np.sqrt((temp[i][0] - obs[0]) ** 2 + (temp[i][1] - obs[1]) ** 2)
        #         if distance_obs < self.AVOID_RADIUS / rt:  # 如果点在障碍物范围内
        #             to_remove.append(i)
        #             break

        # temp = np.delete(temp, to_remove, axis=0)


        # 1. 找到y坐标最大的点
        max_y_idx = np.argmax(contour_coords[:, 1])
        start_point = contour_coords[max_y_idx]

        # 2. 从max_y_idx开始重新排列坐标，使等高线从max_y点开始
        coords_rolled = np.roll(contour_coords, -max_y_idx, axis=0)

        # 3. 计算等高线的总长度
        distances = np.sqrt(np.sum(np.diff(coords_rolled, axis=0)**2, axis=1))
        total_length = np.sum(distances)

        # 4. 计算三个等分点的目标累积距离
        target_distances = np.array([total_length * 1/3, total_length * 2/3])

        # 5. 找到每个目标距离对应的点
        trisection_points = [start_point]  # 第一个点是y最大的点
        current_distance = 0
        target_idx = 0

        for i in range(len(distances)):
            if target_idx >= len(target_distances):
                break
            
            current_distance += distances[i]

            # 当累积距离达到或超过目标距离时，记录该点
            if current_distance >= target_distances[target_idx]:
                # 线性插值找到更精确的点位置
                point = coords_rolled[i]
                trisection_points.append(point.copy())
                target_idx += 1

            # 如果没有找到所有等分点，添加最后一个点
        while len(trisection_points) < 3:
            trisection_points.append(coords_rolled[-1])

        return np.array(trisection_points),con_line  # 返回目标点和综合浓度场

        # # 获取数组的尺寸
        # num = temp.shape[0]

        # # 找到第二列中最大值的索引
        # max_y_index = np.argmax(temp[:, 1])

        # # 根据索引计算有效区域的边界
        # valid1 = max_y_index
        # valid2 = num - valid1 
        # valid3 = valid2 + 1
        # valid4 = valid1 - 1

        # # 重新排列数组
        # new_temp = np.empty_like(temp)  # 创建一个与 temp 大小相同的空数组
        # new_temp[:valid2, :] = temp[valid1:, :]  # 将 temp 的后半部分复制到 new_temp 的前半部分
        # new_temp[valid3:, :] = temp[:valid4, :]  # 将 temp 的前半部分复制到 new_temp 的后半部分



        # point1 = new_temp[
        #         np.round(np.linspace(0, num - 1, self.robotnum - len(mynum)  + 1)).astype(
        #             int), :]  # 将多个目标点按等间距划分为围捕机器人数量个目标点
        # point1 = point1[:-1, :]
        # return point1, con_line  # 返回目标点

    # 计算障碍物浓度场
    def neighbor_pattern(self, this_time_obs, size):
        N_cell_x = 100
        N_cell_y = 100
        epi = Trapping_R / (10 * size)
        all_ = np.zeros((N_cell_x, N_cell_y))

        for obs in this_time_obs:  # 遍历所有障碍物，计算障碍物浓度场
            p1 = self.calc_mor(obs, epi)  # 计算该障碍物的浓度场，epi改变浓度场的斜率
            all_ = np.maximum(all_, p1)

        all_ = all_ / np.max(all_)
        np.seterr(divide='ignore', invalid='ignore')
        neighborpattern = all_
        return neighborpattern
         
    def calc_mor(self, pos, epi):
        N_cell_x = 100
        N_cell_y = 100
        pos = np.array(pos)  # 将pos列表转换为NumPy数组
        # epi = 2.7
        m = np.zeros((N_cell_x, N_cell_y))

        i, j = np.meshgrid(range(N_cell_x), range(N_cell_y))
        r = np.sqrt((cell_wid * np.abs(j - 0.5) - pos[0]) ** 2 + (cell_wid * np.abs(i - 0.5) - pos[1]) ** 2)
        m = m + 1 * np.exp(-r / epi)

        return m

    def OwnPattern_1(self, this_time_rigidbody, pattern, size):
        global Trapping_R, TargetIndex
        N_cell_x = 100
        N_cell_y = 100
        epi = Trapping_R / (12 * size)
        k = 1
        nxor = 0.1256

        all_ = np.zeros((N_cell_x, N_cell_y))
        for targetrobot in this_time_rigidbody:  # 遍历所有目标机器人
            p1 = self.calc_mor(targetrobot, epi)  # 计算目标机器人的综合浓度场
            all_ = np.maximum(all_, p1)

        p = all_ / np.max(all_)  # 目标机器人的综合浓度场
        z = self.Sigmoid(1 - p * p, nxor, k)
        u = self.Sigmoid(pattern * pattern, nxor, k)
        g1 = self.Sigmoid(z + u, nxor, k)  # 包含目标机器人和障碍物的综合浓度场

        x_mat = np.zeros((N_cell_x, N_cell_y))
        y_mat = np.zeros((N_cell_x, N_cell_y))
        for i1 in range(N_cell_x):
            x_mat[i1, :] = abs(i1 - 0.5) * cell_wid
        for j1 in range(N_cell_y):
            y_mat[:, j1] = abs(j1 - 0.5) * cell_wid

        # 绘制等高线
        plt.ion()
        plt.clf()
        #levels=[0.735]
        contour_set = plt.contour(x_mat, y_mat, g1,levels=[0.735])  # 根据综合浓度场生成等高线
        # plt.clabel(contour_set, inline=True, fontsize=8) 
        # plt.colorbar(contour_set)
        # 显示图形
        plt.show()

        contour_path = contour_set.collections[0].get_paths()[0]  # 获取第一个等高线的路径
        contour_coords = contour_path.vertices  # 获取路径上的所有坐标点
        # In = []
        # lines = list()
        # for level in contour_set.collections:  # 遍历所有等高线
        #     if len(level.get_paths()) == 0:
        #         continue
        #     point = level.get_paths()[-1].vertices
        #     # find repeat elements
        #     point = np.round(point * 1e5).astype(int) * 1e-5
        #     unique_values, counts = np.unique(point, return_counts=True, axis=0)
        #     duplicate_values = unique_values[counts > 1]
        #     for i in range(len(duplicate_values)):
        #         indices = np.where(np.all(point == duplicate_values[i], axis=1))[0]
        #         lines.append(point[indices[0]:indices[1] + 1])

        # # 提取目标机器人的位置
        # target_positions = [robotPosition[i] for i in TargetIndex]
        # # 计算目标机器人平均位置
        # average_position = tuple(np.mean(target_positions, axis=0))

        # for line in lines:
        #     poly = Polygon(line)
        #     in_poly = poly.contains_point(average_position)  # 按照目标机器人的平均位置选取围捕圈
        #     In.append(in_poly)

        # r = np.where(In)[0]
        # if len(r) != 0:
        #     n = np.max(r)
        #     con_line = lines[n]
        # else:
        #     print(this_time_rigidbody)
        #     con_line = [this_time_rigidbody]
        # plt.plot(outer_coordinates[:, 0], outer_coordinates[:, 1])
        # plt.show()

        return contour_coords

    def OwnPattern_2(self, this_time_rigidbody, pattern, size):
        global Trapping_R
        N_cell_x = 100
        N_cell_y = 100
        epi = Trapping_R / (11 * size)
        k = 1
        nxor = 0.1256

        p = self.calc_mor(this_time_rigidbody, epi)
        z = self.Sigmoid(1 - p * p, nxor, k)
        u = self.Sigmoid(pattern * pattern, nxor, k)
        g1 = self.Sigmoid(z + u, nxor, k)

        x_mat = np.zeros((N_cell_x, N_cell_y))
        y_mat = np.zeros((N_cell_x, N_cell_y))
        for i1 in range(N_cell_x):
            x_mat[i1, :] = abs(i1 - 0.5) * cell_wid
        for j1 in range(N_cell_y):
            y_mat[:, j1] = abs(j1 - 0.5) * cell_wid

        # 绘制等高线
        plt.clf()
        contour_set = plt.contour(x_mat, y_mat, g1)  # 根据综合浓度场生成等高线
        plt.colorbar(contour_set)
        plt.pause(0.01)  # 暂停以显示图形
        ##显示图形
        plt.show()

        In = []
        lines = list()
        for level in contour_set.collections:  # 遍历所有等高线
            if len(level.get_paths()) == 0:
                continue
            point = level.get_paths()[-1].vertices
            # find repeat elements
            point = np.round(point * 1e5).astype(int) * 1e-5
            unique_values, counts = np.unique(point, return_counts=True, axis=0)
            duplicate_values = unique_values[counts > 1]
            for i in range(len(duplicate_values)):
                indices = np.where(np.all(point == duplicate_values[i], axis=1))[0]
                lines.append(point[indices[0]:indices[1] + 1])

        for line in lines:
            poly = Polygon(line)
            in_poly = poly.contains_point(this_time_rigidbody)  # 按照目标机器人的平均位置选取围捕圈
            In.append(in_poly)

        r = np.where(In)[0]
        if len(r) != 0:
            n = np.max(r)
            con_line = lines[n]
        else:
            print(this_time_rigidbody)
            con_line = [this_time_rigidbody]
        # plt.plot(outer_coordinates[:, 0], outer_coordinates[:, 1])
        # plt.show()

        return con_line

    def Sigmoid(self, p, theta, k):
        return 1 / (1 + np.exp(-k * (p - theta)))




if __name__ == "__main__":
    main()