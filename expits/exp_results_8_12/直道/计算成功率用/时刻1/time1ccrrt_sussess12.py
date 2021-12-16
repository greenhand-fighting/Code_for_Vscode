import sys
sys.path.append("../..")
sys.path.append("../../..")
from 路径规划算法.CCRRT import *
#from draw_legend import *

def main():
    # standard 1
    maxcc_list_100times_succ_1=[]
    mincc_list_100times_succ_1=[]
    avgcc_list_100times_succ_1=[]
    long_list_100times_succ_1 = []
    pointnumber_list_100times_succ_1 = []
    # standard 2
    maxcc_list_100times_succ_2 = []
    mincc_list_100times_succ_2 = []
    avgcc_list_100times_succ_2 = []
    long_list_100times_succ_2 = []
    pointnumber_list_100times_succ_2 = []
    fail1 = 0
    fail2=0
    cycle_times = 0
    for i in range(100):
        print("Start -----------------------------1ccrrt!!!!!!!!!!!!!!!")
        cycle_times+=1
        area = [0, 15, 30, 100]
        start = [12.25, 35, np.deg2rad(90.0)]
        goal = [0.0, 70.0, np.deg2rad(90.0)]
        car = Vehicle()

        obstacle_list_gt_from_pu = [
            (5.58, 44.99, 4.12, 1.62, np.deg2rad(89.0), 2.7, 1.17),
            (5.47, 64.62, 4.4, 1.78, np.deg2rad(88.0), 2.87, 1.3),
            (9.17, 54.65, 4.17, 1.69, np.deg2rad(90.0), 2.65, 1.19),
            (12.67, 44.82, 3.88, 1.61, np.deg2rad(89.90), 2.49, 1.14),
            (12.76, 64.96, 3.95, 1.65, np.deg2rad(90.0), 2.91, 1.25),
        ]
        ganzhi_kuang = [
            (5.58, 44.99, 4.12 / 2, 1.62 / 2, np.deg2rad(89.0)),
            (5.47, 64.62, 4.4 / 2, 1.78 / 2, np.deg2rad(88.0)),
            (9.17, 54.65, 4.17 / 2, 1.69 / 2, np.deg2rad(90.0)),
            (12.67, 44.82, 3.88 / 2, 1.61 / 2, np.deg2rad(89.90)),
            (12.76, 64.96, 3.95 / 2, 1.65 / 2, np.deg2rad(90.0)),
        ]
        obstacle_list_uncertainty = []
        for obs in ganzhi_kuang:
            dist = np.hypot(start[0] - obs[0], start[1] - obs[1])
            dist=25
            un = (un_generate(dist, 0.15, 0.75),  # sigma_ver
                  un_generate(dist, 0.1, 0.65),  # sigma_hor
                  un_generate(dist, 0.05, 0.06)  # sigma_radius
                  )
            obstacle_list_uncertainty.append(un)

        ground_true_obs_list = [
            (12.25, 35.0, 4.51, 2.0, np.deg2rad(90.0)),
            (1.9, 75.0, 4.72, 1.89, np.deg2rad(90.0)),
            (5.5, 45.0, 4.19, 1.82, np.deg2rad(90.0)),
            (5.5, 65.0, 4.79, 2.16, np.deg2rad(90.0)),
            (5.5, 95.0, 5.36, 2.03, np.deg2rad(90.0)),
            (9.1, 55.0, 4.86, 2.03, np.deg2rad(90.0)),
            (9.1, 85.0, 3.99, 1.85, np.deg2rad(90.0)),
            (12.7, 45.0, 4.18, 1.99, np.deg2rad(90.0)),
            (12.7, 65.0, 4.61, 2.24, np.deg2rad(90.0)),
        ]
        zhenzhi_kuang = [
            (1.9, 75.0, 4.72 / 2, 1.89 / 2, np.deg2rad(90.0)),
            (5.5, 45.0, 4.19 / 2, 1.82 / 2, np.deg2rad(90.0)),
            (5.5, 65.0, 4.79 / 2, 2.16 / 2, np.deg2rad(90.0)),
            (5.5, 95.0, 5.36 / 2, 2.03 / 2, np.deg2rad(90.0)),
            (9.1, 55.0, 4.86 / 2, 2.03 / 2, np.deg2rad(90.0)),
            (9.1, 85.0, 3.99 / 2, 1.85 / 2, np.deg2rad(90.0)),
            (12.7, 45.0, 4.18 / 2, 1.99 / 2, np.deg2rad(90.0)),
            (12.7, 65.0, 4.61 / 2, 2.24 / 2, np.deg2rad(90.0)),
        ]

        # (x, y, long_axis, short_axis, radius [-pi, pi])
        # vehicle_length = long_axis * 2
        # vehicle_width = short_axis * 2
        obstacle_list = obstacle_uncertainty_fusion(ganzhi_kuang,obstacle_list_uncertainty)
        ganzhi_tuoyuan=obstacle_uncertainty_fusion_from_pu(obstacle_list_gt_from_pu)
        cc_rrt = CCRRT(
            car=car,
            start=start,
            goal=goal,
            rand_area=area,
            obstacle_list=obstacle_list,
            obstacle_list_from_pu=ganzhi_tuoyuan,
            obstacle_list_for_pengzhuang_jiance=ganzhi_kuang)
        cc_rrt.p_safe = 0.99
        cc_rrt.planning(animation=False)
        # 整理数据
        tmp = [cc_rrt.get_chance_constrain_from_pu(node) for node in cc_rrt.path]  # 从这个可以看出这个finalpath里面都是一些节点
        print(tmp)
        print(len(cc_rrt.path))
        path_min = np.min(tmp)
        path_max = np.max(tmp)
        path_avg = np.average(tmp)
        pointnumber_list_100times_succ_1.append(len(cc_rrt.path))
        # 计算路径长度
        sum = 0
        for j in range(len(cc_rrt.path) - 1):
            changdu, _ = cc_rrt.calc_distance_and_angle(cc_rrt.path[j], cc_rrt.path[j + 1])
            sum = sum + changdu
            print("sum: %f" % sum)

        pengzhuang = 0
        for node_in_final_path in cc_rrt.path:
            if cc_rrt.peng_zhuang_jian_ce(node_in_final_path, zhenzhi_kuang):
                # 发生碰撞
                pengzhuang = 1
                break
        if pengzhuang:  # 碰撞
            fail1 += 1
            fail2 += 1

        else:  # 没有碰撞

            long_list_100times_succ_1.append(sum)
            maxcc_list_100times_succ_1.append(path_max)
            mincc_list_100times_succ_1.append(path_min)
            avgcc_list_100times_succ_1.append(path_avg)
            pointnumber_list_100times_succ_1.append(len(cc_rrt.path))

            if len(cc_rrt.path_end):  # 有完整路径
                long_list_100times_succ_2.append(sum)
                maxcc_list_100times_succ_2.append(path_max)
                mincc_list_100times_succ_2.append(path_min)
                avgcc_list_100times_succ_2.append(path_avg)
                pointnumber_list_100times_succ_2.append(len(cc_rrt.path))

            else:  # 无完整路径
                fail2 += 1

        print('*' * 100)
        print("n lun : %d" % i)
    pointnumber_list_100times_succ_1=np.array(pointnumber_list_100times_succ_1)
    long_list_100times_succ_1=np.array(long_list_100times_succ_1)
    maxcc_list_100times_succ_1=np.array(maxcc_list_100times_succ_1)
    mincc_list_100times_succ_1=np.array(mincc_list_100times_succ_1)
    avgcc_list_100times_succ_1=np.array(avgcc_list_100times_succ_1)

    pointnumber_list_100times_succ_2 = np.array(pointnumber_list_100times_succ_1)
    long_list_100times_succ_2= np.array(long_list_100times_succ_1)
    maxcc_list_100times_succ_2 = np.array(maxcc_list_100times_succ_1)
    mincc_list_100times_succ_2 = np.array(mincc_list_100times_succ_1)
    avgcc_list_100times_succ_2 = np.array(avgcc_list_100times_succ_1)

    print("STANDARD 1")
    print("max: %f" % np.average(maxcc_list_100times_succ_1))
    print("avg: %f" % np.average(avgcc_list_100times_succ_1))
    print("min: %f" % np.average(mincc_list_100times_succ_1))
    print("points: %f" % np.average(pointnumber_list_100times_succ_1))
    print("changdu: %f" % np.average(long_list_100times_succ_1))
    print('*'*100)
    print("fail1=:= %f" % fail1)
    print(cycle_times)
    success_rate1= (cycle_times - fail1) / (cycle_times)
    print("success1 rate: %f" % success_rate1)

    print("STANDARD 2")
    print("max: %f" % np.average(maxcc_list_100times_succ_2))
    print("avg: %f" % np.average(avgcc_list_100times_succ_2))
    print("min: %f" % np.average(mincc_list_100times_succ_2))
    print("points: %f" % np.average(pointnumber_list_100times_succ_2))
    print("changdu: %f" % np.average(long_list_100times_succ_2))
    print('*' * 100)
    print("fail2=:= %f" % fail2)
    print(cycle_times)
    success_rate2 = (cycle_times - fail2) / (cycle_times)
    print("success2 rate: %f" % success_rate2)


main()
