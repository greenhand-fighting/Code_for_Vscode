import sys
sys.path.append("../../..")
from 路径规划算法.CCRRT import *
from draw_legend import *

def main():
    maxcc_list_100times=[]
    mincc_list_100times=[]
    avgcc_list_100times=[]
    long_list_100times = []
    pointnumber_list_100times = []
    fail = 0
    cycle_times = 0
    for i in range(1):
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
        obstacle_list_gt = [
            # (1.9, 75.0, 4.72, 1.89, np.deg2rad(90.0)),
            (5.58, 44.99, 4.12, 1.62, np.deg2rad(89.0)),
            (5.47, 64.62, 4.4, 1.78, np.deg2rad(88.0)),
            #   (5.5, 95, 5.36, 2.03, np.deg2rad(90.0)),
            (9.17, 54.65, 4.17, 1.69, np.deg2rad(90.0)),
            #  (9.1, 85, 3.99, 1.85, np.deg2rad(90.0)),
            (12.67, 44.82, 3.88, 1.61, np.deg2rad(89.90)),
            (12.76, 64.96, 3.95, 1.65, np.deg2rad(90.0)),
        ]
        obstacle_list_uncertainty = []
        for obs in obstacle_list_gt:
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
        obstacle_list = obstacle_uncertainty_fusion(obstacle_list_gt,obstacle_list_uncertainty)
        obstacle_list_from_pu=obstacle_uncertainty_fusion_from_pu(obstacle_list_gt_from_pu)
        obstacle_list_for_pengzhuang_jiance = get_obstacle_list_for_pengzhuang_jiance(obstacle_list_gt)
        cc_rrt = CCRRT(
            car=car,
            start=start,
            goal=goal,
            rand_area=area,
            obstacle_list=obstacle_list,
            obstacle_list_from_pu=obstacle_list_from_pu,
            obstacle_list_for_pengzhuang_jiance=obstacle_list_for_pengzhuang_jiance)
        cc_rrt.p_safe = 0.99
        cc_rrt.planning(animation=False)
        if len(cc_rrt.path_end) == 0:  # 这里是没有找到路径
            fail += 1
            print("because on path ::::fail=== %f " % fail)
        else:  # 这里是找到了路径，然后要判断它是不是压过了groundtruth
            flag = fail
            print('flag== %f' % flag)
            for node_in_final_path in cc_rrt.path:
                if cc_rrt.peng_zhuang_jian_ce(node_in_final_path,
                                              zhenzhi_kuang):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
                    print(node_in_final_path.x, node_in_final_path.y)
                    fail += 1
                    break
            if flag != fail:
                print('有路径，但是这个路径从groundtruth上踏过！ \n 失败次数： %d！' % fail)
            if fail == flag:  # 说明有路径而且没有和grountruth碰撞
                tmp = [cc_rrt.get_chance_constrain_from_pu(node) for node in cc_rrt.path]  # 从这个可以看出这个finalpath里面都是一些节点
                print(tmp)
                print(len(cc_rrt.path))
                path_min = np.min(tmp)
                path_max = np.max(tmp)
                path_avg = np.average(tmp)
                pointnumber_list_100times.append(len(cc_rrt.path))
                # 计算路径长度
                sum=0
                for j in range(len(cc_rrt.path)-1):
                    changdu, _ = cc_rrt.calc_distance_and_angle(cc_rrt.path[j], cc_rrt.path[j+1])
                    sum=sum+changdu
                    print("sum: %f" % sum)

                long_list_100times.append(sum)
                maxcc_list_100times.append(path_max)
                mincc_list_100times.append(path_min)
                avgcc_list_100times.append(path_avg)

        print('*' * 100)
        print("n lun : %d" % i)
    pointnumber_list_100times=np.array(pointnumber_list_100times)
    long_list_100times=np.array(long_list_100times)
    maxcc_list_100times=np.array(maxcc_list_100times)
    mincc_list_100times=np.array(mincc_list_100times)
    avgcc_list_100times=np.array(avgcc_list_100times)
    print("max: %f" % np.average(maxcc_list_100times))
    print("avg: %f" % np.average(avgcc_list_100times))
    print("min: %f" % np.average(mincc_list_100times))
    print("points: %f" % np.average(pointnumber_list_100times))
    print("changdu: %f" % np.average(long_list_100times))
    print('*'*100)
    print("fail=:= %f" % fail)
    print(cycle_times)
    success_rate = (cycle_times - fail) / (cycle_times)
    print("success rate: %f" % success_rate)


# if __name__ == '__main__':
#     main()
