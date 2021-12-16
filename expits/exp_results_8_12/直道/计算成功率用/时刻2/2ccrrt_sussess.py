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
    for i in range(100):
        print("Start -----------------------------1ccrrt! on gt")
        cycle_times+=1

        area = [0, 15, 30, 100]  # x-min x-max y-min y-max

        # Set Initial parameters
        start = [5.5, 50.7, np.deg2rad(90.0)]
        goal = [5.0, 85.0, np.deg2rad(90.0)]
        car = Vehicle()

        # ====Search Path with CCRRT====
        # (x, y, vehicle_length, vehicle_width, radius [-pi, pi])
        # axis = length + sigma

        # -----在列表的后面直接将长半轴和短半轴加上去，不用再进行计算长短半轴了，因为rosbag中已经给出了长短半轴
        obstacle_list_gt_from_pu = [
            (1.978, 74.55, 4.13, 1.63, np.deg2rad(90.0), 2.67, 1.2),
            # (5.36, 44.96, 3.96, 1.59, np.deg2rad(100.0),2.61, 1.19),
            (5.47, 64.54, 3.79, 1.63, np.deg2rad(89.0), 2.47, 1.14),
            #   (5.5, 95, 5.36, 2.03, np.deg2rad(90.0),5.36/2, 2.03/2),
            (9.0, 54.93, 4.08, 1.59, np.deg2rad(90.0), 2.6, 1.14),
            (9.02, 84.86, 4.37, 1.67, np.deg2rad(90.0), 2.65, 1.32),
            # (12.73, 44.84, 3.4, 1.5, np.deg2rad(89.90), 2.28, 1.13),
            (12.55, 64.96, 4.45, 1.71, np.deg2rad(90.0), 2.83, 1.18),
        ]
        obstacle_list_gt = [
            (1.978, 74.55, 4.13, 1.63, np.deg2rad(90.0)),
            # (5.36, 44.96, 3.96, 1.59, np.deg2rad(100.0)),
            (5.47, 64.54, 3.79, 1.63, np.deg2rad(89.0)),
            #   (5.5, 95, 5.36, 2.03, np.deg2rad(90.0)),
            (9.0, 54.93, 4.08, 1.59, np.deg2rad(90.0)),
            (9.02, 84.86, 4.37, 1.67, np.deg2rad(90.0)),
            # (12.73, 44.84, 3.4, 1.5, np.deg2rad(89.90)),
            (12.55, 64.96, 4.45, 1.71, np.deg2rad(90.0)),
        ]
        obstacle_list_uncertainty = []
        for obs in obstacle_list_gt:
            dist = np.hypot(start[0] - obs[0], start[1] - obs[1])
            dist=25
            un = (un_generate(dist, 0.5, 1.0),  # sigma_ver
                  un_generate(dist, 0.3, 0.85),  # sigma_hor
                  un_generate(dist, 0.2, 0.1)  # sigma_radius
                  )
            obstacle_list_uncertainty.append(un)

        # sigam_ver, sigma_hor, sigma_radius

        ground_true_obs_list = [
            (5.5, 50.7, 4.51, 2.0, np.deg2rad(90.0)),
            (1.9, 75.0, 4.72, 1.89, np.deg2rad(90.0)),
            (5.5, 45.0, 4.19, 1.82, np.deg2rad(90.0)),
            (5.5, 65.0, 4.79, 2.16, np.deg2rad(90.0)),
            (5.5, 95.0, 5.36, 2.03, np.deg2rad(90.0)),
            (9.1, 55.0, 4.86, 2.03, np.deg2rad(90.0)),
            (9.1, 85.0, 3.99, 1.85, np.deg2rad(90.0)),
            (12.7, 45.0, 4.18, 1.99, np.deg2rad(90.0)),
            (12.7, 65.0, 4.61, 2.24, np.deg2rad(90.0)),
        ]
        ground_truth = [
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
        obstacle_list_for_pengzhuang_jiance=get_obstacle_list_for_pengzhuang_jiance(obstacle_list_gt)

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
            #fail += 1
            print("because no path ::::fail=== %f " % fail)
        else:  # 这里是找到了路径，然后要判断它是不是压过了groundtruth
            # print(cc_rrt.check_chance_constrain(cc_rrt.end, cc_rrt.p_safe))
            # print(cc_rrt.check_chance_constrain(cc_rrt.start, cc_rrt.p_safe))
            # if path is None:
            #     print("Cannot find path")
            # else:
            #     print("found path!!")

            # # Draw final path
            # if show_animation:
            #     cc_rrt.draw_graph()
            #     plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            #     plt.grid(True)
            #     plt.pause(0.01)  # Need for Mac
            #     plt.show()

            """
            cc_rrt.draw_graph()
            cc_rrt.draw_path()
            draw_vehicle(obstacle_list_gt)
            draw_ground_true(ground_true_obs_list)
            draw_carsize_of_final_path(cc_rrt.path)

            #plt.clf()

            plt.figure(2)
            #plt.clf()
            """
            flag = fail
            print('flag== %f' % flag)
            for node_in_final_path in cc_rrt.path:
                if cc_rrt.peng_zhuang_jian_ce(node_in_final_path,
                                              ground_truth):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
                    print(node_in_final_path.x, node_in_final_path.y)
                    fail += 1
                    break
            if flag != fail:
                print('有路径，但是这个路径从groundtruth上踏过！！！！！！！！')
                """
                print('有路径，但是这个路径从groundtruth上踏过！！！！！！！！')

                print('great! ' * 100)
                # 画1
                cc_rrt.draw_graph()
                cc_rrt.draw_path()
                draw_vehicle(obstacle_list_gt)
                draw_ground_true(ground_true_obs_list)
                draw_carsize_of_final_path(cc_rrt.path)

                # plt.clf()
                tmp1 = [node.cc for node in cc_rrt.path]
                path_min1 = np.min(tmp1)
                path_max1 = np.max(tmp1)
                path_avg1 = np.average(tmp1)
                plt.figure(2)
                # plt.clf()
                plt.axes([0.3, 0.1, 8 / 50, 8 / 10.55])
                plt.title("Without considering uncertainty\n Blue bounding box: Ground True")
                plt.scatter([node.x for node in cc_rrt.node_list],
                            [node.y for node in cc_rrt.node_list],
                            s=3,
                            c=[node.cc for node in cc_rrt.node_list],
                            cmap='jet')
                plt.plot([node.x for node in cc_rrt.path],
                         [node.y for node in cc_rrt.path],
                         c='k',
                         label="path risk value:\nmin: %.3f\nmax: %.3f\navg: %.3f" % (path_min1, path_max1, path_avg1))
                plt.colorbar()
                plt.axis("equal")
                plt.axis([area[0], area[1], area[2], area[3]])
                plt.legend(loc='upper right')
                plt.grid(True)
                plt.show()
                """

            if fail == flag:  # 说明有路径而且没有和grountruth碰撞
                tmp = [cc_rrt.get_chance_constrain_from_pu(node) for node in cc_rrt.path]  # 从这个可以看出这个finalpath里面都是一些节点
                print(tmp)
                print(len(cc_rrt.path))
                path_min = np.min(tmp)
                path_max = np.max(tmp)
                path_avg = np.average(tmp)
                """
                plt.axes([0.3, 0.1, 8 / 50, 8 / 10.55])
                plt.title("Without considering uncertainty\n Blue bounding box: Ground True" )
                plt.scatter([node.x for node in cc_rrt.node_list],
                            [node.y for node in cc_rrt.node_list],
                            s=3,
                            c=[node.cc for node in cc_rrt.node_list],
                            cmap='jet')
                plt.plot([node.x for node in cc_rrt.path],
                         [node.y for node in cc_rrt.path],
                         c='k',
                         label="path risk value:\nmin: %.3f\nmax: %.3f\navg: %.3f" % (path_min, path_max, path_avg))
                plt.colorbar()
                plt.axis("equal")
                plt.axis([area[0], area[1], area[2], area[3]])
                plt.legend(loc='upper right')
                plt.grid(True)
                plt.show()
                """
                pointnumber_list_100times.append(len(cc_rrt.path))
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


if __name__ == '__main__':
    main()