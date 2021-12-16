import sys
sys.path.append("../../..")
from 路径规划算法.CLRRT import *
from draw_legend import *


def main():
    maxcc_list_100times = []
    mincc_list_100times = []
    avgcc_list_100times = []
    long_list_100times = []
    pointnumber_list_100times = []
    fail = 0
    cycle_times = 0
    for i in range(100):
        cycle_times += 1
        print("Start -----------------------------------1clrrt100--sussess1  : only on gt box!!")
        area = [-5, 25, -7, 0]  # x-min x-max y-min y-max

        # Set Initial parameters
        start = [0.25, -2.0, -0.807054]
        goal = [20.0, -1.75, np.deg2rad(90.0)]
        car = Vehicle()

        # ====Search Path with CCRRT====
        # (x, y, vehicle_length, vehicle_width, radius [-pi, pi])
        # axis = length + sigma
        ganzhi_tuoyuan = [
            (19.9126, -5.1657, 2.7425, 1.319, 3.09614, 2.7425 - 3.996 / 2, 1.319 - 1.6033 / 2),
            (6.3358, -9.1656, 2.8204, 1.2810, 1.3434, 2.8204 - 4.2273 / 2, 1.2810 - 1.605 / 2),
            (10.76, -1.9073, 2.5059, 1.2517, 3.112, 2.5059 - 3.712 / 2, 1.2517 - 1.5643 / 2),
            (1.810, -8.54, 2.8743, 1.3125, 1.624, 2.8743 - 4.1776 / 2, 1.3125 - 1.5893 / 2),
        ]
        ganzhi_kuang = [
            (19.9126, -5.1657, 3.996/2, 1.6033/2, 3.09614),
            (6.3358, -9.1656, 4.2273/2, 1.605/2, 1.3434),
            (10.76, -1.9073, 3.712/2, 1.5643/2, 3.112),
            (1.810, -8.54, 4.1776/2, 1.5893/2, 1.624),
        ]
        zhenzhi_kuang = [
            (-5.3, -8.5, 4.8557 / 2, 2.0323 / 2, -np.pi / 2),
            (-5.3, 9.0, 4.7175 / 2, 1.895 / 2, -np.pi / 2),
            (1.8, -8.5, 4.611 / 2, 2.2417 / 2, np.pi / 2),
            (1.4, 1.2, 4.974 / 2, 2.0384 / 2, -0.890120),
            (6.4, -9.3, 3.8058 / 2, 1.9703 / 2, 1.20428),
            (11.0, -1.8, 3.9877 / 2, 1.851 / 2, 0.0),
            (20.0, -5.3, 3.9877 / 2, 1.851 / 2, 0.0),
            # (-1.75, 13.0, 4.5135, 2.0068, -np.pi/2)
        ]

        # Set Initial parameters
        cc_rrt = CCRRT(
            car=car,
            start=start,
            goal=goal,
            rand_area=area,
            obstacle_list=ganzhi_kuang,
            obstacle_list1=ganzhi_tuoyuan)
        # path = cc_rrt.planning(animation=False)
        cc_rrt.planning(animation=False)
        if len(cc_rrt.path_end) == 0:  # 这里是没有找到路径
            fail+=1
            print("because no path ::::fail=== %f " % fail)
        else:  # 这里是找到了路径，然后要判断它是不是压过了groundtruth
            flag = fail
            print('flag== %f' % flag)
            for node_in_final_path in cc_rrt.path:
                if cc_rrt.peng_zhuang_jian_ce(node_in_final_path,zhenzhi_kuang):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
                    print(node_in_final_path.x, node_in_final_path.y)
                    fail += 1
                    break
            if flag != fail:
                print('有路径，但是这个路径从groundtruth上踏过！！！！！！！！')
            if fail == flag:  # 说明有路径而且没有和grountruth碰撞
                tmp = cc_rrt.final_path_de_cc(cc_rrt.path)  # 从这个可以看出这个finalpath里面都是一些节点
                # print(tmp)
                print(len(cc_rrt.path))
                if np.max(tmp) >= 1 - cc_rrt.p_safe:
                    fail += 1
                    print("the max cc >= 1-Psafe !")
                else:
                    path_min = np.min(tmp)
                    path_max = np.max(tmp)
                    path_avg = np.average(tmp)
                    pointnumber_list_100times.append(len(cc_rrt.path))
                    sum = 0
                    for j in range(len(cc_rrt.path) - 1):
                        changdu, _ = cc_rrt.calc_distance_and_angle(cc_rrt.path[j], cc_rrt.path[j + 1])
                        sum = sum + changdu
                        print("sum: %f" % sum)
                    long_list_100times.append(sum)
                    maxcc_list_100times.append(path_max)
                    mincc_list_100times.append(path_min)
                    avgcc_list_100times.append(path_avg)

        print('*' * 100)
        print("n lun : %f" % i)
    pointnumber_list_100times = np.array(pointnumber_list_100times)
    long_list_100times = np.array(long_list_100times)
    maxcc_list_100times = np.array(maxcc_list_100times)
    mincc_list_100times = np.array(mincc_list_100times)
    avgcc_list_100times = np.array(avgcc_list_100times)
    print("max: %f" % np.average(maxcc_list_100times))
    print("avg: %f" % np.average(avgcc_list_100times))
    print("min: %f" % np.average(mincc_list_100times))
    print("points: %f" % np.average(pointnumber_list_100times))
    print("changdu: %f" % np.average(long_list_100times))
    print('*' * 100)
    print("fail=:= %f" % fail)
    print(cycle_times)
    success_rate = (cycle_times - fail) / (cycle_times)
    print("success rate: %f" % success_rate)

    # plt.savefig("cc-rrt-h-fun-3.png")


if __name__ == '__main__':
    main()
