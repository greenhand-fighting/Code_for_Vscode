import sys
sys.path.append("../../..")
from 路径规划算法.CCRRT import *
import draw

def main():
    maxcc_list_100times=[]
    mincc_list_100times=[]
    avgcc_list_100times=[]
    long_list_100times = []
    pointnumber_list_100times = []
    fail = 0
    cycle_times = 0
    for i in range(100):
        print("Start -----------------------------1ccrrt!!!!!!!!!!!!!!!!!!!!!!!!!")
        cycle_times+=1

        area = [-7, 7, -7, 16]  # x-min x-max y-min y-max

        # Set Initial parameters
        start = [-1.75, 13.0, -np.pi / 2]
        goal = [4.5, -4, np.deg2rad(90.0)]
        car = Vehicle()

        # ====Search Path with CCRRT====
        # (x, y, vehicle_length, vehicle_width, radius [-pi, pi])
        # axis = length + sigma

        # -----在列表的后面直接将长半轴和短半轴加上去，不用再进行计算长短半轴了，因为rosbag中已经给出了长短半轴
        ganzhi_tuoyuan = [
            (1.361, 1.348, 2.36, 1.24, 2.237, 2.36 - 3.53 / 2, 1.24 - 1.59 / 2),
            (-5.181, -8.498, 2.41, 1.176, 1.616, 2.41 - 3.64 / 2, 1.176 - 1.61 / 2),
            (1.6467, -8.446, 2.945, 1.259, 1.5526, 2.945 - 4.377 / 2, 1.259 - 1.692 / 2),
            (11.0285, -1.6743, 2.899, 1.318, 3.109, 2.899 - 4.309 / 2, 1.318 - 1.645 / 2),
            (6.3373, -9.1127, 2.928, 1.377, 1.36976, 2.928 - 4.0051 / 2, 1.377 - 1.585 / 2),
            (-5.3691, 9.0414, 2.2515, 1.1159, 1.5587, 2.2515 - 3.3907 / 2, 1.1159 - 1.5249 / 2)
        ]
        ganzhi_kuang = [
            (1.361, 1.348, 3.53 / 2, 1.59 / 2, 2.237),
            (-5.181, -8.498, 3.64 / 2, 1.61 / 2, 1.616),
            (1.6467, -8.446, 4.377 / 2, 1.692 / 2, 1.5526),
            (11.0285, -1.6743, 4.309 / 2, 1.645 / 2, 3.109),
            (6.3373, -9.1127, 4.0051 / 2, 1.585 / 2, 1.36976),
            (-5.3691, 9.0414, 3.3907 / 2, 1.5249 / 2, 1.5587)
        ]
        obstacle_list_uncertainty = []
        for obs in ganzhi_kuang:
            dist = np.hypot(start[0] - obs[0], start[1] - obs[1])
            dist=25
            un = (un_generate(dist, 0.5, 1.0),  # sigma_ver
                  un_generate(dist, 0.3, 0.85),  # sigma_hor
                  un_generate(dist, 0.2, 0.1)  # sigma_radius
                  )
            obstacle_list_uncertainty.append(un)

        zhenzhi_kuang = [
            (-5.3, -8.5, 4.8557/2, 2.0323/2, -np.pi / 2),
            (-5.3, 9.0, 4.7175/2, 1.895/2, -np.pi / 2),
            (1.8, -8.5, 4.611/2, 2.2417/2, np.pi / 2),
            (1.4, 1.2, 4.974/2, 2.0384/2, -0.890120),
            (6.4, -9.3, 3.8058/2, 1.9703/2, 1.20428),
            (11.0, -1.8, 3.9877/2, 1.851/2, 0.0),
            (20.0, -5.3, 3.9877/2, 1.851/2, 0.0),
            # (-1.75, 13.0, 4.5135, 2.0068, -np.pi/2)
        ]#真值框，用于检测是不是和真值发生碰撞

        # (x, y, long_axis, short_axis, radius [-pi, pi])
        # vehicle_length = long_axis * 2
        # vehicle_width = short_axis * 2
        obstacle_list = obstacle_uncertainty_fusion(ganzhi_kuang,obstacle_list_uncertainty)
        #obstacle_list_from_pu=obstacle_uncertainty_fusion_from_pu(obstacle_list_gt_from_pu)
       # obstacle_list_for_pengzhuang_jiance=get_obstacle_list_for_pengzhuang_jiance(obstacle_list_gt)
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
        x = draw.Artist(draw_area=[-25, 25, -25, 25], rrt_object=cc_rrt, ganzhi_kuang=ganzhi_kuang,
                        zhenzhi_kuang=zhenzhi_kuang, is_clrrt=0, if_draw_tree=1)
        x.play_cross()
        if len(cc_rrt.path_end) == 0:  # 这里是没有找到路径
            #fail += 1
            print("because no path ::::fail=== %f " % fail)
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