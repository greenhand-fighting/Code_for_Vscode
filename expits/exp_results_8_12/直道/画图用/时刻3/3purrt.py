import sys
sys.path.append("../../..")
print(sys.path)
from 路径规划算法.PURRT import *
from draw_legend import  *

def plot_arrow(object, x, y, yaw, length=0.5, width=0.25, fc="r", ec="k"):
    """
    Plot arrow
    """
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(object,ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def draw_graph(object, rnd=None):
    plt.clf()
    #ax = plt.axes()
    ax = plt.axes([0.3, 0.1, 8 / 55, 8 / 10])
    # _, ax = plt.subplots()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])
    for obs in object.obstacle_list:
        ellipse = Ellipse((obs[0], obs[1]), obs[2] * 2.0, obs[3] * 2.0, math.degrees(obs[4]),color='#1f77b4')
        ax.add_patch(ellipse)
    #ziche = Rectangle((8.0+1.9/2, 1.11-2.9/2), 2.9, 1.9 , math.degrees(np.deg2rad(130.0)))
    # 8.0, 1.11, 2.9, 1.9, np.deg2rad(130.0)
    #ax.add_patch(ziche)
    if rnd is not None:
        plt.plot(rnd.x, rnd.y, "^k")
    # ----不画树节点
    # for node in object.node_list:
    #     plt.plot(node.x, node.y, "*b")
    #     plot_arrow(object, node.x, node.y, node.yaw, fc='b')
  #  plt.title("With considering uncertainty (represented by ellipse)\npurrt    P_safe:%f" % object.p_safe)
    plt.plot(object.start.x, object.start.y, "xr")
    plot_arrow(object,object.start.x, object.start.y, object.start.yaw, fc='r')
    plt.plot(object.end.x, object.end.y, "xg")
    plot_arrow(object,object.end.x, object.end.y, object.end.yaw, fc='g')
    plt.axis("equal")
    plt.axis([object.min_rand_x, object.max_rand_x, object.min_rand_y, object.max_rand_y])
   # plt.grid(True)
    plt.pause(0.01)

def draw_path(object):
    for node in object.path:
        plt.plot(node.x, node.y, "*r")
        plot_arrow(object,node.x, node.y, node.yaw, fc='r')
   # plt.title("With considering uncertainty (represented by ellipse)\npurrt    P_safe:%f" % object.p_safe)
    plt.plot(object.start.x, object.start.y, "*y", markersize=10, label='start')
    plot_arrow(object,object.start.x, object.start.y, object.start.yaw, fc='y')
    plt.plot(object.end.x, object.end.y, "*g", markersize=10, label='goal')
    plot_arrow(object,object.end.x, object.end.y, object.end.yaw, fc='g')
    plt.legend(loc='upper right')

def obstacle_uncertainty_fusion(gts):
    obs = []
    for gt in gts:
        obs.append((gt[0], gt[1], gt[5], gt[6], gt[4]))  #注意，这里的列表中表示的椭圆，使用长短半轴表示的，并不是整个轴！！！
    return obs


def draw_vehicle(obs_list):
    for obs in obs_list:
        w = obs[3] / 2.0
        l = obs[2] / 2.0
        x = obs[0]
        y = obs[1]
        yaw = obs[4]
        p0 = [
            x + l * math.cos(yaw) + w * math.sin(yaw),
            y + l * math.sin(yaw) - w * math.cos(yaw)
        ]
        p1 = [
            x + l * math.cos(yaw) - w * math.sin(yaw),
            y + l * math.sin(yaw) + w * math.cos(yaw)
        ]
        p2 = [
            x - l * math.cos(yaw) - w * math.sin(yaw),
            y - l * math.sin(yaw) + w * math.cos(yaw)
        ]
        p3 = [
            x - l * math.cos(yaw) + w * math.sin(yaw),
            y - l * math.sin(yaw) - w * math.cos(yaw)
        ]
        plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'k', color="r")
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k', color="r")
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k', color="r")
        plt.plot([p3[0], p0[0]], [p3[1], p0[1]], 'k', color="r")

def draw_ground_true(obs_list):
    x1=True
    for obs in obs_list:
       # x=0
        w = obs[3] / 2.0
        l = obs[2] / 2.0
        x = obs[0]
        y = obs[1]
        yaw = obs[4]
        p0 = [
            x + l * math.cos(yaw) + w * math.sin(yaw),
            y + l * math.sin(yaw) - w * math.cos(yaw)
        ]
        p1 = [
            x + l * math.cos(yaw) - w * math.sin(yaw),
            y + l * math.sin(yaw) + w * math.cos(yaw)
        ]
        p2 = [
            x - l * math.cos(yaw) - w * math.sin(yaw),
            y - l * math.sin(yaw) + w * math.cos(yaw)
        ]
        p3 = [
            x - l * math.cos(yaw) + w * math.sin(yaw),
            y - l * math.sin(yaw) - w * math.cos(yaw)
        ]
        if (x1):
            pass
            # plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'k', color="orange", lw='1')
            # plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k', color="orange", lw='1')
            # plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k', color="orange", lw='1')
            # plt.plot([p3[0], p0[0]], [p3[1], p0[1]], 'k', color="orange", lw='1')
        else:
            plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'k', color="green", lw='1')
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k', color="green", lw='1')
            plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k', color="green", lw='1')
            plt.plot([p3[0], p0[0]], [p3[1], p0[1]], 'k', color="green", lw='1')
        x1 = False

def draw_carsize_of_final_path(obs_list):
    for obs in obs_list:
       # x=0
        w = 2 / 2.0
        l = 4.51 / 2.0
        x = obs.x
        y = obs.y
        yaw = obs.yaw
        p0 = [
            x + l * math.cos(yaw) + w * math.sin(yaw),
            y + l * math.sin(yaw) - w * math.cos(yaw)
        ]
        p1 = [
            x + l * math.cos(yaw) - w * math.sin(yaw),
            y + l * math.sin(yaw) + w * math.cos(yaw)
        ]
        p2 = [
            x - l * math.cos(yaw) - w * math.sin(yaw),
            y - l * math.sin(yaw) + w * math.cos(yaw)
        ]
        p3 = [
            x - l * math.cos(yaw) + w * math.sin(yaw),
            y - l * math.sin(yaw) - w * math.cos(yaw)
        ]
        plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'k', color="black",lw='1')
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k', color="black",lw='1')
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k', color="black",lw='1')
        plt.plot([p3[0], p0[0]], [p3[1], p0[1]], 'k', color="black",lw='1')


def main():
    print("Start " + __file__)

    area = [0, 15, 30, 100]  # x-min x-max y-min y-max

    # Set Initial parameters
    start = [1.86, 61.3, np.deg2rad(90.0)]
    goal = [12.0, 95.0, np.deg2rad(90.0)]
    car = Vehicle()

    # ====Search Path with CCRRT====
    # (x, y, vehicle_length, vehicle_width, radius [-pi, pi])
    # axis = length + sigma

    # -----在列表的后面直接将长半轴和短半轴加上去，不用再进行计算长短半轴了，因为rosbag中已经给出了长短半轴
    obstacle_list_gt = [
        (1.83, 74.57, 3.82, 1.59, np.deg2rad(90.0), 2.48, 1.13),
        # (5.36, 44.96, 3.96, 1.59, np.deg2rad(100.0),2.61, 1.19),
        (5.44, 65.03, 3.56, 1.54, np.deg2rad(89.0), 2.34, 1.11),
        (5.41, 94.42, 4.28, 1.65, np.deg2rad(90.0), 3.15, 1.37),
        # (9.0, 54.93, 4.08, 1.59, np.deg2rad(90.0), 2.6, 1.14),
        (9.124, 84.98, 3.93, 1.66, np.deg2rad(90.0), 2.63, 1.23),
        # (12.73, 44.84, 3.4, 1.5, np.deg2rad(89.90), 2.28, 1.13),
        # (12.55, 64.96, 4.45, 1.71, np.deg2rad(90.0), 2.83, 1.18),
    ]

    # sigam_ver, sigma_hor, sigma_radius

    ground_true_obs_list = [
        (1.86, 61.3, 4.51, 2.0, np.deg2rad(90.0)),
        (1.9, 75.0, 4.72, 1.89, np.deg2rad(90.0)),
        (5.5, 45.0, 4.19, 1.82, np.deg2rad(90.0)),
        (5.5, 65.0, 4.79, 2.16, np.deg2rad(90.0)),
        (5.5, 95.0, 5.36, 2.03, np.deg2rad(90.0)),
        (9.1, 55.0, 4.86, 2.03, np.deg2rad(90.0)),
        (9.1, 85.0, 3.99, 1.85, np.deg2rad(90.0)),
        (12.7, 45.0, 4.18, 1.99, np.deg2rad(90.0)),
        (12.7, 65.0, 4.61, 2.24, np.deg2rad(90.0)),
    ]

    # (x, y, long_axis, short_axis, radius [-pi, pi])
    # vehicle_length = long_axis * 2
    # vehicle_width = short_axis * 2
    obstacle_list = obstacle_uncertainty_fusion(obstacle_list_gt)
    obstacle_list_for_pengzhuang_jiance = get_obstacle_list_for_pengzhuang_jiance(obstacle_list_gt)

    # Set Initial parameters
    cc_rrt = CCRRT(
        car=car,
        start=start,
        goal=goal,
        rand_area=area,
        obstacle_list=obstacle_list,
        obstacle_list_for_pengzhuang_jiance=obstacle_list_for_pengzhuang_jiance)
    # path = cc_rrt.planning(animation=False)
    cc_rrt.planning(animation=False)
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

    draw_graph(cc_rrt)
    draw_path(cc_rrt)
    draw_vehicle(obstacle_list_gt)
    draw_ground_true(ground_true_obs_list)
    draw_carsize_of_final_path(cc_rrt.path)
    plt.axis([area[0], area[1], area[2], area[3]])
    # 画路
    plt.plot([0, 0], [30, 105], color="grey")
    plt.plot([14.4, 14.4], [30, 105], color="grey")
    plt.plot([3.6, 3.6], [30, 105], "--", color="grey")
    plt.plot([7.2, 7.2], [30, 105], "--", color="grey")
    plt.plot([10.8, 10.8], [30, 105], "--", color="grey")
    # 画图例的-------------------------
    c = mpatches.Circle((0.5, 0.5), 0.25, facecolor="#1f77b4",
                        edgecolor="#1f77b4", linewidth=3)
    # plt.gca().add_patch(c)

    # 第一个图例
    first_legend = plt.legend([AnyObject(), AnyObject2(), AnyObject3(), c],
                              ['ego_car(trajectory)', "prediction bounding box", "ground truth",
                               "spatial uncertainty"],
                              handler_map={AnyObject: AnyObjectHandler()
                               #   , AnyObject1: AnyObjectHandler1()
                                  , AnyObject2: AnyObjectHandler2()
                                  , AnyObject3: AnyObjectHandler3()
                                  , mpatches.Circle: HandlerEllipse()}
                              , loc='upper right', fontsize=12
                              # , bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0.
                              )

    plt.gca().add_artist(first_legend)

    #     line2, = plt.plot([3, 2, 1], label="Line 2", linewidth=4)
    start_star, = plt.plot(0, 0, "*y", markersize=10, label='start')
    goal_star, = plt.plot(0, 0, "*g", markersize=10, label='goal')
    mid_star, = plt.plot(0, 0, "*r", markersize=10, label='path point')
    #tree_node_star, = plt.plot(0, 0, "*b", markersize=10, label='tree_node')
    # 第二个图例
    plt.legend(handles=[
        # line2
        start_star
        , goal_star
        , mid_star
    #    , tree_node_star
    ]
        , loc='lower left'
        , fontsize=12
        #    , bbox_to_anchor=(1.01, 0), loc='lower left', borderaxespad=0.
    )
    plt.show()
    # 画图例完成---------------
    plt.figure(2)
    cc_rrt.draw_graph()
    draw_vehicle(obstacle_list_gt)
    draw_ground_true(ground_true_obs_list)
    tmp = [node.cc for node in cc_rrt.path]
    path_min = np.min(tmp)
    path_max = np.max(tmp)
    path_avg = np.average(tmp)

   # plt.axes([0.3, 0.1, 8 / 50, 8 / 10.55])
  #  plt.title("With considering uncertainty (represented by ellipse)\npurrt    P_safe:%f" % cc_rrt.p_safe)
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
    plt.axis([area[0], area[1], area[2], area[3]])
    # 画路
    plt.plot([0, 0], [30, 105], color="grey")
    plt.plot([14.4, 14.4], [30, 105], color="grey")
    plt.plot([3.6, 3.6], [30, 105], "--", color="grey")
    plt.plot([7.2, 7.2], [30, 105], "--", color="grey")
    plt.plot([10.8, 10.8], [30, 105], "--", color="grey")
    plt.legend(loc='upper right')
   # plt.grid(True)
    plt.show()


        # plt.savefig("cc-rrt-h-fun-3.png")




if __name__ == '__main__':
    main()
