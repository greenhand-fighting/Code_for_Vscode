import sys
sys.path.append("../../..")
from 路径规划算法.CCRRT import *
import matplotlib.patches as mpatches
from matplotlib.legend_handler import HandlerPatch
##画图例需要
class AnyObject:
    pass

class AnyObject1:
    pass

class AnyObject2:
    pass

class AnyObject3:
    pass

class AnyObjectHandler:
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width, height = handlebox.width, handlebox.height
        patch = mpatches.Rectangle([x0, y0], width, height, facecolor='white',
                                   edgecolor='black', lw=1,
                                   transform=handlebox.get_transform())
        handlebox.add_artist(patch)
        return patch


class AnyObjectHandler1:
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width, height = handlebox.width, handlebox.height
        patch = mpatches.Rectangle([x0, y0], width, height, facecolor='white',
                                   edgecolor='orange', lw=1,
                                   transform=handlebox.get_transform())
        handlebox.add_artist(patch)
        return patch


# 预测
class AnyObjectHandler2:
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width, height = handlebox.width, handlebox.height
        patch = mpatches.Rectangle([x0, y0], width, height, facecolor='white',
                                   edgecolor='red', lw=1,
                                   transform=handlebox.get_transform())
        handlebox.add_artist(patch)
        return patch


# green 真值
class AnyObjectHandler3:
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width, height = handlebox.width, handlebox.height
        patch = mpatches.Rectangle([x0, y0], width, height, facecolor='white',
                                   edgecolor='green', lw=1,
                                   transform=handlebox.get_transform())
        handlebox.add_artist(patch)
        return patch


class HandlerEllipse(HandlerPatch):
    def create_artists(self, legend, orig_handle,
                       xdescent, ydescent, width, height, fontsize, trans):
        center = 0.5 * width - 0.5 * xdescent, 0.5 * height - 0.5 * ydescent
        p = mpatches.Ellipse(xy=center, width=width + xdescent,
                             height=height + ydescent)
        self.update_prop(p, orig_handle, legend)
        p.set_transform(trans)
        return [p]
##画图例需要

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
  #  for node in object.node_list:
   #     plt.plot(node.x, node.y, "*b")
   #     plot_arrow(object, node.x, node.y, node.yaw, fc='b')
 #   plt.title("With considering uncertainty (represented by ellipse)\npurrt    P_safe:%f" % object.p_safe)
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
  #  plt.title("With considering uncertainty (represented by ellipse)\npurrt    P_safe:%f" % object.p_safe)
    plt.plot(object.start.x, object.start.y, "*y", markersize=10, label='start')
    plot_arrow(object,object.start.x, object.start.y, object.start.yaw, fc='y')
    plt.plot(object.end.x, object.end.y, "*g", markersize=10, label='goal')
    plot_arrow(object,object.end.x, object.end.y, object.end.yaw, fc='g')
    plt.legend(loc='upper right')

def obstacle_uncertainty_fusion(gts, uncertainties):
    obs = []
    for gt, un in zip(gts, uncertainties):
        a = gt[2] / 2.0
        b = gt[3] / 2.0
        d_a = a * (1 - b * np.sqrt((1 + np.tan(un[2])**2) / (b**2 + a**2 * np.tan(un[2])**2)))
        d_b = d_a / a * b
        obs.append((gt[0], gt[1], a + d_a + un[0], b + d_b + un[1], gt[4]))
        # obs.append((gt[0], gt[1], d_a + un[0], d_b + un[1], gt[4]))
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

dis_threshold=25.5
def un_generate(dis, p1, p2):
    base = dis / dis_threshold
    sigma_base = np.random.normal(0.0, base * p1)
    #print(sigma_base)
    return (base + sigma_base) * p2

def main():
    print("Start " + __file__)

    area = [0, 15, 30, 100]  # x-min x-max y-min y-max

    # Set Initial parameters
    start = [9.066, 67.66, np.deg2rad(90.0)]
    goal = [2.0, 100.0, np.deg2rad(90.0)]
    car = Vehicle()

    # ====Search Path with CCRRT====
    # (x, y, vehicle_length, vehicle_width, radius [-pi, pi])
    # axis = length + sigma
    # -----在列表的后面直接将长半轴和短半轴加上去，不用再进行计算长短半轴了，因为rosbag中已经给出了长短半轴
    obstacle_list_gt_from_pu = [
        (1.929, 74.62, 4.29, 1.6, np.deg2rad(90.0), 2.89, 1.21),
        # (5.36, 44.96, 3.96, 1.59, np.deg2rad(100.0),2.61, 1.19),
        # (5.44, 65.03, 3.56, 1.54, np.deg2rad(89.0), 2.34, 1.11),
        (5.56, 94.53, 4.31, 1.66, np.deg2rad(90.0), 2.96, 1.27),
        # (9.0, 54.93, 4.08, 1.59, np.deg2rad(90.0), 2.6, 1.14),
        (9.128, 84.93, 3.62, 1.64, np.deg2rad(90.0), 2.32, 1.14),
        # (12.73, 44.84, 3.4, 1.5, np.deg2rad(89.90), 2.28, 1.13),
        # (12.55, 64.96, 4.45, 1.71, np.deg2rad(90.0), 2.83, 1.18),
    ]
    # -----在列表的后面直接将长半轴和短半轴加上去，不用再进行计算长短半轴了，因为rosbag中已经给出了长短半轴
    obstacle_list_gt = [
        (1.929, 74.62, 4.29, 1.6, np.deg2rad(90.0)),
        # (5.36, 44.96, 3.96, 1.59, np.deg2rad(100.0),2.61, 1.19),
        # (5.44, 65.03, 3.56, 1.54, np.deg2rad(89.0), 2.34, 1.11),
        (5.56, 94.53, 4.31, 1.66, np.deg2rad(90.0)),
        # (9.0, 54.93, 4.08, 1.59, np.deg2rad(90.0), 2.6, 1.14),
        (9.128, 84.93, 3.62, 1.64, np.deg2rad(90.0)),
        # (12.73, 44.84, 3.4, 1.5, np.deg2rad(89.90), 2.28, 1.13),
        # (12.55, 64.96, 4.45, 1.71, np.deg2rad(90.0), 2.83, 1.18),
    ]
    obstacle_list_uncertainty = []
    for obs in obstacle_list_gt:
        dist = np.hypot(start[0] - obs[0], start[1] - obs[1])
        dist=25
        un = (un_generate(dist, 0.22, 0.92),  # sigma_ver
              un_generate(dist, 0.11, 0.82),  # sigma_hor
              un_generate(dist, 0.055, 0.077)  # sigma_radius
              )
        obstacle_list_uncertainty.append(un)

    # sigam_ver, sigma_hor, sigma_radius

    ground_true_obs_list = [
        (9.066, 67.66, 4.51, 2.0, np.deg2rad(90.0)),
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
    obstacle_list = obstacle_uncertainty_fusion(obstacle_list_gt, obstacle_list_uncertainty) # 人工椭圆
    obstacle_list_from_pu = obstacle_uncertainty_fusion_from_pu(obstacle_list_gt_from_pu)   # 这是检测成功率使用的，这个函数在ccrrt中， 感知椭圆
    obstacle_list_for_pengzhuang_jiance = get_obstacle_list_for_pengzhuang_jiance(obstacle_list_gt)  # 这个是树扩展的时候的一个判断条件，这个函数也在模板中，感知框,半长半短

    cc_rrt = CCRRT(
        car=car,
        start=start,
        goal=goal,
        rand_area=area,
        obstacle_list=obstacle_list,
        obstacle_list_from_pu=obstacle_list_from_pu,
        obstacle_list_for_pengzhuang_jiance=obstacle_list_for_pengzhuang_jiance)
    # path = cc_rrt.planning(animation=False)
    cc_rrt.planning(animation=False)

    #开始绘制第一章图
    draw_graph(cc_rrt)
    draw_path(cc_rrt)
    draw_vehicle(obstacle_list_gt)
    draw_ground_true(ground_true_obs_list)
    draw_carsize_of_final_path(cc_rrt.path)
    plt.axis([area[0], area[1], area[2], area[3]+10])
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
                              ['ego_car(trajectory)', "starting position", "ground truth",
                               "Spatial uncertainty"],
                              handler_map={AnyObject: AnyObjectHandler()
                           #       , AnyObject1: AnyObjectHandler1()
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
    mid_star, = plt.plot(0, 0, "*r", markersize=10, label='Path point')
  #  tree_node_star, = plt.plot(0, 0, "*b", markersize=10, label='tree_node')
    # 第二个图例
    plt.legend(handles=[
        # line2
        start_star
        , goal_star
        , mid_star
      #  , tree_node_star
        ]
        , loc='lower left'
        , fontsize=12
        #    , bbox_to_anchor=(1.01, 0), loc='lower left', borderaxespad=0.
    )
    plt.show()
    # 画图例完成---------------
    # 第一张图绘制完成

    plt.figure(2)
    draw_graph(cc_rrt)
    draw_vehicle(obstacle_list_gt)
    draw_ground_true(ground_true_obs_list)
    tmp = [node.cc for node in cc_rrt.path]
    path_min = np.min(tmp)
    path_max = np.max(tmp)
    path_avg = np.average(tmp)
    # 绘制两个张图
  #  plt.axes([0.3, 0.1, 8 / 50, 8 / 10.55])
   # plt.title("With considering uncertainty (represented by ellipse)\nccrrt    P_safe:%f" % cc_rrt.p_safe)
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
    # 第二张图绘制完成




if __name__ == '__main__':
    main()