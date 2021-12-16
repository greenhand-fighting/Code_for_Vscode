import sys
sys.path.append("../../..")
from 路径规划算法.PURRT import *
from draw_legend import *
"""
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
"""
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
    ax = plt.axes([0.3, 0.1, 8 / 25, 8 / 14])
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
    #    plot_arrow(object, node.x, node.y, node.yaw, fc='b')
   # plt.title("With considering uncertainty (represented by ellipse)\npurrt    P_safe:%f" % object.p_safe)
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
        a = gt[2]
        b = gt[3]
        d_a = a * (1 - b * np.sqrt((1 + np.tan(un[2])**2) / (b**2 + a**2 * np.tan(un[2])**2)))
        d_b = d_a / a * b
        obs.append((gt[0], gt[1], a + d_a + un[0], b + d_b + un[1], gt[4],d_a + un[0],d_b + un[1]))
        # obs.append((gt[0], gt[1], d_a + un[0], d_b + un[1], gt[4]))
    return obs

def draw_vehicle(obs_list):
    for obs in obs_list:
        w = obs[3]
        l = obs[2]
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
    for obs in obs_list:
       # x=0
        w = obs[3]
        l = obs[2]
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
        plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'k', color="green", lw='1')
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k', color="green", lw='1')
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k', color="green", lw='1')
        plt.plot([p3[0], p0[0]], [p3[1], p0[1]], 'k', color="green", lw='1')

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
    car = Vehicle()
    car.l_f = 4.51 / 2.0
    car.l_r = 4.51 / 2.0
    car.w = 2.0

    # Set Initial parameters
    start = [-1.75, 13.0, -np.pi / 2]
    goal = [4.5, -4, np.deg2rad(90.0)]

    # (x, y, vehicle_length, vehicle_width, radius [-pi, pi])

    ganzhi_kuang = [
        (1.361, 1.348, 3.53 / 2, 1.59 / 2, 2.237),
        (-5.181, -8.498, 3.64 / 2, 1.61 / 2, 1.616),
        (1.6467, -8.446, 4.377 / 2, 1.692 / 2, 1.5526),
        (11.0285, -1.6743, 4.309 / 2, 1.645 / 2, 3.109),
        (6.3373, -9.1127, 4.0051 / 2, 1.585 / 2, 1.36976),
        (-5.3691, 9.0414, 3.3907 / 2, 1.5249 / 2, 1.5587)
    ]
    # (x, y, long_axis, short_axis, radius [-pi, pi], sigma_ver, sigma_hor)
    # 感知椭圆：
    ganzhi_tuoyuan = [
        (1.361, 1.348, 2.36, 1.24, 2.237, 2.36 - 3.53 / 2, 1.24 - 1.59 / 2),
        (-5.181, -8.498, 2.41, 1.176, 1.616, 2.41 - 3.64 / 2, 1.176 - 1.61 / 2),
        (1.6467, -8.446, 2.945, 1.259, 1.5526, 2.945 - 4.377 / 2, 1.259 - 1.692 / 2),
        (11.0285, -1.6743, 2.899, 1.318, 3.109, 2.899 - 4.309 / 2, 1.318 - 1.645 / 2),
        (6.3373, -9.1127, 2.928, 1.377, 1.36976, 2.928 - 4.0051 / 2, 1.377 - 1.585 / 2),
        (-5.3691, 9.0414, 2.2515, 1.1159, 1.5587, 2.2515 - 3.3907 / 2, 1.1159 - 1.5249 / 2)
    ]

    zhenzhi_kuang = [
        (-5.3, -8.5, 4.8557/2, 2.0323/2, -np.pi / 2),
        (-5.3, 9.0, 4.7175/2, 1.895/2, -np.pi / 2),
        (1.8, -8.5, 4.611/2, 2.2417/2, np.pi / 2),
        (1.4, 1.2, 4.974/2, 2.0384/2, -0.890120),
        (6.4, -9.3, 3.8058/2, 1.9703/2, 1.20428),
        (11.0, -1.8, 3.9877/2, 1.851/2, 0.0),
        (20.0, -5.3, 3.9877/2, 1.851/2, 0.0),
        # (-1.75, 13.0, 4.5135, 2.0068, -np.pi/2)
    ]


    # ganzhi_tuoyuan = [
    #     (1.361, 1.348, 2.36, 1.24, 2.237),
    #     (-5.181, -8.498, 2.41, 1.176, 1.616),
    #     (1.6467, -8.446, 2.945, 1.259, 1.5526),
    #     (11.0285, -1.6743, 2.899, 1.318, 3.109),
    #     (6.3373, -9.1127, 2.928, 1.377, 1.36976),
    #     (-5.3691, 9.0414, 2.2515, 1.1159, 1.5587)
    # ]

    area = [-7, 7, -7, 16]  # x-min x-max y-min y-max
    cc_rrt = CCRRT(   # 需要感知框 和 感知椭圆
        car=car,
        start=start,
        goal=goal,
        rand_area=area,
        obstacle_list=ganzhi_tuoyuan,  # 感知椭圆
       # obstacle_list_from_pu=ganzhi_tuoyuan,
        obstacle_list_for_pengzhuang_jiance=ganzhi_kuang
        )
    #cc_rrt.p_safe = 0.99
    #cc_rrt.max_n_node = 1500
    #cc_rrt.draw_tree = False
    cc_rrt.planning(animation=False)

    area = [-25, 25, -25, 25]  # x-min x-max y-min y-max
    plt.figure(1, figsize=(6, 6))
    draw_graph(cc_rrt)
    draw_path(cc_rrt)
    draw_vehicle(ganzhi_kuang)
    draw_ground_true(zhenzhi_kuang)
    draw_carsize_of_final_path(cc_rrt.path)
    plt.axis("equal")
    plt.axis([area[0], area[1], area[2], area[3]])
    # 画路
    plt.plot([7, 7], [12, 25], color="grey")
    plt.plot([12, 25], [7, 7], color="grey")
    plt.plot([-7, -7], [12, 25], color="grey")
    plt.plot([-12, -25], [7, 7], color="grey")
    plt.plot([-7, -7], [-12, -25], color="grey")
    plt.plot([-12, -25], [-7, -7], color="grey")
    plt.plot([7, 7], [-12, -25], color="grey")
    plt.plot([12, 25], [-7, -7], color="grey")
    plt.plot([7, 12], [-12, -7], color="grey")
    plt.plot([-7, -12], [-12, -7], color="grey")
    plt.plot([-7, -12], [12, 7], color="grey")
    plt.plot([7, 12], [12, 7], color="grey")
    plt.plot([-3.5, -3.5], [7, 25], "--", color="grey")
    plt.plot([0, 0], [7, 25], color="grey")
    plt.plot([3.5, 3.5], [7, 25], "--", color="grey")
    plt.plot([7, 25], [3.5, 3.5], "--", color="grey")
    plt.plot([7, 25], [0, 0], color="grey")
    plt.plot([7, 25], [-3.5, -3.5], "--", color="grey")
    plt.plot([3.5, 3.5], [-7, -25], "--", color="grey")
    plt.plot([0, 0], [-7, -25], color="grey")
    plt.plot([-3.5, -3.5], [-7, -25], "--", color="grey")
    plt.plot([-7, -25], [3.5, 3.5], "--", color="grey")
    plt.plot([-7, -25], [0, 0], color="grey")
    plt.plot([-7, -25], [-3.5, -3.5], "--", color="grey")
    # 画规划空间范围
    plt.plot([-7, -7], [16, -1.75], "--", color="orange")
    plt.plot([-7, -1.75], [-1.75, -7], "--", color="orange")
    plt.plot([-1.75, 23], [-7, -7], "--", color="orange")
    plt.plot([23, 23], [-7, 0], "--", color="orange")
    plt.plot([23, 7], [0, 0], "--", color="orange")
    plt.plot([7, 0], [0, 7], "--", color="orange")
    plt.plot([0, 0], [7, 16], "--", color="orange")
    plt.plot([0, -7], [16, 16], "--", color="orange")

    #area = [-10, 10, -10, 20]
    # 画图例的-------------------------
    c = mpatches.Circle((0.5, 0.5), 0.25, facecolor="#1f77b4",
                        edgecolor="#1f77b4", linewidth=3)
    # plt.gca().add_patch(c)

    # 第一个图例
    first_legend = plt.legend([AnyObject(), AnyObject2(), AnyObject3(), c],
                              ['ego_car(trajectory)', "prediction bounding box", "ground truth",
                               "spatial uncertainty"],
                              handler_map={AnyObject: AnyObjectHandler()
                                           #  , AnyObject1: AnyObjectHandler1()
                                  , AnyObject2: AnyObjectHandler2()
                                  , AnyObject3: AnyObjectHandler3()
                                  , mpatches.Circle: HandlerEllipse()}
                              , loc='upper right', fontsize=15
                              # , bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0.
                              )

    plt.gca().add_artist(first_legend)

    #     line2, = plt.plot([3, 2, 1], label="Line 2", linewidth=4)
    start_star, = plt.plot(-100, 0, "*y", markersize=10, label='start')
    goal_star, = plt.plot(-100, 0, "*g", markersize=10, label='goal')
    mid_star, = plt.plot(-100, 0, "*r", markersize=10, label='path point')
    # tree_node_star, = plt.plot(0, 0, "*b", markersize=10, label='tree_node')
    # 第二个图例
    plt.legend(handles=[
        # line2
        start_star
        , goal_star
        , mid_star
        #    , tree_node_star
    ]
        , loc='lower left'
        , fontsize=15
        #    , bbox_to_anchor=(1.01, 0), loc='lower left', borderaxespad=0.
    )
    plt.show()
    # 画图例完成---------------
    plt.figure(2, figsize=(6, 6))
    draw_graph(cc_rrt)
    draw_vehicle(ganzhi_kuang)
    draw_ground_true(zhenzhi_kuang)

    tmp = [node.cc for node in cc_rrt.path]  # 从这个可以看出这个finalpath里面都是一些节点
    # print(tmp)
    print(len(cc_rrt.path))
    path_min = np.min(tmp)
    path_max = np.max(tmp)
    path_avg = np.average(tmp)

    # plt.axes([0.3, 0.1, 8 / 50, 8 / 10.55])
    plt.scatter([node.x for node in cc_rrt.node_list],
                [node.y for node in cc_rrt.node_list],
                s=3,
                c=[node.cc for node in cc_rrt.node_list],
                cmap='jet')
    plt.plot([node.x for node in cc_rrt.path],
             [node.y for node in cc_rrt.path],
             c='k',
             label="path risk value:\nmin: %.6f\nmax: %.6f\navg: %.6f" % (path_min, path_max, path_avg))
    plt.colorbar()
    plt.axis("equal")
    plt.axis([area[0], area[1], area[2], area[3]])
    plt.legend(loc='upper right',fontsize=15)
   # plt.grid(True)
    plt.plot([7, 7], [12, 25], color="grey")
    plt.plot([12, 25], [7, 7], color="grey")
    plt.plot([-7, -7], [12, 25], color="grey")
    plt.plot([-12, -25], [7, 7], color="grey")
    plt.plot([-7, -7], [-12, -25], color="grey")
    plt.plot([-12, -25], [-7, -7], color="grey")
    plt.plot([7, 7], [-12, -25], color="grey")
    plt.plot([12, 25], [-7, -7], color="grey")
    plt.plot([7, 12], [-12, -7], color="grey")
    plt.plot([-7, -12], [-12, -7], color="grey")
    plt.plot([-7, -12], [12, 7], color="grey")
    plt.plot([7, 12], [12, 7], color="grey")
    plt.plot([-3.5, -3.5], [7, 25], "--", color="grey")
    plt.plot([0, 0], [7, 25], color="grey")
    plt.plot([3.5, 3.5], [7, 25], "--", color="grey")
    plt.plot([7, 25], [3.5, 3.5], "--", color="grey")
    plt.plot([7, 25], [0, 0], color="grey")
    plt.plot([7, 25], [-3.5, -3.5], "--", color="grey")
    plt.plot([3.5, 3.5], [-7, -25], "--", color="grey")
    plt.plot([0, 0], [-7, -25], color="grey")
    plt.plot([-3.5, -3.5], [-7, -25], "--", color="grey")
    plt.plot([-7, -25], [3.5, 3.5], "--", color="grey")
    plt.plot([-7, -25], [0, 0], color="grey")
    plt.plot([-7, -25], [-3.5, -3.5], "--", color="grey")
    # 画规划空间范围
    plt.plot([-7, -7], [16, -1.75], "--", color="orange")
    plt.plot([-7, -1.75], [-1.75, -7], "--", color="orange")
    plt.plot([-1.75, 23], [-7, -7], "--", color="orange")
    plt.plot([23, 23], [-7, 0], "--", color="orange")
    plt.plot([23, 7], [0, 0], "--", color="orange")
    plt.plot([7, 0], [0, 7], "--", color="orange")
    plt.plot([0, 0], [7, 16], "--", color="orange")
    plt.plot([0, -7], [16, 16], "--", color="orange")

    plt.show()


if __name__ == "__main__":
    main()
