import sys
sys.path.append("../../..")
from 路径规划算法.CLRRT import *
import draw
def main():
    area = [0, 15, 30, 100]  # x-min x-max y-min y-max
    start = [9.066, 67.66, np.deg2rad(90.0)]
    goal = [2.0, 100.0, np.deg2rad(90.0)]
    car = Vehicle()   # 这里也使用了导入的内容，就这样两个地方使用了import导入的内容
    obstacle_list_gt = [
        (1.929, 74.62, 4.29, 1.6, np.deg2rad(90.0),2.89, 1.21),
        (5.56, 94.53, 4.31, 1.66, np.deg2rad(90.0),2.96, 1.27),
        (9.128, 84.93, 3.62, 1.64, np.deg2rad(90.0),2.32, 1.14),
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
    obstacle_list = obstacle_uncertainty_fusion(obstacle_list_gt)
    obstacle_list1 = obstacle_uncertainty_fusion1(obstacle_list_gt)
    cc_rrt = CCRRT(
        car=car,
        start=start,
        goal=goal,
        rand_area=area,
        obstacle_list=obstacle_list,
        obstacle_list1 = obstacle_list1)
    cc_rrt.planning(animation=False)
    x=draw.Artist(draw_area=[0, 15, 30, 100], rrt_object=cc_rrt, ganzhi_kuang=obstacle_list,zhenzhi_kuang=ground_truth,is_clrrt=1,if_draw_tree=0)
    x.play_long()
