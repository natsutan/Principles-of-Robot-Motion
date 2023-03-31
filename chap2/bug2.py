# BUG アルゴリズム
import math

from common.world import world1, world2
from common.robot import Robot
from common.geometry import is_crossed_line_segment

class BugRobot2(Robot):

    ST_IDLE = 0
    ST_MOVE_STRAIGHT = 1
    ST_MOVE_ROTATE_RIGHT = 2
    ST_STOP = -1

    def __init__(self):
        super().__init__()
        self.state = self.ST_IDLE
        # ゴールまでの直線と軌道が交わった座標を表すList
        self.cross_points = []

    def set_start_goal_from_world(self):
        super().set_start_goal_from_world()
        # 回転を開始した座標のインデックスを保存する変数
        self.rotation_start_index = 0


    def calc_theta_to_goal(self):
        x, y, _ = self.pose()
        x = self.goal[0] - x
        y = self.goal[1] - y
        theta = math.atan2(y, x)
        return theta

    def calc_next_theta(self):
        """
        障害物をよける角度を見つける。
        正面先を調べて場合分け
        ・正面に進むとぶつかるとき、左側でぶつからない角度を探す
        ・正面に進んでもぶつからないとき、右側のぶつからない角度を探す
        """
        _, _, theta = self.pose()
        theta_ori = theta
        next_x, next_y = self.next_pos()
        if self.collision_detection((next_x, next_y)):
            sign = 1
        else:
            sign = -1


        if sign == 1:
            # 進路に障害物があるので、障害物が無くなるまでループ
            while True:
                theta = theta + sign * self.DEFAULT_ROTATION_UNIT
                nx, ny = self.next_pos(theta)
                if not self.collision_detection((nx, ny)):
                    return theta

        # sign == -1
        while True:
            new_theta = theta - self.DEFAULT_ROTATION_UNIT
            nx, ny = self.next_pos(new_theta)
            if self.collision_detection((nx, ny)):
                # 右側にぶつかるので一つ前の値を返す。
                return theta
            theta = new_theta

    # 軌道が１度通ったことがあるかを判定する関数
    def visited_pos(self, pos):
        if len(self.trajectory) - self.rotation_start_index < 2:
            return None

        for pi in range(self.rotation_start_index, len(self.trajectory) - 1):
            p = self.trajectory[pi]
            if math.fabs(p[0] - pos[0]) < self.EPS and math.fabs(p[1] - pos[1]) < self.EPS:
                return pi
        return None

    # 軌道からゴールに一番近い場所を見つける
    def find_nearest_goal(self):
        min_dist = 10000000000000
        min_index = self.rotation_start_index
        for i, p in enumerate(self.trajectory[self.rotation_start_index:]):
            dist = math.sqrt((self.goal[0] - p[0]) ** 2 + (self.goal[1] - p[1]) ** 2)
            if dist < min_dist:
                min_dist = dist
                min_index = i
        return min_index + self.rotation_start_index

    # 軌道がゴールに向かって直線を横切ったかを判定する関数
    # ただし、一度横切った場所は無視する。
    def is_cross_to_goal(self):
        if is_crossed_line_segment(self.start, self.goal, self.pos):
            for c in self.cross_points:
                if math.fabs(c[0] - self.pos[0]) < self.EPS and math.fabs(c[1] - self.pos[1]) < self.EPS:
                    return False
            self.cross_points.append(self.pos)
            return True

        return False


    # indexを二つ与えると、その間の軌道にそって連続移動する関数
    def run(self):
        """
        軌道のアルゴリズム本体
        """
        # goalを向く
        theta = self.calc_theta_to_goal()
        self.rotation(theta)
        self.state = self.ST_MOVE_STRAIGHT

        search_count = 0
        while self.state != self.ST_STOP:
            if self.is_goal():
                self.state = self.ST_STOP
                print("Goal")
                break

            if self.state == self.ST_MOVE_STRAIGHT:
                next_x, next_y = self.next_pos()
                if self.collision_detection((next_x, next_y)):
                    theta = self.calc_next_theta()
                    self.rotation(theta)
                    self.state = self.ST_MOVE_ROTATE_RIGHT
                    self.rotation_start_index = len(self.trajectory)
                    next_x, next_y = self.next_pos()
            elif self.state == self.ST_MOVE_ROTATE_RIGHT:
                # 初めて遭遇したゴールまでの直線を横切ったか
                if self.is_cross_to_goal():
                    # ゴールに向かって直進する。
                    self.state = self.ST_MOVE_STRAIGHT
                    theta = self.calc_theta_to_goal()
                    self.rotation(theta)
                    next_x, next_y = self.next_pos()
                    continue
                else:
                    theta = self.calc_next_theta()
                    self.rotation(theta)
                    next_x, next_y = self.next_pos()

            self.move((next_x, next_y))
            search_count += 1
            if search_count > self.MAX_SEARCH_COUNT:
                print("search count over")
                self.state = self.ST_STOP
                break


def main():
    world = world1()

    robot = BugRobot2()
    robot.set_world(world)
    robot.set_start_goal_from_world()

    robot.run()

    robot.save_trajectory('out/result_bug2.png')
    world.save("out/map.png")


    pass


if __name__ == '__main__':
    main()