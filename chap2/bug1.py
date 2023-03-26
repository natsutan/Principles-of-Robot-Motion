# BUG アルゴリズム
import math
from PIL import Image, ImageDraw
from more_itertools import windowed

from common.world import world1, world2

class BugRobot:
    DRAW_ROBO_SIZE = 2
    DEFAULT_SPEED = 0.1
    DEFAULT_ROTATION_UNIT = math.pi / 16.0

    ST_IDLE = 0
    ST_MOVE_STRAIGHT = 1
    ST_MOVE_ROTATE_RIGHT = 2
    ST_STOP = -1
    # 同一座標かを判定する範囲Δ
    EPS = 0.01
    MAX_SEARCH_COUNT = 100000

    def __init__(self):
        self.state = self.ST_IDLE
        self.pos = None
        self.theta = None
        self.world = None
        self.start = None
        self.goal = None
        self.trajectory = []
        self.speed = self.DEFAULT_SPEED

    def pose(self):
        return (self.pos[0], self.pos[1], self.theta)

    def set_world(self, world):
        self.world = world

    def set_start_goal_from_world(self):
        self.start = self.world.start
        self.goal = self.world.goal
        self.pos = (self.start[0], self.start[1])
        self.theta = 0
        self.trajectory = [self.pose(), ]
        # 回転を開始した座標のインデックスを保存する変数
        self.rotation_start_index = 0


    def collision_detection(self, next_pos):
        for o in self.world.obstacles:
            for i in range(0, 10):
                x = next_pos[0] + (self.pos[0] - next_pos[0]) * i / 10.0
                y = next_pos[1] + (self.pos[1] - next_pos[1]) * i / 10.0
                if o.is_collision((x, y)):
                    return True

        return False

    def move(self, pos):
        if math.fabs(self.pos[0] - pos[0]) > 3:
            print("robot warped")


        self.pos  = pos

        # 衝突判定
        if self.collision_detection(pos):
            print(f"Collision detected {pos}")

        self.trajectory.append(self.pose())

    def rotation(self, theta):
        """
        thetaの方向を向く
        """
        if theta > math.pi * 2:
            theta -= 2 * math.pi
        elif theta < 0:
            theta += 2 * math.pi

        #print(f"rotation {theta}")

        self.theta = theta

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

    def calc_next_theta_rotation(self):
        theta = self.theta
        theta = theta - math.pi + self.DEFAULT_ROTATION_UNIT
        while True:
            nx, ny = self.next_pos(theta)
            if self.collision_detection((nx, ny)):
                theta = theta + self.DEFAULT_ROTATION_UNIT
                continue
            return theta



    def next_pos(self, theta = None):
        """
        単位時間後の自分の位置を計算する。
        thetaが指定されたときはその向きに移動した場所。
        thetaが指定されていないときは、現在のthetaを使用する。
        """
        if theta is None:
            x, y, theta = self.pose()
        else:
            x, y, _ = self.pose()

        next_x = x + math.cos(theta) * self.speed
        next_y = y + math.sin(theta) * self.speed
        return (next_x, next_y)

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

    # indexを二つ与えると、その間の軌道にそって連続移動する関数
    def move_trajectory(self, start_index, end_index):
        for i in range(start_index, end_index+1):
            self.move(self.trajectory[i])
            if self.collision_detection(self.pose()):
                print("Collision detected")

    def is_goal(self):
        if len(self.trajectory) < 2:
            return False

        x, y, _ = self.pose()
        last_x, last_y, _ = self.trajectory[-2]
        for i in range(0, 10):
            tmp_x = last_x + (x - last_x) * i / 10.0
            tmp_y = last_y + (y - last_y) * i / 10.0

            dist = math.sqrt((self.goal[0] - tmp_x) ** 2 + (self.goal[1] - tmp_y) ** 2)
            if dist < self.EPS:
                return True
        return False
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
                restart_pos = self.visited_pos(self.pose())

                if restart_pos != None:
                    # ゴールに一番近い場所を探す
                    min_index = self.find_nearest_goal()
                    # そこに移動する。
                    self.move_trajectory(restart_pos, min_index)
                    # ゴールまでの向きを計算する
                    theta = self.calc_theta_to_goal()
                    self.rotation(theta)
                    # そこに向かって移動する。
                    self.rotation_start_index = 0
                    self.state = self.ST_MOVE_STRAIGHT
                    continue
                theta = self.calc_next_theta()
                self.rotation(theta)
                next_x, next_y = self.next_pos()

            self.move((next_x, next_y))
            search_count += 1
            if search_count > self.MAX_SEARCH_COUNT:
                print("search count over")
                self.state = self.ST_STOP
                break

    def save_trajectory(self, filepath):
        image = Image.new('RGB', (self.world.IMAGE_SIZE, self.world.IMAGE_SIZE), color=(255, 255, 255))
        draw = ImageDraw.Draw(image)
        self.world.draw(draw)

        for p in windowed(self.trajectory, 2):
            src_p = self.world.point_to_pixel((p[0][0], p[0][1]))
            dst_p = self.world.point_to_pixel((p[1][0], p[1][1]))

            draw.line((src_p[0], src_p[1], dst_p[0], dst_p[1]), fill = (0, 255, 0), width = 2)
            robo_bbox = (dst_p[0] - self.DRAW_ROBO_SIZE, dst_p[1] - self.DRAW_ROBO_SIZE, dst_p[0] + self.DRAW_ROBO_SIZE, dst_p[1] + self.DRAW_ROBO_SIZE)
            draw.ellipse(robo_bbox, outline = (0, 255, 0), width=1)

        image.save(filepath)


def main():
    world = world2()

    robot = BugRobot()
    robot.set_world(world)
    robot.set_start_goal_from_world()

    robot.run()

    robot.save_trajectory('out/result.png')
    world.save("out/map.png")


    pass


if __name__ == '__main__':
    main()