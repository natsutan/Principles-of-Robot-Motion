# BUG アルゴリズム
import math
from PIL import Image, ImageDraw, ImageFont
from more_itertools import windowed

font = ImageFont.truetype('C:/Windows/Fonts/Consola.ttf', 30)



class World():
    DRAW_POINT_SIZE = 5
    IMAGE_SIZE = 1200
    def __init__(self):
        self.min_x = -10.0
        self.max_x = 10.0
        self.min_y = -10.0
        self.max_y = 10.0
        self.start = None
        self.goal = None
        self.obstacles = []

    def set_start(self, pos):
        self.start = pos

    def set_goal(self, pos):
        self.goal = pos

    def add_obstacle(self, obst):
        self.obstacles.append(obst)

    def point_to_pixel(self, pos):
        # 0～10.0の座標を、0～500のピクセルに変換して、センターをそろえる。
        px = int(pos[0] * 50 + self.IMAGE_SIZE / 2)
        py = self.IMAGE_SIZE - int(pos[1] * 50 + self.IMAGE_SIZE / 2)
        return (px, py)


    def save(self, filepath):
        image = Image.new('RGB', (self.IMAGE_SIZE, self.IMAGE_SIZE), color=(255, 255, 255))
        draw = ImageDraw.Draw(image)
        self.draw(draw)
        image.save(filepath)

    def draw(self, draw):
        self.draw_axis(draw)
        self.draw_start(draw)
        self.draw_goal(draw)

        for obst in self.obstacles:
            obst.draw(draw)

    def draw_start(self, draw):
        if self.start is None:
            return
        start_p = self.point_to_pixel(self.start)
        bbox = (start_p[0] - self.DRAW_POINT_SIZE, start_p[1] - self.DRAW_POINT_SIZE,
                start_p[0] + self.DRAW_POINT_SIZE, start_p[1] + self.DRAW_POINT_SIZE)

        draw.ellipse(bbox, fill=(0, 0, 255))

    def draw_goal(self, draw):
        if self.goal is None:
            return
        goal_p = self.point_to_pixel(self.goal)
        bbox = (goal_p[0] - self.DRAW_POINT_SIZE, goal_p[1] - self.DRAW_POINT_SIZE,
                goal_p[0] + self.DRAW_POINT_SIZE, goal_p[1] + self.DRAW_POINT_SIZE)

        draw.ellipse(bbox, fill=(255, 0, 0))

    def draw_axis(self, draw):

        draw.line((100, 100, 1100, 100), fill = (0,0,0), width=2)
        draw.line((100, 100, 100, 1100), fill = (0,0,0), width=2)
        draw.line((1100, 1100, 1100, 100), fill = (0,0,0), width=2)
        draw.line((1100, 1100, 100, 1100), fill = (0,0,0), width=2)

        draw.line((0, 600, 1200, 600), fill = (0,0,0), width=1)
        draw.line((600, 0, 600, 1200), fill = (0,0,0), width=1)

        draw.text((1150, 570), 'x', 'black', font=font)
        draw.text((630, 50), 'y', 'black', font=font)

        draw.line((350, 1100, 350, 1080), fill = (0,0,0), width=2)
        draw.line((850, 1100, 850, 1080), fill = (0,0,0), width=2)

        draw.line((100, 350, 120, 350), fill = (0,0,0), width=2)
        draw.line((100, 850, 120, 850), fill = (0,0,0), width=2)

        draw.text((50, 1100), '-10.0', 'black', font=font)
        draw.text((300, 1100), '-5.0', 'black', font=font)
        draw.text((600, 1100), '0.0', 'black', font=font)
        draw.text((830, 1100), '5.0', 'black', font=font)
        draw.text((1100, 1100), '10.0', 'black', font=font)

        draw.text((50, 1100), '-10.0', 'black', font=font)
        draw.text((25, 830), '-5.0', 'black', font=font)
        draw.text((50, 600), '0.0', 'black', font=font)
        draw.text((45, 330), '5.0', 'black', font=font)
        draw.text((25, 70), '10.0', 'black', font=font)




class Obstacle:
    def __init__(self, w, h):
        self.w = w
        self.h = h
        self.pos = None

    def set_pos(self, pos):
        self.pos = pos

    def draw(self, draw):

        lu_x = self.pos[0] - self.w / 2.0
        lu_y = self.pos[1] - self.h / 2.0
        rl_x = self.pos[0] + self.w / 2.0
        rl_y = self.pos[1] + self.h / 2.0

        lu_p = World().point_to_pixel((lu_x, lu_y))
        rl_p = World().point_to_pixel((rl_x, rl_y))

        bbox = (lu_p[0], lu_p[1], rl_p[0], rl_p[1])
        draw.rectangle(bbox, fill=(116, 80, 48))

    def is_collision(self, pos):
        if self.pos is None:
            return False

        min_x = self.pos[0] - self.w / 2
        max_x = self.pos[0] + self.w / 2
        min_y = self.pos[1] - self.h / 2
        max_y = self.pos[1] + self.h / 2

        return min_x < pos[0] < max_x and min_y < pos[1] < max_y


class BugRobot:
    DRAW_ROBO_SIZE = 6
    DEFAULT_SPEED = 1.0
    DEFAULT_ROTATION_UNIT = math.pi / 4.0

    ST_IDLE = 0
    ST_MOVE_STRAIGHT = 1
    ST_MOVE_ROTATE_RIGHT = 2
    ST_STOP = -1
    # 同一座標かを判定する範囲Δ
    EPS = 0.05

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
        self.rotation_end_index = 0


    def collision_detection(self, next_pos):
        for o in self.world.obstacles:
            if o.is_collision(next_pos):
                return True

        return False

    def move(self, pos):
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

    def calc_next_theta(self, inverse = False):
        """
        障害物をよける角度を見つける。
        正面先を調べて場合分け
        ・正面に進むとぶつかるとき、左側でぶつからない角度を探す
        ・正面に進んでもぶつからないとき、右側のぶつからない角度を探す
        """
        _, _, theta = self.pose()
        next_x, next_y = self.next_pos()
        if self.collision_detection((next_x, next_y)):
            sign = 1
        else:
            sign = -1

        if inverse:
            sign = -sign

        while True:
            theta = theta + sign * self.DEFAULT_ROTATION_UNIT
            nx, ny = self.next_pos(theta)
            if self.collision_detection((nx, ny)):
                continue
            return theta

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
    def is_visited(self, pos):
        if len(self.trajectory) - self.rotation_start_index < 2:
            return False

        for p in self.trajectory[self.rotation_start_index:-1]:
            if math.fabs(p[0] - pos[0]) < self.EPS and math.fabs(p[1] - pos[1]) < self.EPS:
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
            if self.state == self.ST_MOVE_STRAIGHT:
                next_x, next_y = self.next_pos()
                if self.collision_detection((next_x, next_y)):
                    theta = self.calc_next_theta()
                    self.rotation(theta)
                    self.state = self.ST_MOVE_ROTATE_RIGHT
                    self.rotation_start_index = len(self.trajectory)
                    next_x, next_y = self.next_pos()
            elif self.state == self.ST_MOVE_ROTATE_RIGHT:
                if self.is_visited(self.pose()):
                    print("visited")

                    # ゴールに一番近い場所を探す
                    
                    # そこに移動する。

                    # ゴールまでの向きを計算する

                    # そこに向かって移動する。

                    self.state = self.ST_MOVE_STRAIGHT
                    continue
                theta = self.calc_next_theta_rotation()
                self.rotation(theta)
                next_x, next_y = self.next_pos()

            self.move((next_x, next_y))
            search_count += 1
            if search_count > 1000:
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
    world = World()
    world.set_start((-8.0, -8.0))
    world.set_goal((9.0, 9.0))

    obst1 = Obstacle(5.0, 3.0)
    obst1.set_pos((-3.0, -2.0))
    world.add_obstacle(obst1)

    obst2 = Obstacle(7.0, 3.0)
    obst2.set_pos((0, -1.0))
    world.add_obstacle(obst2)

    obst2 = Obstacle(4.0, 4.0)
    obst2.set_pos((5.0, 5.0))
    world.add_obstacle(obst2)


    robot = BugRobot()
    robot.set_world(world)
    robot.set_start_goal_from_world()

    robot.run()

    robot.save_trajectory('out/result.png')
    world.save("out/map.png")


    pass


if __name__ == '__main__':
    main()