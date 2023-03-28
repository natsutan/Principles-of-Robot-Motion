import math
from PIL import Image, ImageDraw
from more_itertools import windowed

class Robot:
    DRAW_ROBO_SIZE = 2
    DEFAULT_SPEED = 0.1
    DEFAULT_ROTATION_UNIT = math.pi / 16.0

    # 同一座標かを判定する範囲Δ
    EPS = 0.01
    MAX_SEARCH_COUNT = 100000

    def __init__(self):
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
