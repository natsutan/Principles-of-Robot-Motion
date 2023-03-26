from PIL import Image, ImageDraw, ImageFont

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

def world1():
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

    return world

def world2():
    world = World()
    world.set_start((-8.0, -0.0))
    world.set_goal((6.0, 0.0))

    obst1 = Obstacle(10.0, 1.0)
    obst1.set_pos((-1, 6.0))
    world.add_obstacle(obst1)

    obst2 = Obstacle(1.0, 12.0)
    obst2.set_pos((-6.0, -0.0))
    world.add_obstacle(obst2)

    obst3 = Obstacle(14.0, 1.0)
    obst3.set_pos((1.0, -6.0))
    world.add_obstacle(obst3)

    return world
