# BUG アルゴリズム
import math
from PIL import Image, ImageDraw, ImageFont
from more_itertools import windowed

font = ImageFont.truetype('C:/Windows/Fonts/Consola.ttf', 30)

class World():
    IMAGE_SIZE = 1200
    def __init__(self):
        self.min_x = -10.0
        self.max_x = 10.0
        self.min_y = -10.0
        self.max_y = 10.0
        self.obstacles = []

    def draw(self, filepath):
        image = Image.new('RGB', (self.IMAGE_SIZE, self.IMAGE_SIZE), color=(255, 255, 255))
        draw = ImageDraw.Draw(image)
        self.draw_axis(draw)

        image.save(filepath)

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

def main():
    # robot.set_map()
    # robot.run()
    world = World()
    world.draw('white.png')
    #world.set_start((50, 400))
    #world.set_goal((540, 80))

    #robot = Bug1Robot()
    #robot.set_world(world)
    #robot.set_start_goal_from_world()




    #robot.save_trajectory('out/result.png')



    pass


if __name__ == '__main__':
    main()