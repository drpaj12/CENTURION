#Imports
import pygame, sys
from pygame.locals import *
import pygame.gfxdraw
import random, time
from bs4 import BeautifulSoup
import lxml
import math


def draw_agent(surface, color, x, y, radius, color_line, angle):
    pygame.gfxdraw.aacircle(surface, x, y, radius, color)
    pygame.gfxdraw.filled_circle(surface, x, y, radius, color)
    rad_angle = math.radians(angle)
    pygame.gfxdraw.line(surface, x, y, x + int(radius * math.cos(rad_angle)), y + int(radius * math.sin(rad_angle)), color_line)

def draw_circle(surface, color, x, y, radius):
    pygame.gfxdraw.aacircle(surface, x, y, radius, color)
    #pygame.gfxdraw.filled_circle(surface, x, y, radius, color)

def to_pygame(coords, height):
    """Convert coordinates into pygame coordinates (lower-left => top left)."""
    return (coords[0], height - coords[1])
def to_pygame_x(x, width):
    return width - x
def to_pygame_y(y, height):
    return height - y
def to_pygame(coords, height, obj_height):
    """Convert an object's coords into pygame coordinates (lower-left of object => top left in pygame coords)."""
    return (coords[0], height - coords[1] - obj_height)

#Creating colors
BLUE  = (0, 0, 255)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

SURFACE_COLOR = (255, 255, 255)#(167, 255, 100)
COLOR = (255, 100, 98)

#Other Variables for use in the program
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 600
# add 10 surrounding space
DISPLAY_EDGE = 10

#Initialzing 
pygame.init()

size = (SCREEN_WIDTH, SCREEN_HEIGHT)
screen = pygame.display.set_mode(size)

#Setting up FPS 
FPS = 60
FramePerSec = pygame.time.Clock()

#Setting up Sprites        
all_sprites_list = pygame.sprite.Group()


# Object class
class circle_static(pygame.sprite.Sprite):
    def __init__(self, color, x, y, radius):
        super().__init__()

        # need to make surface big enough for circle
        self.image = pygame.Surface((2*radius+10, 2*radius+10), pygame.SRCALPHA)
        self.color = color
        self.x = x
        self.y = y
        self.radius = radius
        #pygame.draw.circle(self.image, self.color, (self.radius, self.radius), self.radius)
        draw_circle(self.image, self.color, int(self.radius), int(self.radius), int(self.radius))
        self.rect = self.image.get_rect(center=(x, y))

class circle_agent(pygame.sprite.Sprite):
    def __init__(self, color, x, y, radius, line_color, angle):
        super().__init__()

        # need to make surface big enough for circle
        self.image = pygame.Surface((radius*2+10, radius*2+10),pygame.SRCALPHA)
        self.color = color
        self.line_color = line_color
        self.x = x
        self.y = y
        self.angle = angle
        self.radius = radius
        draw_agent(self.image, self.color, int(self.radius), int(self.radius), self.radius, self.line_color, self.angle)
        self.rect = self.image.get_rect(center=(x, y))
    
    def change_angle(self, angle):
        self.angle = angle
        draw_agent(self.image, self.color, int(self.radius), int(self.radius), self.radius, self.line_color, self.angle)

# read xml in
infile = open("log_file.xml","r")
contents = infile.read()
soup = BeautifulSoup(contents,features='lxml')

time_step_in_s = float(soup.find('time_step_in_s').text)
simulation_width = float(soup.find('sim_x_in_m').text)
simulation_height = float(soup.find('sim_y_in_m').text)
agent_radius = float(soup.find('agent_radius').text)
print("Simulation - time_step_in_s:"+str(time_step_in_s)+" simulation_width:"+str(simulation_height)+" :simulation_height"+str(simulation_height))

# scale the x and ys from sim by this many times
scale_factor_w = (SCREEN_WIDTH-2*DISPLAY_EDGE) / simulation_width
scale_factor_h = (SCREEN_HEIGHT-2*DISPLAY_EDGE) / simulation_height
scale_factor = min(scale_factor_w, scale_factor_h) 
print("Simulation - scale_factor:"+str(scale_factor))

result = soup.find('object')
circles = result.find_all('circle')
circle_list = []
for i in range(0, len(circles)):
    x = float(circles[i].find('x').text)
    y = float(circles[i].find('y').text)
    radius = float(circles[i].find('radius').text)
    circle_list.append((x,y,radius))

    #print("b_cs->"+str(x)+":"+str(y)+":"+str(radius));
    # SCALE and FLIP X
    x = int(x*scale_factor)+DISPLAY_EDGE
    y = y*scale_factor
    y = y + DISPLAY_EDGE
    y = int(to_pygame_y(y, SCREEN_HEIGHT))
    radius = int(radius*scale_factor)
    # sprites added at x,y is top corner, so -radius to make centered
    x = x - radius
    y = y - radius

    # add object to sprite list
    object_ = circle_static(RED, x, y, radius)
    object_.rect.x = x
    object_.rect.y = y

    all_sprites_list.add(object_)

print(circle_list)

rectangles = result.find_all('rectangle')
rectangle_list = []
rectangle_draw_list = []
for i in range(0, len(rectangles)):
    x = float(rectangles[i].find('x').text)
    y = float(rectangles[i].find('y').text)
    halfx = float(rectangles[i].find('halfx').text)
    halfy = float(rectangles[i].find('halfy').text)
    rotation = float(rectangles[i].find('rotation').text)
    rectangle_list.append((x,y,halfx,halfy,rotation))

    # scale and flip y
    #print("b_rs->"+str(x)+":"+str(y)+":"+str(halfx)+":"+str(halfy)+":"+str(rotation));
    x = x*scale_factor
    y = y*scale_factor
    halfx = halfx * scale_factor
    halfy = halfy * scale_factor

    # add object to sprite list
    # 0 1
    # 2 3
    rad_angle = math.radians(rotation)
    cx = x 
    cy = y 
    x0 = cx - halfx;
    y0 = cy + halfy;
    rx0 = int(cx + math.cos(rad_angle)*(x0 - cx) - math.sin(rad_angle)*(y0 - cy))
    ry0 = int(cy + math.sin(rad_angle)*(x0 - cx) + math.cos(rad_angle)*(y0 - cy))
    x1 = cx + halfx;
    y1 = cy + halfy;
    rx1 = int(cx + math.cos(rad_angle)*(x1 - cx) - math.sin(rad_angle)*(y1 - cy))
    ry1 = int(cy + math.sin(rad_angle)*(x1 - cx) + math.cos(rad_angle)*(y1 - cy))
    x2 = cx + halfx;
    y2 = cy - halfy;
    rx2 = int(cx + math.cos(rad_angle)*(x2 - cx) - math.sin(rad_angle)*(y2 - cy))
    ry2 = int(cy + math.sin(rad_angle)*(x2 - cx) + math.cos(rad_angle)*(y2 - cy))
    x3 = cx - halfx;
    y3 = cy - halfy;
    rx3 = int(cx + math.cos(rad_angle)*(x3 - cx) - math.sin(rad_angle)*(y3 - cy))
    ry3 = int(cy + math.sin(rad_angle)*(x3 - cx) + math.cos(rad_angle)*(y3 - cy))

    # flip ys 
    rx0 = rx0 + DISPLAY_EDGE
    ry0 = ry0 + DISPLAY_EDGE
    ry0 = int(to_pygame_y(ry0, SCREEN_HEIGHT))
    rx1 = rx1 + DISPLAY_EDGE
    ry1 = ry1 + DISPLAY_EDGE
    ry1 = int(to_pygame_y(ry1, SCREEN_HEIGHT))
    rx2 = rx2 + DISPLAY_EDGE
    ry2 = ry2 + DISPLAY_EDGE
    ry2 = int(to_pygame_y(ry2, SCREEN_HEIGHT))
    rx3 = rx3 + DISPLAY_EDGE
    ry3 = ry3 + DISPLAY_EDGE
    ry3 = int(to_pygame_y(ry3, SCREEN_HEIGHT))
    #print("rs->"+str(rx0)+":"+str(ry0)+":"+str(rx1)+":"+str(ry1)+":"+str(rx2)+":"+str(ry2)+":"+str(rx3)+":"+str(ry3))
    rectangle_draw_list.append((rx0, ry0, rx1, ry1, rx2, ry2, rx3, ry3))

print(rectangle_list)

time_step = soup.find_all('time_step')
agent_list = []
agent_sprites = []
for i in range(0, len(time_step)):
    agents = time_step[i].find_all('agent')
    agent_list.append([])
    for j in range(0, len(agents)):
        x = float(agents[j].find('x').text)
        y = float(agents[j].find('y').text)
        # SCALE
        radius = int(agent_radius*scale_factor)

        # SCALE and FLIP Y
        x = int(x*scale_factor)+DISPLAY_EDGE
        y = y*scale_factor
        y = y + DISPLAY_EDGE
        y = int(to_pygame_y(y, SCREEN_HEIGHT))
        # sprites added at x,y is top corner, so -radius to make centered
        x = x - radius
        y = y - radius

        angle = float(agents[j].find('angle').text)
        agent_list[i].append((x,y,angle))

        if i == 0:

            # add object to sprite list
            object_ = circle_agent(GREEN, x, y, radius, BLUE, angle)
            object_.rect.x = x
            object_.rect.y = y

            all_sprites_list.add(object_)
            agent_sprites.append(object_)

print(agent_list)

exit = True
clock = pygame.time.Clock()

# keeps track of which frame we're in
current_step = 0
step_size = 0
last_step = len(time_step) - 1

while exit:
    for event in pygame.event.get():
        #print("an event->"+str(event))
        if event.type == pygame.QUIT:
            exit = False
        if event.type == KEYDOWN:
            if event.key == pygame.K_q:
                print("Quit")
                exit = False
            if event.key == pygame.K_1:
                step_size = 1
            if event.key == pygame.K_2:
                step_size = 5
            if event.key == pygame.K_3:
                step_size = 10
            if event.key == pygame.K_4:
                step_size = 20
            if event.key == pygame.K_5:
                step_size = 30
            if event.key == pygame.K_n:
                step_size = -step_size
            if event.key == pygame.K_SPACE:
                print("Next Frame")
                if current_step == last_step:
                    current_step = last_step
                elif current_step + step_size > last_step:
                    current_step = last_step
                else:
                    current_step = current_step + step_size
                    
                print("Current_step:"+str(current_step))
                for i in range(0, len(agent_sprites)):
                    print(str(agent_list[current_step][i][0])+':'+str(agent_list[current_step][i][1])+':'+str(agent_list[current_step][i][2]));
                    agent_sprites[i].rect.x = agent_list[current_step][i][0]
                    agent_sprites[i].rect.y = agent_list[current_step][i][1]
                    agent_sprites[i].change_angle(agent_list[current_step][i][2])
    

    all_sprites_list.update()
    screen.fill(SURFACE_COLOR)
    # draw sim box
    pygame.draw.polygon(screen, BLACK, ((DISPLAY_EDGE, DISPLAY_EDGE), (DISPLAY_EDGE, SCREEN_HEIGHT-DISPLAY_EDGE), (SCREEN_WIDTH-DISPLAY_EDGE, SCREEN_HEIGHT-DISPLAY_EDGE), (SCREEN_WIDTH-DISPLAY_EDGE, DISPLAY_EDGE)), 2)

    all_sprites_list.draw(screen)

    # draw static rectangles
    for i in range(0, len(rectangle_draw_list)):
        pygame.draw.polygon(screen, BLUE, ((rectangle_draw_list[i][0], rectangle_draw_list[i][1]), (rectangle_draw_list[i][2], rectangle_draw_list[i][3]), (rectangle_draw_list[i][4], rectangle_draw_list[i][5]), (rectangle_draw_list[i][6], rectangle_draw_list[i][7])), 1) 

    #pygame.draw.circle(screen, BLACK, (66,462), 2)

    pygame.display.flip()


    clock.tick(60)

pygame.quit()
