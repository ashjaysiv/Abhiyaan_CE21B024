import time
import threading
import pygame
from pygame import mixer
import sys
import math

# utils
#rotate image of car as it moves


mixer.init()
mixer.music.load("yes_roundabout.ogg") #change the cars to a mini speedwagon for bonus :3
mixer.music.set_volume(0.7)
mixer.music.play()

# Default values of signal timers
defaultGreen = {0:10, 1:10, 2:10, 3:10}
defaultRed = 150
defaultYellow = 5

signals = []
noOfSignals = 4
currentGreen = 0   # Indicates which signal is green currently
nextGreen = (currentGreen+1)%noOfSignals    # Indicates which signal will turn green next
currentYellow = 0   # Indicates whether yellow signal is on or off 

speeds = {'car':0.14, 'bus':0.5, 'truck':0.5, 'bike':0.5}  # average speeds of vehicles, in the year 2030 no speeding tickets exist,feel free to increase this 

# Coordinates of vehicles' start at each of the 4 lanes per road
x = {'right':[0,0,0,0], 'down':[550,610,670,720], 'left':[1265,1265,1265,1265], 'up':[550,610,670,720]}    
y = {'right':[270,330,380,440], 'down':[0,0,0,0], 'left':[270,330,380,440], 'up':[680,680,680,680]}

vehicles = {'right': {0:[], 1:[], 2:[], 'crossed':0}, 'down': {0:[], 1:[], 2:[], 'crossed':0}, 'left': {0:[], 1:[], 2:[], 'crossed':0}, 'up': {0:[], 1:[], 2:[], 'crossed':0}}
vehicleTypes = {0:'car', 1:'bus', 2:'truck', 3:'bike'}
directionNumbers = {0:'right', 1:'down', 2:'left', 3:'up'}

# Coordinates of signal image, timer, and vehicle count
signalCoods = [(455,233),(576,546),(706,100),(834,400)]
signalTimerCoods = [(455,233),(576,546),(706,100),(834,400)]

# Coordinates of stop lines
stopLines = {'right': 290, 'down': 90, 'left': 940, 'up': 690}

# Gap between vehicles
stoppingGap = 15    # stopping gap
movingGap = 15   # moving gap

# Path
path_1 = [(76, 281), (177, 279), (348, 276), (383, 265), (406, 249), (432, 232), (457, 219), (475, 200), (496, 177), (525, 162), (553, 150), (587, 139), (613, 133), (637, 131), (669, 136), (691, 143), (723, 150), (754, 163), (782, 179), (797, 195), (813, 218), (827, 236), (841, 254), (852, 272), (856, 295), (862, 324), (866, 351), (860, 382), (853, 424), (848, 458), (842, 474), (831, 485), (812, 497), (793, 515), (780, 529), (766, 542), (756, 565), (750, 581), (736, 614), (729, 629), (729, 651), (725, 669), (723, 683), (721, 697)]      
[(76, 281), (79, 283), (348, 276), (383, 265), (406, 249), (432, 232), (457, 219), (475, 200), (496, 177), (525, 162), (553, 150), (587, 139), (613, 133), (646, 132), (669, 136), (691, 143), (723, 150), (754, 163), (782, 179), (797, 195), (813, 218), (827, 236), (841, 254), (852, 272), (856, 295), (862, 324), (860, 358), (860, 382), (853, 424), (848, 458), (842, 474), (831, 485), (812, 497), (793, 515), (780, 529), (766, 542), (756, 565), (750, 581), (736, 614), (729, 629), (729, 651), (725, 669), (723, 683), (721, 697), (722, 709), (722, 750)]

stop_positions = [(177, 279), (637, 131), (860, 358), (643, 583)]

path_2 =[ (502, 543), (478, 513), (457, 489), (440, 466), (427, 438), (415, 411), (409, 378), (414, 339), (420, 311), (428, 274), (435, 245), (451, 222), (472, 207), (498, 187), (517, 169), (535, 149), (542, 118), (547, 93), (552, 60), (553, 33), (555, 13)]

path_3 = [(1257, 380), (1201, 380), (1137, 378), (1069, 375), (1013, 379), (934, 383), (883, 397), (816, 422), (782, 454), (751, 489), (721, 507), (693, 525), (684, 555), (680, 589), (675, 629), (672, 675), (671, 705)]

path_4 = [(719, 37), (724, 64), (727, 84), (735, 112), (748, 143), (763, 163), (781, 182), (801, 201), (817, 221), (830, 243), (839, 263), (851, 287), (865, 325), (863, 351), (861, 378), (857, 407), (852, 429), (840, 466), (823, 498), (812, 511), (792, 533), (759, 553), (722, 567), (673, 573), (634, 583), (587, 571), (556, 560), (519, 558), (502, 536), (479, 506), (453, 485), (427, 459), (387, 448), (355, 445), (309, 435), (255, 439), (221, 439), (181, 437), (150, 437), (126, 436), (95, 436), (62, 439)]
[(719, 37), (724, 64), (727, 84), (735, 112), (748, 143), (763, 163), (781, 182), (801, 201), (817, 221), (830, 243), (839, 263), (851, 287), (865, 325), (863, 351), (861, 378), (857, 407), (852, 429), (840, 466), (823, 498), (812, 511), (792, 533), (759, 553), (722, 567), (673, 573), (634, 583), (587, 571), (556, 560), (519, 558), (502, 536), (479, 506), (453, 485), (427, 459), (387, 448), (355, 445), (309, 435), (255, 439), (221, 439), (181, 437), (150, 437), (126, 436), (95, 436), (62, 439), (22, 439)]

pygame.init()
simulation = pygame.sprite.Group()

class TrafficSignal:
    def __init__(self, red, yellow, green):
        self.red = red
        self.yellow = yellow
        self.green = green
        self.signalText = ""
        

#your job is in this Class, you need to create functions within this class that will aid you in following the rules of the road, 
# this is where you will implement your behaviour planner
class Vehicle(pygame.sprite.Sprite):
    def __init__(self, lane, vehicleClass, direction_number, direction, points):
        pygame.sprite.Sprite.__init__(self)
        self.lane = lane
        self.vehicleClass = vehicleClass
        self.speed = speeds[vehicleClass]
        self.rotation_vel = 1
        self.direction_number = direction_number
        self.direction = direction
        self.x = x[direction][lane]
        self.y = y[direction][lane]
        self.angle = 0
        self.crossed = 0
        vehicles[direction][lane].append(self)
        self.index = len(vehicles[direction][lane]) - 1
        path = "images/" + direction + "/" + vehicleClass + ".png"
        self.image = pygame.image.load(path)
        self.point = points
        self.current_point = 0
        self.red_cross = 0

        simulation.add(self)

    def render(self, screen):
        screen.blit(self.image, (self.x, self.y))

    def move(self):
        if self.current_point >= len(self.point):
            # kill on reaching the end
            self.kill()
            return
        
        self.calculate_angle()
        self.update_path_point()
        
        # calculate velocity components
        radians = math.radians(self.angle)
        vertical = math.cos(radians) * self.speed
        horizontal = math.sin(radians) * self.speed

        self.y -= vertical
        self.x -= horizontal 

    def stop(self):
        self.speed = 0
    
    def rotate(self, left = False, right = False):
        if left:
            self.angle += self.rotation_vel
        elif right:
            self.angle -= self.rotation_vel        
    #function to rortate the image as it moves in the path
    def draw(self, screen):
        if self.direction_number == 3:
            rotated_image = pygame.transform.rotate(self.image,  self.angle)
        
        elif self.direction_number == 2:
            rotated_image = pygame.transform.rotate(self.image,  self.angle - 90)

        elif self.direction_number == 1:
            rotated_image = pygame.transform.rotate(self.image,  - self.angle)

        else:
            rotated_image = pygame.transform.rotate(self.image,  90 + self.angle)

        new_rect = rotated_image.get_rect(center = self.image.get_rect(topleft = (self.x, self.y)).center)
        screen.blit(rotated_image, new_rect.topleft)

    # calculate angle between the next path point and the current point
    def calculate_angle(self):
        target_x, target_y = self.point[self.current_point]
        x_diff = target_x - self.x
        y_diff = target_y - self.y

        if y_diff == 0:
            desired_radian_angle = math.pi / 2
        else:
            desired_radian_angle = math.atan(x_diff / y_diff)

        if target_y > self.y:
            desired_radian_angle += math.pi

        difference_in_angle = self.angle - math.degrees(desired_radian_angle)
        if difference_in_angle >= 180:
            difference_in_angle -= 360

        if difference_in_angle > 0:
            self.angle -= min(self.rotation_vel, abs(difference_in_angle))
        else:
            self.angle += min(self.rotation_vel, abs(difference_in_angle))

    # once point is reached we move to the next state
    def update_path_point(self):
        target = self.point[self.current_point]
        rect = pygame.Rect(
            self.x, self.y, self.image.get_width(), self.image.get_height())
        if rect.collidepoint(*target):
            if target in stop_positions:
                self.red_cross += 1
                
            self.current_point += 1

# stop vehicle infront of another vehicle when safe stooping distance is achieved
def collide(vehicle, simulation):
    for veh in simulation:
        if veh != vehicle:
            distance = ((vehicle.x - veh.x)**2 + (vehicle.y - veh.y)**2) **0.5
            if distance < 80 and veh.speed == 0:
                vehicle.speed = 0
                return True
            

                
      

        

#########################################################################################################################################
#dont worry about this part


# Initialization of signals with default values
def initialize():
    ts1 = TrafficSignal(0, defaultYellow, defaultGreen[0])
    signals.append(ts1)
    ts2 = TrafficSignal(ts1.red+ts1.yellow+ts1.green, defaultYellow, defaultGreen[1])
    signals.append(ts2)
    ts3 = TrafficSignal(defaultRed, defaultYellow, defaultGreen[2])
    signals.append(ts3)
    ts4 = TrafficSignal(defaultRed, defaultYellow, defaultGreen[3])
    signals.append(ts4)
    repeat()

def repeat():
    global currentGreen, currentYellow, nextGreen
    while(signals[currentGreen].green>0):   # while the timer of current green signal is not zero
        updateValues()
        time.sleep(1)
    currentYellow = 1   # set yellow signal on
    # reset stop coordinates of lanes and vehicles 
    for i in range(0,3):
        for vehicle in vehicles[directionNumbers[currentGreen]][i]:
            vehicle.stop = stopLines[directionNumbers[currentGreen]]
    while(signals[currentGreen].yellow>0):  # while the timer of current yellow signal is not zero
        updateValues()
        time.sleep(1)
    currentYellow = 0   # set yellow signal off
    
     # reset all signal times of current signal to default times
    signals[currentGreen].green = defaultGreen[currentGreen]
    signals[currentGreen].yellow = defaultYellow
    signals[currentGreen].red = defaultRed
       
    currentGreen = nextGreen # set next signal as green signal
    nextGreen = (currentGreen+1)%noOfSignals    # set next green signal
    signals[nextGreen].red = signals[currentGreen].yellow+signals[currentGreen].green    # set the red time of next to next signal as (yellow time + green time) of next signal
    repeat()  

# Update values of the signal timers after every second
def updateValues():
    for i in range(0, noOfSignals):
        if(i==currentGreen):
            if(currentYellow==0):
                signals[i].green-=1
            else:
                signals[i].yellow-=1
        else:
            signals[i].red-=1
####################################################################################################################################################

# model the conditions to stop or move a vehicle at the signal
def signal_0(currentGreen, currentYellow, car):
    if currentGreen == 0:
        # allow vehicle to move once green light is on
        car.speed = 0.14
        # cars from roundabout shudnt cross y = 318
        if round(car.y) == 318 and round(car.x) < 637:
            car.speed = 0
                  
    else:
        
        if round(car.y) < 350 and round(car.x) < 630:
            car.speed = 0.14

        # cars from west shouldnt cross x = 177
        if round(car.x) == 150:
            car.speed = 0
            for vehicle in simulation:  
                collide(vehicle, simulation)

def signal_2(currentGreen, currentYellow, car):
    
    if currentGreen == 2:
        #make car move once it is green
        if round(car.y) < 318 and round(car.x) > 600:
            car.speed = 0.14

        # car from roundabout stops
        if round(car.x) == 630 and round(car.y) < 318:
            car.speed = 0

    else:
        # car from north stops
        if round(car.y) == 50 and round(car.x) > 630:
            car.speed = 0
            

def signal_3(currentGreen, currentYellow, car):
    if currentGreen == 3:
        #make the car move once light is green
        if round(car.x) > 630 and round(car.y) < 400 and round(car.y) > 100:
            car.speed = 0.14
        # car from roundabout stops
        if round(car.y) == 318 and round(car.x) > 630:
            
            car.speed = 0
    else:
        # car from east stops
        if round(car.x) == 1080:
            car.speed = 0
           

def signal_1(currentGreen, currentYelow, car):
    if currentGreen == 1:
        #make car move once the light is green
        if round(car.y) > 318 and round(car.x < 600):
            car.speed = 0.14
        
        # car from roundabouts stop
        if round(car.x) == 650 and round(car.y) > 318:
            car.speed = 0
    
    else:
        #car from south stops
        if round(car.y) == 600 and round(car.x) < 630:
            car.speed = 0
            for vehicle in simulation:  
                collide(vehicle, simulation)
        
# Generating vehicles in the simulation, later you can change this so it randomly generates different vehicles in different lanes
def generateVehicles():
        while(True):
         
            Vehicle(0, vehicleTypes[0], 0, directionNumbers[0], path_1)
            Vehicle(0, vehicleTypes[1], 3, directionNumbers[3], path_2)
            Vehicle(2, vehicleTypes[2], 2, directionNumbers[2], path_3)
            Vehicle(2, vehicleTypes[3], 1, directionNumbers[1], path_4)

            time.sleep(100)

class Main:
    thread1 = threading.Thread(name="initialization",target=initialize, args=())    # initialization
    thread1.daemon = True
    thread1.start()

    # Colours 
    black = (0, 0, 0)
    white = (255, 255, 255)

    # Screensize 
    screenWidth = 1280
    screenHeight = 720
    screenSize = (screenWidth, screenHeight)

    # Setting background image i.e. image of Gajendra Circle
    background = pygame.image.load('images/roundabout.png')

    screen = pygame.display.set_mode(screenSize)
    pygame.display.set_caption("SIMULATION")

    # Loading signal images and font
    redSignal = pygame.image.load('images/signals/red.png')
    yellowSignal = pygame.image.load('images/signals/yellow.png')
    greenSignal = pygame.image.load('images/signals/green.png')
    font = pygame.font.Font(None, 30)

    thread2 = threading.Thread(name="generateVehicles",target=generateVehicles, args=())    # Generating vehicles
    thread2.daemon = True
    thread2.start()

    #initialize the time
    t = 0

    screen.blit(background,(0,0))
    
    # initialize the thread 
    first = True
    thread_count_a = 0
    thread_a = [0 for i in range(10)]

    
    while True:
        
        for event in pygame.event.get():
            
            if event.type == pygame.QUIT:
                sys.exit()
            
            # to get the points of the path

            # if event.type == pygame.MOUSEBUTTONDOWN:
            #     pos = pygame.mouse.get_pos()
            #     for point in path_4:
            #         pygame.draw.circle(screen, (0,255,0), pos, 5)
            #         pygame.draw.circle(screen, (0,255,0), point, 5)
            #         pygame.display.update()
            #     path_4.append(pos)
            #     print(path_4)
      
        
        # display background in simulation
        screen.blit(background,(0,0))
        
        # display signal and set timer according to current status: green, yello, or red
        for i in range(0,noOfSignals):  
            if(i==currentGreen):
                if(currentYellow==1):
                    signals[i].signalText = signals[i].yellow
                    screen.blit(yellowSignal, signalCoods[i])
                else:
                    signals[i].signalText = signals[i].green
                    screen.blit(greenSignal, signalCoods[i])
            else:
                if(signals[i].red<=10):
                    signals[i].signalText = signals[i].red
                else:
                    signals[i].signalText = "---"
                screen.blit(redSignal, signalCoods[i])
        signalTexts = ["","","",""]

        # display signal timer
        for i in range(0,noOfSignals):  
            signalTexts[i] = font.render(str(signals[i].signalText), True, white, black)
            screen.blit(signalTexts[i],signalTimerCoods[i])
        
        # create the vehicle
        if round(time.time()) % 5 == 0 and round(time.time()) != t:
            if not first and thread_count_a < 5:
                t = round(time.time())
                thread_a[thread_count_a] = threading.Thread(name="generateVehicles",target=generateVehicles, args=())    # Generating vehicles
                thread_a[thread_count_a].daemon = True
                thread_a[thread_count_a].start()
                screen.blit(background,(0,0))
                thread_count_a += 1
            first = False
            
        # display the vehicles
        for vehicle in simulation:  

            #move the vehicle
            vehicle.move()

            # rotate the image as it moves
            vehicle.draw(screen)

            # check if the vehicle has to stop because of a stopped vehicle in front of it
            collide(vehicle, simulation)

            # calling the signals to check if the vehicle must change its behviour on the basis of the nature of the signal
            signal_0(currentGreen, currentYellow, vehicle)
            signal_2(currentGreen, currentYellow, vehicle)
            signal_3(currentGreen, currentYellow, vehicle)
            signal_1(currentGreen, currentYellow, vehicle)

            

        pygame.display.update()

Main()