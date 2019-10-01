import numpy as np
import pygame
import time

clock = pygame.time.Clock()

while True:
    print("---------")
    print(clock.get_time())
    clock.tick_busy_loop(10)
    print(clock.get_time())
    # time.sleep(0.5)