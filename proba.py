import pygame
pygame.init()
screen = pygame.display.set_mode([100, 100])
flag = True
x = 0
y = 0
while flag:
    if pygame.event.type() == pygame.MOUSEBUTTONDOWN:
        x, y = pygame.mouse.get_pos()
        print(x, y)
