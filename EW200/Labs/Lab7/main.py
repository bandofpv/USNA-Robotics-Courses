import pygame

# pygame setup
pygame.init()
WIDTH = 500  # screen width
HEIGHT = 500  # screen height
screen = pygame.display.set_mode((WIDTH, HEIGHT))  # create screen object

# load our players (fish)
puffer_fish = pygame.image.load('assets/images/puffer_fish.png')

background = pygame.Surface((WIDTH, HEIGHT))  # create surface object
background.fill((66, 35, 223))  # fill the surface with blue
pygame.draw.rect(background, (255, 231, 143), (0, WIDTH-100, WIDTH, 100))  # draw sand on background surface
screen.blit(background, (0, 0))

running = True

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # RENDER YOUR GAME HERE
    screen.blit(puffer_fish, (WIDTH/2, HEIGHT/2))

    # flip() the display to put your work on screen
    pygame.display.flip()

pygame.quit()
