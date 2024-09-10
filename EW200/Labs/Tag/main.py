from player import Player
from timer import Timer
from button import Button
from background import *
from random import choice

# pygame setup
pygame.init()
clock = pygame.time.Clock()
WIDTH = 1080  # screen width
HEIGHT = 620  # screen height

# comment out either screen initializations to set manual size or full screen
# screen = pygame.display.set_mode((WIDTH, HEIGHT))  # create screen surface
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

# game constants
BLOCK_SIZE = (25, 25)
FPS = 60
GAME_TIME = 60
SPAWN_INTERVAL = 5
LONGEVITY = 10

# game flag variables
running = True
play = True
started = False
show_cursor = False
spawned = False
pro = False

# setup control keys
arrow_keys = (pygame.K_RIGHT, pygame.K_LEFT, pygame.K_UP)
wasd_keys = (pygame.K_d, pygame.K_a, pygame.K_w)

it = choice([True, False])  # randomize which player is it

# create Player instances on the bottom of the map
player1 = Player(screen.get_width() // 3, screen.get_height() - 50, BLOCK_SIZE, 'green', wasd_keys, it)
player2 = Player(2 * screen.get_width() // 3, screen.get_height() - 50, BLOCK_SIZE, 'blue', arrow_keys, not it)

# create Sprite Groups
platforms = pygame.sprite.Group()
collectibles = pygame.sprite.Group()

# create in game timer
game_timer = Timer(GAME_TIME, screen)

# create button instances
screen_centerx, screen_centery = screen.get_rect().centerx, screen.get_rect().centery
big_novice_button = Button((screen_centerx, screen_centery), 'Novice', 110, 'green', screen)
big_pro_button = Button((screen_centerx, screen_centery + 140), 'Pro', 110, 'red', screen)
small_novice_button = Button((screen_centerx, screen_centery - 60), 'Novice', 100, 'white', screen)
small_pro_button = Button((screen_centerx, screen_centery + 60), 'Pro', 100, 'white', screen)
quit_button = Button((screen_centerx, screen_centery + 170), 'Quit', 60, 'white', screen)

# game loop
while running:

    screen.fill('black')

    game_events = pygame.event.get()  # poll for events

    for event in game_events:
        if event.type == pygame.QUIT:  # quit game if user clicked on X to close window
            running = False

    # display start menu
    if not started:
        start_text(screen, 120)
        # if pressed on "novice" button, play game on novice mode
        if big_novice_button.click(game_events, 1.1, 'green'):
            started = True
            pro = False
        # if pressed on "pro" button, play game on pro mode
        elif big_pro_button.click(game_events, 1.1, 'red'):
            started = True
            pro = True
        # build game map and start game timer
        if started:
            build_border(screen, BLOCK_SIZE, platforms)
            build_game(BLOCK_SIZE, platforms, screen, FPS, pro)
            game_timer = Timer(GAME_TIME, screen)

    else:
        # if game is running, display game
        if play:
            show_cursor = display_cursor(show_cursor, game_events)  # determine whether to show cursor or not
            pygame.mouse.set_visible(show_cursor)

            # update game objects
            player1.update(platforms, player2, collectibles)
            player2.update(platforms, player1, collectibles)
            platforms.update()
            game_timer.update()
            collectibles.update(LONGEVITY)

            # spawn collectibles every SPAWN_INTERVAL
            if game_timer.timer // 1000 % SPAWN_INTERVAL == 0 and not spawned:
                spawn_collectible(screen, BLOCK_SIZE, collectibles)
                spawned = True
            elif game_timer.timer // 1000 % SPAWN_INTERVAL != 0:
                spawned = False

            # draw game objects
            game_timer.draw()
            collectibles.draw(screen)
            platforms.draw(screen)
            player1.draw(screen)
            player2.draw(screen)

            play = game_timer.play  # update game play status

        # if game is over
        else:
            # clean up
            player1.kill()
            player2.kill()
            [platform.kill() for platform in platforms]
            [collectible.kill() for collectible in collectibles]

            # display play again menu
            pygame.mouse.set_visible(True)  # show cursor
            # if pressed on "novice" button, play game on novice mode
            if small_novice_button.click(game_events, 1.1, 'green'):
                play = True
                pro = False
            # if pressed on "pro" button, play game on pro mode
            elif small_pro_button.click(game_events, 1.1, 'red'):
                play = True
                pro = True
            display_winner(screen, player1, player2, 130)  # display winner of the game
            # quit game if quit button clicked
            if quit_button.click(game_events, font_color='red'):
                running = False

            # play again
            if play:
                it = choice([True, False])  # randomize which player is it

                # create Player instances on the bottom of the map
                player1 = Player(screen.get_width() // 3, screen.get_height() - 50, BLOCK_SIZE, 'green',
                                 wasd_keys, it)
                player2 = Player(2 * screen.get_width() // 3, screen.get_height() - 50, BLOCK_SIZE, 'blue',
                                 arrow_keys, not it)

                # build game map and start game timer
                build_border(screen, BLOCK_SIZE, platforms)
                build_game(BLOCK_SIZE, platforms, screen, FPS, pro)
                game_timer = Timer(GAME_TIME, screen)
                show_cursor = False  # hide cursor

    pygame.display.flip()  # display game
    clock.tick(FPS)  # set fps

pygame.quit()
