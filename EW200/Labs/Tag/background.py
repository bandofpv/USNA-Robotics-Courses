import pygame
import pygame.freetype
from platform import Platform, MovingPlatform
from collectible import Collectible
from random import randrange, triangular, normalvariate, choice


def build_border(screen, size, platform_group):
    """Builds border surrounding the screens perimeter.

    This function creates multiple Platform Sprites around the screen's perimeter. The top, left, and right edges of
    the screen are placed right outside the visible portion of the screen. The bottom edge is visible along the bottom
    side of the screen.

    Args:
        screen (pygame.Surface): pygame.Surface of game screen.
        size (tuple[int, int]): Tuple(width, height) of Platform tile size to surround the screen's perimeter.
        platform_group (pygame.sprite.Group): pygame.sprite.Group for Platform Sprites.

    """
    # get dimensions of screen and platform
    screen_width = screen.get_width()
    screen_height = screen.get_height()
    platform_width = size[0]
    platform_height = size[1]
    for x in range(0, screen_width, platform_width):  # loop along width of screen
        platform_group.add(Platform(x, -platform_height, size, 'center'))  # top edge
        platform_group.add(Platform(x, screen_height-platform_height, size, 'center')) # bottom edge
    for y in range(0, screen_height, platform_height):  # loop along height of screen
        platform_group.add(Platform(-platform_width, y, size, 'center'))  # left edge
        platform_group.add(Platform(screen_width, y, size, 'center'))  # right edge


def static_platform(x, y, size, length, platform_group):
    """Builds Platform of given `length`.

    This function creates multiple Platform Sprites to take up a given horizontal length and adds it to the given 
    `platform_group`.
    
    Note:
        Uses top left corner convention for tile position.

    Args:
        x (int): x-axis location to place Platform.
        y (int): y-axis location to place Platform.
        size (tuple[int, int]): Tuple[width, height] of Platform tile size.
        length (int): Length of Platform.
        platform_group (pygame.sprite.Group): pygame.sprite.Group for Platform Sprites.

    """
    platform_width = size[0]
    # create left, center, and right platforms along the length of the platform
    for i in range(x + platform_width, x + length - platform_width, platform_width):
        platform_group.add(Platform(i, y, size, 'center'))
    platform_group.add(Platform(x, y, size, 'left'))
    platform_group.add(Platform(x + length - platform_width, y, size, 'right'))


def moving_platform(x, y, size, length, platform_group, speed, move_time, direction):
    """Builds MovingPlatform of given `length`, `speed`, `move_time`, and `direction`.

        This function creates multiple Platform Sprites to take up a given horizontal length and adds it to the given
        `platform_group`.

        Note:
            Uses top left corner convention for tile position.

        Args:
            x (int): x-axis location to place MovingPlatform.
            y (int): y-axis location to place MovingPlatform.
            size (tuple[int, int]): Tuple[width, height] of MovingPlatform tile size.
            length (int): Length of MovingPlatform.
            platform_group (pygame.sprite.Group): pygame.sprite.Group for MovingPlatform Sprites.
            speed (int): Speed of MovingPlatform.
            move_time (int): Time MovingPlatform moves in one direction in milliseconds.
            direction (str): String('left' or 'right') of starting moving direction.

        """
    platform_width = size[0]
    # create left, center, and right platforms along the length of the platform
    # the -15 accounts for error within the in game time changes
    for i in range(x + platform_width - 10, x + length - platform_width, platform_width - 10):
        platform_group.add(MovingPlatform(i, y, size, 'center', speed, move_time, direction))
    platform_group.add(MovingPlatform(x, y, size, 'left', speed, move_time, direction))
    platform_group.add(MovingPlatform(x + length - platform_width, y, size, 'right', speed, move_time, direction))


def calc_length(length, speed, move_time, fps):
    """Calculates the moving length of a MovingPlatform.

    Takes in the length, speed, moving time, and fps of a MovingPlatform to calculate the total moving length.

    Args:
        length (int): Length of MovingPlatform.
        speed (int): Speed of MovingPlatform.
        move_time (int): Time MovingPlatform moves in one direction in milliseconds.
        fps (int): Frames per second of game.

    Returns:
        int: calculated moving length.

    """
    moving_length = length + speed * fps * (move_time / 1000)
    return int(moving_length)


def build_game(size, platform_group, screen, fps, pro):
    """Randomly generates a new game map on `screen`.

    Given the `screen`, this function will randomly generate static and moving platforms to create a new playable map.

    Args:
        size (tuple[int, int]): Tuple(width, height) of Platform or MovingPlatform tile size.
        platform_group (pygame.sprite.Group): pygame.sprite.Group for Platform or MovingPlatform Sprites.
        screen (pygame.Surface): pygame.Surface of game screen.
        fps (int): Frames per second of game.
        pro (bool): True if pro difficulty, False if novice difficulty.

    """
    # get dimensions of screen and platform
    screen_width = screen.get_width()
    screen_height = screen.get_height()
    platform_width = size[0]
    platform_height = size[1]
    # loop through equally spaced levels along screen
    for y in range(screen_height - platform_height - 100, 0, -100):
        first_plat = True
        was_moving = False
        count = 0
        # loop along length of screen
        while 0 <= count < screen_width:
            spacing = randrange(50, 100, 5)  # randomize platform spacing
            speed = int(triangular(0, 2)) if pro else 0  # randomize speed if pro difficulty
            moving_time = randrange(700, 1700, 50)  # randomize moving_time for a moving platform
            direction = choice(['right', 'left'])  # randomize starting direction for a moving platform
            # randomize whether the first platform is connected to left edge of screen
            if round(triangular(0, 1, 0.75)) and first_plat:
                length = round(normalvariate(200, 100))  # randomize length of platform
                length = 50 if length < 50 else 400 if length > 400 else length  # cap length between 50 and 400
                static_platform(0 - platform_width, y, size, length, platform_group)
                count += length - platform_width
            else:
                # first platform shouldn't move
                if speed and not was_moving and not first_plat:
                    length = round(normalvariate(100, 20))  # randomize length of moving platform
                    length = 50 if length < 50 else 150 if length > 150 else length  # cap length between 50 and 150
                    moving_length = calc_length(length, speed, moving_time // speed, fps)
                    # if moving platform will move outside of screen, make a static platform
                    if count + moving_length > screen_width:
                        static_platform(count + spacing, y, size, length, platform_group)
                        count += spacing + length
                        continue
                    spacing = platform_width + 1  # set a short spacing for a moving platform
                    was_moving = True
                    # set direction of moving platform
                    if direction == 'right':
                        moving_platform(count + spacing, y, size, length, platform_group, speed, moving_time // speed,
                                        direction)
                    else:
                        moving_platform(count + spacing + moving_length - length, y, size, length, platform_group,
                                        speed, moving_time // speed, direction)
                    length = moving_length
                # if platform doesn't have a speed, create a static platform
                else:
                    # if previous platform (platform to the left) is a moving platform, shorten spacing
                    if was_moving:
                        spacing = platform_width + 1
                        was_moving = False
                    length = round(normalvariate(200, 100))  # randomize length of moving platform
                    length = 50 if length < 50 else 400 if length > 400 else length  # cap length between 50 and 400
                    static_platform(count + spacing, y, size, length, platform_group)
                count += spacing + length
            first_plat = False


def spawn_collectible(screen, size, collectible_group):
    """Randomly spawns a collectible.

    Args:
        screen (pygame.Surface): pygame.Surface of game screen.
        size (tuple[int, int]): Tuple(width, height) of Collectible tile size.
        collectible_group (pygame.sprite.Group): pygame.sprite.Group for Collectible Sprites.
    """
    # get dimensions of screen and platform
    screen_width = screen.get_width()
    screen_height = screen.get_height()
    platform_width = size[0]
    platform_height = size[1]
    x = randrange(0, screen_width - platform_width, 50)  # randomize x position along width of screen
    # randomize y position on same level as platforms
    y = randrange(screen_height - platform_height * 2, 0, -100)
    collectible_group.add(Collectible(x, y))


def start_text(screen, font_size):
    """Displays 'Select Difficulty:' text at the start of the game.

    Args:
        screen (pygame.Surface): pygame.Surface of game screen.
        font_size (int): Size of font.

    """
    font = pygame.font.Font('assets/fonts/rush.otf', font_size)
    text = font.render(f'Select Difficulty:', 1, f'white')
    rect = text.get_rect()
    # Display text at center top of display screen
    rect.center = (screen.get_rect().centerx, screen.get_rect().centery - 125)
    screen.blit(text, rect)


def display_winner(screen, player1, player2, font_size):
    """Displays winner of the game.

    Displays the Player who is not `it` as the winner at top of screen and in the `color` of the Player.

    Args:
        screen (pygame.Surface): pygame.Surface of game screen.
        player1 (Player): Instance of first player.
        player2 (Player): Instance of second player.
        font_size (int): Size of font.

    """
    font = pygame.font.Font('assets/fonts/rush.otf', font_size)
    if player1.it:  # If player1 is it, player2 wins
        text = font.render(f'{player2.color.title()} Wins', 1, f'{player2.color}')
    else:  # If player2 is it, player1 wins
        text = font.render(f'{player1.color.title()} Wins', 1, f'{player1.color}')
    rect = text.get_rect()
    # Display text at center top of display screen
    rect.center = (screen.get_rect().centerx, screen.get_rect().centery - 175)
    screen.blit(text, rect)


def display_cursor(visible, game_events):
    """Determines whether to display cursor based on given `visible` and `game_events` arguments.

    Used to display or not display cursor during gameplay. See `main.py` for example usage.

    Args:
        visible (bool): True if mouse is visible, False if otherwise.
        game_events (list[pygame.Event]): List[pygame.event] of pygame events.

    Returns:
        bool: True if `visible` is True. True if `visible` is False and escape button clicked or detected mouse motion.
            False if `visible` is True and mouse button is clicked. False if no user input provided.

    """
    for e in game_events:
        if e.type == pygame.KEYDOWN:
            # if escape key pressed and cursor is not visible, make visible
            if e.key == pygame.K_ESCAPE and not visible:
                return True
        # if mouse moved and cursor is not visible, make visible
        elif e.type == pygame.MOUSEMOTION and not visible:
            return True
        # if mouse clicked and cursor is visible, make not visible
        elif e.type == pygame.MOUSEBUTTONDOWN and visible:
            return False
    if visible:
        return True
    return False  # make cursor not visible if no user input
