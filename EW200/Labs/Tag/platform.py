import pygame


class Platform(pygame.sprite.Sprite):
    def __init__(self, x, y, size, side):
        """Creates Platform Sprite.

        Note:
            Uses top left corner convention for tile position.

        Args:
            x (int): x-axis location of Platform Sprite.
            y (int): y-axis location of Platform Sprite.
            size (tuple[int, int]): Tuple[width, height] of Platform tile size.
            side (str): String ('center', 'left', or 'right') of Platform tile side.

        """
        super().__init__()
        self.image = pygame.image.load(f'assets/images/tile_{side}.png')
        self.image = pygame.transform.smoothscale(self.image, size)
        self.rect = self.image.get_rect()
        self.rect.topleft = (x, y)
        self.xvelocity = 0

    def draw(self, screen):
        """Draws the Platform Sprite on the `screen` Surface.

        Args:
            screen (pygame.Surface): pygame.Surface of the screen to draw the Platform Sprite on.
        """
        screen.blit(self.image, self.rect)


class MovingPlatform(pygame.sprite.Sprite):
    def __init__(self, x, y, size, side, speed, move_time, direction):
        """Creates MovingPlatform Sprite

        Note:
            Uses top left corner convention for tile position.

        Args:
            x (int): x-axis location of MovingPlatform Sprite.
            y (int): y-axis location of MovingPlatform Sprite.
            size (tuple[int, int]): Tuple[width, height] of MovingPlatform tile size.
            side (str): String ('center', 'left', or 'right') of MovingPlatform tile side.
            speed (int): Speed of MovingPlatform Sprite.
            move_time (int): Time MovingPlatform Sprite moves in one direction in milliseconds.
            direction (str): String('left' or 'right') of starting moving direction.
        """
        super().__init__()
        self.image = pygame.image.load(f'assets/images/tile_half_{side}.png')
        self.image = pygame.transform.smoothscale(self.image, size)
        self.rect = self.image.get_rect()
        self.rect.topleft = (x, y)
        self.speed = speed
        self.xvelocity = self.speed  # used to get speed of moving platform to update Player speed
        self.move_time = move_time
        self.time = pygame.time.get_ticks()
        self.start_time = pygame.time.get_ticks()
        self.direction = direction

    def move_right(self):
        """Moves MovingPlatform Sprite to the right

        """
        self.xvelocity = self.speed
        self.rect.move_ip(self.speed, 0)

    def move_left(self):
        """Moves MovingPlatform Sprite to the left

        """
        self.xvelocity = -self.speed
        self.rect.move_ip(-self.speed, 0)

    def update(self):
        self.time = pygame.time.get_ticks()
        if self.direction == 'right':
            self.move_right()
            # after moving for set move_time, change direction
            if self.time - self.start_time > self.move_time:
                self.direction = 'left'
                self.start_time = self.time
        else:
            self.move_left()
            # after moving for set move_time, change direction
            if self.time - self.start_time > self.move_time:
                self.direction = 'right'
                self.start_time = self.time

    def draw(self, screen):
        """Draws the MovingPlatform Sprite on the `screen` Surface.

        Args:
            screen (pygame.Surface): pygame.Surface of the screen to draw the Platform Sprite on.
        """
        screen.blit(self.image, self.rect)
