import pygame


class Collectible(pygame.sprite.Sprite):
    def __init__(self, x, y):
        """Creates Collectible Sprite.

        Note:
            Uses top left corner convention for tile position.

        Args:
            x (int): x-axis location of Collectible Sprite.
            y (int): y-axis location of Platform Sprite.
        """
        super().__init__()
        self.image = pygame.image.load(f'assets/images/tile_exclamation.png')
        self.image = pygame.transform.smoothscale(self.image, [25, 25])
        self.alpha = 0
        self.image.set_alpha(self.alpha)
        self.rect = self.image.get_rect()
        self.rect.topleft = (x, y)
        self.start_time = pygame.time.get_ticks()
        self.time = pygame.time.get_ticks()
        self.spawned = False
        self.collected = False

    def update(self, longevity):
        """Updates appearance of Collectible.

        This function will spawn (fade onto screen) a new Collectible upon instantiation, fade away the Collectible
        once the given `longevity` time passes, and will remove the Collectible if a Player collides with it.

        Args:
            longevity (int): How long the collectible will last in seconds.
        """
        self.image.set_alpha(self.alpha)  # set the transparency of the Collectible
        self.time = pygame.time.get_ticks()
        # spawn in the Collectible if not spawned
        if not self.spawned:
            self.spawn()
        # fade Collectible away when it expires
        if self.time - self.start_time > longevity * 1000:
            self.die()
        # kill the Collectible if Player collects it
        if self.collected:
            self.kill()

    def spawn(self):
        """Fades Collectible onto the screen

        """
        self.alpha += 5
        # update spawned status once fully visible
        if self.alpha >= 255:
            self.spawned = True

    def die(self):
        """Fades the Collectible out of the screen

        """
        self.alpha -= 1
        # kill Collectible after fading away
        if self.alpha <= 0:
            self.kill()

    def draw(self, screen):
        """Draws the Collectible Sprite on the `screen` Surface.

        Args:
            screen (pygame.Surface): pygame.Surface of the screen to draw the Collectible Sprite on.
        """
        screen.blit(self.image, self.rect)
