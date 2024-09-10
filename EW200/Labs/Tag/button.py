import pygame


class Button:
    def __init__(self, center, title, font_size, font_color, screen):
        """Creates instance of Button class.

        Args:
            center (tuple[int, int]): Tuple[x, y] coordinates to place center of Button text.
            title (str): Text of Button.
            font_size (int): Size of Button text.
            font_color (pygame.Color): Color of Button text.
            screen (pygame.Surface): pygame.Surface of the `screen` to draw the Button text.
        """
        self.center = center
        self.title = title
        self.font_size = font_size
        self.font_color = font_color
        self.screen = screen
        self.font = pygame.font.Font('assets/fonts/rush.otf', self.font_size)
        self.text = self.font.render(title, 1, self.font_color)
        self.rect = self.text.get_rect()
        self.collision_rect = pygame.Rect(0, 0, 0, 0)

    def calc_collision(self):
        """Calculates collision rectangle and Button text offset.

        Calculates and updates collision rectangle based on dimensions of visible text. Calculates and updates y-axis
        offset between the collision rectangle and Button text rectangle. Shifts Button text rectangle based on
        calculated offset.

        """
        # create a copy of the button rect and locate it in the top left corner of the screen
        calc_rect = pygame.Rect((0, 0), (self.rect.width, self.rect.height))
        # create a rectangle around only the visible portion of the text
        glyph_rect = pygame.mask.from_surface(self.text).get_bounding_rects()[0]
        # calculate the y offset between the two rectangles
        offset = glyph_rect.top - calc_rect.top
        # set collision rectangle based on width of button rectangle and height of glyph rectangle
        self.collision_rect = pygame.Rect(0, 0, self.rect.width, glyph_rect.height)
        self.collision_rect.center = self.center
        # align button rectangle location with collision rectangle location and account for y offset
        self.rect.midtop = (self.collision_rect.centerx, self.collision_rect.top)
        self.rect.top -= offset

    def click(self, game_events, scale=1.0, font_color=None):
        """Checks if Button was clicked on.

        Changes Button text size based on `scale` and changes Button text color based on `font_color` when mouse hovers
        over Button.

        Args:
            game_events (list[pygame.Event]): List[pygame.event] of pygame events.
            scale (float): Scale factor of Button text when mouse hovers over Button. Defaults to 1.0 (no change to
                size).
            font_color (str): pygame.Color of Button text when mouse hovers over Button. Defaults to `font_color` of
                original Button instance (no change to color).

        Returns:
            bool: True if Button was clicked on, False if otherwise.

        """
        font_color = font_color or self.font_color  # set font_color to provided color or default color
        # if mouse cursor collides with collision_rect, change font size and color
        if self.collision_rect.collidepoint(pygame.mouse.get_pos()):
            self.font = pygame.font.Font('assets/fonts/rush.otf', int(self.font_size * scale))
            self.text = self.font.render(self.title, 1, font_color)
            self.rect = self.text.get_rect()
            self.calc_collision()
            for e in game_events:
                if e.type == pygame.MOUSEBUTTONDOWN:  # If mouse click, return True
                    return True
        # if mouse cursor doesn't collide with collision_rect, set to default font size and color
        else:
            self.font = pygame.font.Font('assets/fonts/rush.otf', self.font_size)
            self.text = self.font.render(self.title, 1, self.font_color)
            self.rect = self.text.get_rect(center=self.center)
            self.calc_collision()
        self.screen.blit(self.text, self.rect)
