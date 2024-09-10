import pygame


class Timer:
    def __init__(self, game_time, screen):
        """Creates instance of Timer class.

        Args:
            game_time (int): Length of game in seconds.
            screen (pygame.Surface): pygame.Surface of the `screen` to draw the remaining time left in game.
        """
        self.game_font = pygame.font.Font('assets/fonts/rush.otf', 100)
        self.text = self.game_font.render('Hi', 1, (255, 0, 0))
        self.rect = self.text.get_rect()
        self.rect.midtop = screen.get_rect().midtop
        self.start_time = pygame.time.get_ticks()
        self.time = pygame.time.get_ticks()
        self.timer = 0
        self.game_time = (game_time + 1) * 1000  # adds one second to account for delayed game initialization
        self.screen = screen
        self.play = True
        self.color = 'white'

    def update(self):
        """Updates the remaining time left in game.

        Given the `game_time` in seconds, this function will count down the text to zero. The font will change from
        white to red for the last ten seconds left in game. Once the time runs outs, the game will end.

        """
        self.timer = (self.game_time + self.start_time) - self.time  # calculate remaining time in game
        # update the timer if there is remaining time
        if self.timer > 0:
            self.time = pygame.time.get_ticks()
            self.color = 'white'
            if self.timer < 11000:  # change font color to red for last 10 seconds left in game
                self.color = 'red'
        # if no remaining time
        else:
            self.timer = 0
            self.play = False
        self.text = self.game_font.render(str(self.timer//1000), 1, self.color)
        self.rect = self.text.get_rect()
        self.rect.midtop = self.screen.get_rect().midtop

    def draw(self):
        """Draws the remaining time left in game on the `screen` Surface

        """
        self.screen.blit(self.text, self.rect)
