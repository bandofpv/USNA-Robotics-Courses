import pygame


class Player(pygame.sprite.Sprite):
    def __init__(self, x, y, size, color, keys, it):
        """Creates Player Sprite.

        Args:
            x (int): x-axis location of Player Sprite.
            y (int): y-axis location of Platform Sprite.
            size (tuple[int, int]): Tuple[width, height] of Player tile size.
            color (str): String ('green' or 'blue') of Player color.
            keys (tuple[pygame.Locals, pygame.Locals, pygame.Locals]): Tuple[right_key, left_key, up_key] of Player
                controls.
            it (bool): True if it, False if not it.
        """
        super().__init__()
        self.color = color
        self.it_body = pygame.image.load(f'assets/images/{self.color}_it_body.png')
        self.it_body = pygame.transform.smoothscale(self.it_body, size)
        self.body = pygame.image.load(f'assets/images/{self.color}_body.png')
        self.body = pygame.transform.smoothscale(self.body, size)
        self.image = self.body
        self.rect = self.image.get_rect()
        self.right_key, self.left_key, self.up_key = keys
        self.it = it
        self.rect.center = (x, y)
        self.collide_timer = pygame.time.get_ticks()
        self.collectable_timer = pygame.time.get_ticks()
        # set default player values
        self.pause = False
        self.speed = 3
        self.collected = False
        self.xvelocity, self.yvelocity = 0, 0
        self.jumpvelocity = -15
        self.fallvelocity = 10
        if self.it:  # if player is it, change image and speed
            self.image = self.it_body
            self.pause = True
            self.speed = 4
            self.jumpvelocity = -16

    def update(self, platform_group, opponent, collectable_group):
        """Updates Player location.

        This function incorporates both the collision and physics algorithm used for the Player Sprite. Collisions are
        calculated for Player to Player, Player to Platform, and Player to Collectible.

        Args:
            platform_group (pygame.sprite.Group): pygame.sprite.Group for Platform Sprites.
            opponent (Player): Player instance for opponent. 
            collectable_group (pygame.sprite.Group): pygame.sprite.Group for Collectible Sprites.
        """
        # check if player collides with another player
        opponent_collide = pygame.Rect.colliderect(self.rect, opponent.rect)
        current_time = pygame.time.get_ticks()
        # if player collides with another player and it player is no longer frozen
        if opponent_collide and (current_time - self.collide_timer) > 3000:
            # if it, change status to not it
            if self.it:
                self.it = False
                self.pause = False
                self.speed = 3
                self.jumpvelocity = -15
                self.collide_timer = current_time
                self.image = self.body
            # if not it, change status to it and freeze
            else:
                self.it = True
                self.pause = True
                self.speed = 4
                self.jumpvelocity = -16
                self.collide_timer = current_time
                self.image = self.it_body
                # unfreeze it player after 3 seconds
        if (current_time - self.collide_timer) > 3000:
            self.pause = False
        # check collisions on all four sides of player
        top_collide = self.calc_collision(0, self.yvelocity, platform_group)
        bottom_collide = self.calc_collision(0, self.fallvelocity, platform_group)
        right_collide = self.calc_collision(self.speed, 0, platform_group)
        left_collide = self.calc_collision(-self.speed, 0, platform_group)
        self.xvelocity = 0
        key = pygame.key.get_pressed()
        if key[self.left_key]:
            # if left collision, snap to right side of platform
            if left_collide and self.yvelocity == 0:
                self.rect.left = left_collide.rect.right
            # if no left collision, move left
            elif not left_collide:
                self.xvelocity -= self.speed
        if key[self.right_key]:
            # if right collision, snap to left side of platform
            if right_collide and self.yvelocity == 0:
                self.rect.right = right_collide.rect.left
            # if no right collision, move right
            elif not right_collide:
                self.xvelocity += self.speed
        # if up key pressed and on ground, jump
        if key[self.up_key] and self.yvelocity == 0 and bottom_collide:
            self.yvelocity += self.jumpvelocity
        # if in the air, fall
        elif self.yvelocity < self.fallvelocity and not bottom_collide:
            self.yvelocity += 1
        # if on ground
        elif bottom_collide:
            self.rect.bottom = bottom_collide.rect.top  # snap to ground
            self.yvelocity = 0  # stop falling
            self.xvelocity += bottom_collide.xvelocity  # set xvelocity to xvelocity of touching platform
        # if in air and top collides, snap to top and stop going up
        if self.yvelocity < 0 and top_collide:
            self.rect.top = top_collide.rect.bottom
            self.yvelocity = 0
        # if player is paused (just got tagged), stop in place
        if self.pause:
            self.xvelocity = 0
            self.yvelocity = 0
        self.rect.move_ip(self.xvelocity, self.yvelocity)  # move player based on x and y velocities
        # check collision with collectibles
        collectable_collide = pygame.sprite.spritecollideany(self, collectable_group)
        # if player collides with collectible and is not it, give player speed and jump bonus
        if collectable_collide and not self.it:
            collectable_collide.collected = True
            self.collected = True
            self.speed = 5
            self.jumpvelocity = -17
            self.collectable_timer = current_time
        # after 5 seconds from collecting a collectible, reset speed and jump
        if self.collected and (current_time - self.collectable_timer) > 5000:
            self.collected = False
            self.speed = 3
            self.jumpvelocity = -15

    def calc_collision(self, x, y, platform_group):
        """Predicts future collisions.
        
        Accepts `x` and `y` velocities to move a ghost Player to check a collision one frame before the visible Player.

        Args:
            x (int): Displacement of ghost Player along the x-axis.
            y (int): Displacement of ghost Player along the y-axis.
            platform_group (pygame.spring.Group): pygame.sprite.Group for Collectable Sprites.

        Returns:
            pygame.sprite.Sprite: Sprite of Platform in `platform_group` that the ghost Player collided with. None if
                no collision.

        """
        self.rect.move_ip((x, y))  # move the Player
        collide = pygame.sprite.spritecollideany(self, platform_group)  # check collision
        self.rect.move_ip((-x, -y))  # move Player back
        return collide  # returns Sprite of Platform if collision

    def draw(self, screen):
        """Draws the Player Sprite on the `screen` Surface.

        Args:
            screen (pygame.Surface): pygame.Surface of the screen to draw the Player Sprite on.
        """
        screen.blit(self.image, self.rect)
