import os
import pygame
from math import copysign, atan2, pi, sqrt
from pygame.math import Vector2
from random import random, uniform


class Satellite:
    def __init__(self, pos, vel, angle, image, window):
        self.position = pos
        self.velocity = vel
        self.angle = angle  # angle is respect to the plane
        self.image = image
        self.max_velocity = 1
        self.max_angle_change = 2
        self.window_width = window[0]
        self.window_height = window[1]


    def generate_velocity_correction(self, target_position):
        desired_direction = target_position - self.position
        correction = desired_direction - self.velocity
        # If too close, move in the opposite direction.
        dist_to_target = sqrt((self.position.x-target_position.x)**2 + (self.position.y-target_position.y)**2)
        # The distance to leave between CubeSat and the target is set here arbitrarily as 7.5% of the average
        # of the window height and width.
        move_toward_target = 1 if dist_to_target > (self.window_height + self.window_width)/15 else -5
        correction = int(move_toward_target) * self.limit(correction)
        return correction

    def generate_angle_correction(self, target_position):
        (rel_x, rel_y) = (target_position.x - self.position.x, target_position.y - self.position.y)
        rel_angle = (180 / pi) * (-atan2(rel_y, rel_x))
        correction = (rel_angle - self.angle) % 360
        return self.normalize_angle(correction)  # + 180) % 360 - 180

    def limit(self, v2):
        if abs(v2.x) > self.max_velocity:
            v2 *= self.max_velocity/abs(v2.x)
        if abs(v2.y) > self.max_velocity:
            v2 *= self.max_velocity/abs(v2.y)
        return v2

    @staticmethod
    def normalize_angle(angle):
        return (angle + 180) % 360 - 180

    def update_velocity(self, correction):
        if correction is None:
            # Update target motion

            # Every once in a while, change direction
            if random() < 0.01:
                self.velocity = Game.V2(uniform(-2, 2), uniform(-2, 2))

            # Go go off the screen
            delta_x = min(1, self.window_width**3/self.position.x**5) +\
                      max(-1, self.window_width**3/(self.position.x-self.window_width)**5)
            delta_y = min(1, self.window_height**3/self.position.y**5) +\
                      max(-1, self.window_height**3/(self.position.y-self.window_height)**5)
            self.velocity += Game.V2(delta_x, delta_y)

            # Ensure that the target is moving at a reasonable velocity
            if abs(self.velocity.x) < 0.4:
                self.velocity.x *= 2
            if abs(self.velocity.x) > 1:
                self.velocity.x *= 0.7
            if abs(self.velocity.y) < 0.4:
                self.velocity.y *= 2
            if abs(self.velocity.y) > 1:
                self.velocity.y *= 0.7
        else:
            # Update CubeSat motion
            self.velocity += correction  # * dt
            self.velocity = self.limit(self.velocity)

    def update_angle(self, correction):
        correction1 = correction if abs(correction) < self.max_angle_change else \
                      copysign(self.max_angle_change, correction)
        angle1 = (self.angle + correction1) % 360
        angle2 = self.normalize_angle(angle1)
        self.angle = angle2

    def update_position(self):
        self.position += self.velocity


class Game:
    # frames-per-second
    FPS = 50

    def __init__(self):
        pygame.init()
        pygame.display.set_caption("CubeSat Simulator")
        width = 800
        height = 800
        window = (width, height)
        self.screen = pygame.display.set_mode(window)
        self.clock = pygame.time.Clock()

        current_dir = os.path.dirname(os.path.abspath(__file__))
        cubeSat_image_path = os.path.join(current_dir, "CubeSat.png")
        # Draw a square to represents the target
        cubeSat_image = pygame.image.load(cubeSat_image_path)
        pos = Game.V2(150, 600)
        vel = Game.V2(-1.0, 0.750)
        angle = 110
        self.cubesat = Satellite(pos, vel, angle, cubeSat_image, window)
        target_image_path = os.path.join(current_dir, "ArUcoTarget.png")
        # Draw an ArUco marker, to represents the target
        target_image = pygame.image.load(target_image_path)
        pos = Game.V2(600, 120)
        vel = Game.V2(random(), random())
        angle = 50
        self.target = Satellite(pos, vel, angle, target_image, window)
        self.exit = False

    def refresh_screen(self):
        self.screen.fill((0, 0, 0))
        targeted = pygame.transform.rotate(self.target.image, self.target.angle)
        rect = targeted.get_rect( )
        self.screen.blit(targeted, self.target.position - (rect.width / 2, rect.height / 2))
        rotated = pygame.transform.rotate(self.cubesat.image, self.cubesat.angle)
        rect = rotated.get_rect( )
        self.screen.blit(rotated, self.cubesat.position - (rect.width / 2, rect.height / 2))
        pygame.display.flip( )

    @staticmethod
    def roundVec2(v2: Vector2):
        return Game.V2(round(v2.x, 2), round(v2.y, 2))

    def run(self):
        while not self.exit:
            # dt represents a frame increment
            # Move on to the next frame
            dt = 1/self.FPS
            # Event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            velocity_correction = self.cubesat.generate_velocity_correction(self.target.position)
            self.cubesat.update_velocity(velocity_correction * dt)
            self.cubesat.update_position()

            angle_correction = self.cubesat.generate_angle_correction(self.target.position)
            self.cubesat.update_angle(angle_correction)

            self.target.update_velocity(None)
            self.target.update_position()

            self.refresh_screen()
            self.clock.tick(self.FPS)
        pygame.quit()

    @staticmethod
    def V2(x, y):
        # noinspection PyArgumentList
        return Vector2(x, y)


if __name__ == '__main__':
    game = Game()
    game.run()
