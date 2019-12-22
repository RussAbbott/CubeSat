
from math import atan2, copysign, pi, sqrt
from os import path
from random import random, uniform

import pygame
from pygame.math import Vector2

"""
This code includes two primary classes.
Satellite: A class of which both CubeSat and Target are subclasses. ImpairedCubeSat is a subclass of CubeSat.
Sim: The simulation infrastructure
"""


class Satellite:
    """ A generic satellite """

    max_angle_change = 1  # degrees/frame
    sat_number = 0

    def __init__(self, pos=None, vel=None, angle=None, image=None):
        
        # pos, vel, and angle are with respect to the fixed window/plane
        self.position = pos if pos is not None else \
                        Sim.V2(uniform(100, Sim.window_width-100), uniform(100, Sim.window_height-100))
        self.velocity = vel if vel is not None else Sim.V2(uniform(-1, 1), uniform(-1, 1))
        self.angle = angle if angle is not None else uniform(-180, 180)

        current_directory = path.dirname(path.abspath(__file__))
        image_path = current_directory + '/' + 'images/' + image
        self.image = pygame.image.load(image_path)
        Satellite.sat_number += 1
        self.id = 'Tgt' if isinstance(self, Target) else (
                  'I-' if isinstance(self, ImpairedCubeSat) else
                  'C-') + str(self.sat_number)

    @staticmethod
    def limit_velocity(v2, max_velocity):
        magnitude = v2.magnitude()
        if magnitude > max_velocity:
            v2 *= max_velocity/magnitude
        return v2

    @staticmethod
    def normalize_angle(angle):
        return (angle + 180) % 360 - 180

    def update(self):
        """ Update CubeSat differently from the target. """
        pass

    def update_angle(self, correction):
        """
        Update the satellite's heading (it's angle). Limit the maximum
        change that can occur in any frame. If the entire correction
        does not occur during the current frame, it will continue
        (possibly adjusted) in the next frame.
        """
        if abs(correction) > Satellite.max_angle_change:
            correction = copysign(Satellite.max_angle_change, correction)
        new_angle = Satellite.normalize_angle(self.angle + correction)
        self.angle = new_angle

    def update_velocity(self, _correction):
        """ Update CubeSat's velocity differently from the target's velocity. """
        pass

    def update_position(self):
        self.position += self.velocity
        if Sim.sim.print_ids:
            if isinstance(self, Target):
                print()
            print(f'{self.id}: {Sim.roundV2(self.position)}; ', end='')


class CubeSat(Satellite):
    """ A CubeSat """

    max_velocity = 1.0  # pixels / frame

    def __init__(self, pos=None, vel=None, angle=None, image='CubeSat.png'):
        super().__init__(pos, vel, angle, image)

    def angle_correction(self):
        """ Compute CubeSat angle correction for the current frame update. """
        target_position = Sim.sim.target.position
        (rel_x, rel_y) = (target_position.x - self.position.x, target_position.y - self.position.y)
        rel_angle = (180 / pi) * (-atan2(rel_y, rel_x))
        correction = Satellite.normalize_angle(rel_angle - self.angle)
        return correction

    def repulsive_force(self, other):
        """
        Compute a virtual repulsive force from the other satellite so that
        CubeSat maintains a reasonable distance. The force decreases with
        the square of the distance between them. The actual numbers are
        arbitrary.  Doing it this way makes the relationship between the two
        smoother and more fluid. An alternative might have been to keep CubeSat
        a fixed distance away from the target.
        """
        dist_to_other = Sim.distance(self.position, other.position)
        # Don't divide by 0 if self.position very close to other.position.
        limited_dist_to_target = max(1.0, dist_to_other)
        # Divide distance by an arbitrary number to scale repulsive force to distance units.
        # Could use this to let the Target have a stronger repulsive force than the other CubeSats.
        divisor = 100
        # Repulsive force shrinks with the nth power of distance.
        n = 3
        repulsive_force = 1/(limited_dist_to_target/divisor)**n
        return repulsive_force

    def stay_away_from_other_sats(self):
        repulsive_aggregate = Sim.v2_zero()
        if isinstance(self, ImpairedCubeSat) and self.ticks > ImpairedCubeSat.directional_ticks_limit:
            return repulsive_aggregate
        for sat in Sim.sim.sats:
            direction = self.position - sat.position
            repulsive_aggregate += direction * self.repulsive_force(sat)
        return repulsive_aggregate

    def update(self):
        """
        Update CubeSat's angle, velocity, and position.
        CubeSat does not have a rotational velocity. It is always at a fixed angle,
        which changes frame-by-frame.
        """
        angle_correction = self.angle_correction()
        self.update_angle(angle_correction)

        velocity_correction = self.velocity_correction()
        self.update_velocity(velocity_correction)
        self.update_position()

    def update_velocity(self, correction):
        """
        Update CubeSat's velocity. Limit the maximum change that
        can occur in any frame. If the entire correction does not
        occur during the current frame, it will continue (possibly
        adjusted) in the next frame. Keep a safe distance from
        the target.
        """
        self.velocity += correction
        Satellite.limit_velocity(self.velocity, CubeSat.max_velocity)

    def velocity_correction(self):
        """ Compute CubeSat velocity correction for the current frame update. """
        target_position = Sim.sim.target.position
        desired_direction = target_position - self.position
        correction = desired_direction - self.velocity
        correction += self.stay_away_from_other_sats()
        return correction


class ImpairedCubeSat(CubeSat):
    """ A CubeSat that can't rotate and move directionally at the same time. """

    directional_ticks_limit = 20

    def __init__(self, pos=None, vel=None, angle=None, image='CubeSatImpaired.png'):
        self.ticks = 0
        super().__init__(pos, vel, angle, image)

    def angle_correction(self):
        """ Compute CubeSat angle correction for the current frame update. """
        self.velocity = Sim.v2_zero()
        correction = super().angle_correction()
        # If we are pointing to the target, switch to directional mode (ticks = 1).
        if abs(correction) < 1 and self.ticks == 0:
            self.ticks = 1
        return correction

    def update(self):
        """
        Update CubeSat's angle, velocity, and position.
        CubeSat does not have a rotational velocity. It is always at a fixed angle,
        which changes frame-by-frame.
        """
        # Update the angle when ticks == 0. Keep ticks at 0 until angle is correct.
        if self.ticks == 0:
            angle_correction = super().angle_correction()
            self.update_angle(angle_correction)
            # If we are pointing to the target, switch to directional mode (ticks = 1).
            if abs(angle_correction) < 1:
                self.ticks = 1
            return

        # Otherwise, update the velocity and position for ImpairedCubeSat.directional_ticks_limit
        velocity_correction = self.velocity_correction()
        self.update_velocity(velocity_correction)
        self.update_position()
        self.ticks = (self.ticks + 1) % ImpairedCubeSat.directional_ticks_limit


class Target(Satellite):
    """ The target objecct, treated as a subclass of Satellite. """

    # Probability of changing velocity on any frame.
    prob_velocity_change = 0.05  
    
    velocity_change = 1.0  # pixels / frame

    max_velocity = 1.9   # pixels / frame
    min_velocity = 0.75  # pixels / frame

    def __init__(self, pos=None, vel=None, image='ArUco_64_1.png', fixed=False):
        self.fixed = fixed
        if fixed:
            vel = Sim.v2_zero()
        super().__init__(pos, vel, image=image)

    def update(self):
        if not self.fixed:
            self.update_velocity()
        self.update_position()

    def update_velocity(self, _correction=None):
        """
        Update the target velocity--for this frame. These changes
        are arbitrary and get the target to move around the screen.
        """
        # Change direction every once in a while
        if random() < Target.prob_velocity_change:
            velocity_change = Sim.V2(uniform((-1)*Target.velocity_change, Target.velocity_change),
                                     uniform((-1)*Target.velocity_change, Target.velocity_change))
            self.velocity += velocity_change
            Satellite.limit_velocity(self.velocity, Target.max_velocity)

        # If too far away from any CubeSat, reverse direction
        dist_to_farthest_sat = max([Sim.distance(self.position, cubesat.position) for cubesat in Sim.sim.cubesats])
        if dist_to_farthest_sat > Sim.window_width * 0.6:
            self.velocity *= -0.01  

        # Ensure that the target is moving at a reasonable speed.
        # Allow it to move faster than CubeSat.
        # The actual numbers are arbitrary.
        if abs(self.velocity.x) < Target.min_velocity:
            self.velocity.x *= 2
        if abs(self.velocity.x) > Target.max_velocity:
            self.velocity.x *= 0.7
        if abs(self.velocity.y) < Target.min_velocity:
            self.velocity.y *= 2
        if abs(self.velocity.y) > Target.max_velocity:
            self.velocity.y *= 0.7


class Sim:
    """ The simulation framework """

    FPS = 50

    # Window dimensions, in pixels. These are also plane dimensions and are taken as plane units.
    # As both window and plane dimensions. (0,0) is at the upper left.
    window_width = 800   # pixels
    window_height = 800  # pixels

    # The simulation object itself is available here. It includes references to both
    # the CubeSats and the target
    sim = None

    def __init__(self, cubesats, target=None, print_ids=False):
        # Make this Sim object itself available in the Sim class.
        # The satellites use it to retrieve information about each other.
        # (That's a cheat.)
        Sim.sim = self
        self.print_ids = print_ids

        # Cannot do these at the Sim level.
        self.screen_center = Sim.V2(Sim.window_width, Sim.window_height) / 2

        # Used during recentering.
        self.point_to_be_centered = None
        self.recenter_correction = None
        # How close (in pixels) recentering must get to satisfy recentering.
        self.centered_enough = 5
        self.rescaling = False

        # Every once in a while, system gets stuck in pygame.init()
        print('Starting pygame.init()')
        pygame.init()
        print('Finished pygame.init()')
        
        pygame.display.set_caption("CubeSat Simulator")
        window_dimensions = (Sim.window_width, Sim.window_height)
        self.screen = pygame.display.set_mode(window_dimensions)
        self.clock = pygame.time.Clock()
        self.exit = False

        # Establish initial formation
        self.cubesats = cubesats
        self.target = target if target else Target()
        # Displays the satellites in front of the target and the
        # satellites themselves in reverse order of original list
        self.sats = [self.target] + list(reversed(self.cubesats))

    def add_obj_to_screen(self, obj):
        """ Update the screen "surface" before displaying it. """
        obj_display = pygame.transform.rotate(obj.image, obj.angle)
        rect = obj_display.get_rect()
        self.screen.blit(obj_display, obj.position - (rect.width/2, rect.height/2))

    @staticmethod
    def at_edge_of_screen(sat):
        """
        This is defined in Sim rather than in Satellite because it concerns
        the window within which the simulation is run, not the satellite, which
        may or may not be within the window boundaries.
        """
        return sat.position.x < 70 or sat.position.x > Sim.window_width-70 or \
               sat.position.y < 70 or sat.position.y > Sim.window_height - 70

    @staticmethod
    def distance(a, b):
        return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    def recenter_all(self):
        """
        If one satellite is out of bounds, recenter the average position of all the satellites.
        Don't select another point to recenter until the current point is recentered. Otherwise
        may jump back and forth among different average points on each frame.
        Take one step (frame) in the recentering process.
        """
        if self.point_to_be_centered is None:
            # Not currently recentering. Find the swarm center and recenter that point.
            sum_positions = Sim.v2_zero()
            for sat in Sim.sim.sats:
                sum_positions += sat.position
            swarm_center = sum_positions/(len(Sim.sim.sats))
            self.point_to_be_centered = swarm_center
            
            # If swarm_center is already centered, rescale.
            self.rescaling = self.point_to_be_centered == self.screen_center
            if self.rescaling:
                for sat in Sim.sim.sats:
                    sat.position = 0.997 * sat.position + 0.003 * self.screen_center
                self.point_to_be_centered = None
                return
            
            raw_correction = self.screen_center - self.point_to_be_centered
            max_speed = 5*max(CubeSat.max_velocity, Target.max_velocity)
            max_dir_speed = max([abs(raw_correction.x), abs(raw_correction.y)])
            coeff = min(max_speed, max_dir_speed)/max_dir_speed if max_dir_speed > 1 else 1
            self.recenter_correction = raw_correction * coeff

        for sat in Sim.sim.sats:
            sat.position += self.recenter_correction
        self.point_to_be_centered += self.recenter_correction
        raw_correction = self.screen_center - self.point_to_be_centered
        if max([abs(raw_correction.x), abs(raw_correction.y)]) < self.centered_enough:
            self.point_to_be_centered = self.recenter_correction = None

    def refresh_screen(self):
        """
        Refresh the screen. Create a black background and
        put the two objects in the surface. Then make it visible.
        """
        self.screen.fill((0, 0, 0))
        if self.point_to_be_centered or self.rescaling:
            font = pygame.font.Font(None, 36)
            label = "Rescaling" if self.rescaling else "Recentering"
            self.rescaling = False
            text = font.render(label, 1, (150, 250, 250))
            self.screen.blit(text, Sim.V2(50, 50))
        for sat in self.sats:
            self.add_obj_to_screen(sat)
        pygame.display.flip()

    @staticmethod
    def roundV2(v2: Vector2, prec=2):
        return Sim.V2(round(v2.x, prec), round(v2.y, prec))

    def run(self):
        """ The main loop """
        while not self.exit:
            # Event queue.  Not used here, but standard in pygame applications.
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            if self.point_to_be_centered is not None or any(Sim.at_edge_of_screen(sat) for sat in self.sats):
                self.recenter_all()
            else:
                for sat in self.sats:
                    sat.update()

            self.refresh_screen()
            self.clock.tick(Sim.FPS)

        pygame.quit()

    @staticmethod
    def V2(x, y):
        # noinspection PyArgumentList
        return Vector2(float(x), float(y))

    @staticmethod
    def v2_zero():
        return Sim.V2(0, 0)



if __name__ == '__main__':
    # When the Target is not fixed, it moves slightly faster  than the CubeSats
    # Select one of the runs below.

    # Can adjust repulsive force parameters in CubeSat.repulsive_force()
    # If too strong and CubeSats are pushed to two screen edges, system freezes
    # because it can't recenter.

    # A standard run: one CubeSat and a moving target
    # Sim().run([CubeSat()], Target())

    # An impared CubeSat and a moving target. Impaired CubeSats can't move directionally
    # while they are rotating.
    # Sim().run([ImpairedCubeSat()], Target())

    # A normal CubeSat displayed in front of an impaired CubeSate.
    # Sim().run([CubeSat(), ImpairedCubeSat()], Target())

    # A normal CubeSat in front of a impaired CubeSate and another one behind. The Target is fixed.
    # With a fixed Target, the three CubeSats form a symmetric triangle around the Target.
    # Then motion essentially ceases.
    # Sim().run([CubeSat(), ImpairedCubeSat(), CubeSat()], Target(fixed=True))

    # A swarm of CubeSats. The Target is fixed or not. With a fixed Target,
    # the CubeSats create a symmetric formation around the Target, and motion mainly ceases.
    # If more than 8 CubeSats, formation may not become symmetric. The designated initial
    # positions force scaling at the start to demonstrate that features.
    sim = Sim([CubeSat(pos=Sim.V2(850, 50)), ImpairedCubeSat(pos=Sim.V2(0, 0)), CubeSat(), ImpairedCubeSat(),
               CubeSat(pos=Sim.V2(50, 850)), ImpairedCubeSat(pos=Sim.V2(900, 900)), CubeSat(), ImpairedCubeSat()],
              Target(fixed=True, pos=Sim.V2(850, 850)))
    sim.run()
