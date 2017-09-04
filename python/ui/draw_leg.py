#!/usr/bin/env python

import math
import sys
import time

import pygame


class LegDisplay(object):
    def __init__(self, screen, xy=None, width=350, height=350, scale=2.0):
        self.screen = screen
        if xy is None:
            xy = [0., 0.]
        self.xy = xy
        self.height = height
        self.width = width
        self.limits = [
            [-0.706, 0.706],
            [0., 1.5708],
            [-2.3736, 0.0]]
        self.lengths = [11., 54., 72.]  # inches
        self.colors = [
            (255, 0, 0),
            (0, 255, 0),
            (0, 0, 255)]
        self.scale = scale
        self.outline_color = (255, 255, 255)
        self.background_color = (0, 0, 0)
        self.base_angles = [0., 1.4663, -1.4485]  # radians, + = up
        self.outline_width = 1

    def clear(self):
        self.screen.fill(
            self.background_color,
            (self.xy, (self.width * 2, self.height)))

    def draw_outline(self):
        pygame.draw.rect(
            self.screen, self.outline_color,
            (self.xy, (self.width, self.height)),
            self.outline_width)
        pygame.draw.rect(
            self.screen, self.outline_color,
            ((self.xy[0] + self.width, self.xy[1]), (self.width, self.height)),
            self.outline_width)

    def draw_leg(self, angles):
        # compute points [xyz, xyz, xyz]
        pts = [[0, 0, 0], ]
        x = 0
        y = 0
        z = 0
        # hip
        ex = self.lengths[0] * self.scale
        ez = 0
        sh = math.sin(angles[0])
        ch = math.cos(angles[0])
        x += ex * ch
        y += ex * sh
        z += ez
        pts.append([x, y, z])
        # thigh
        a = self.base_angles[1] - angles[1]
        ex = math.cos(a) * self.lengths[1] * scale
        ez = -math.sin(a) * self.lengths[1] * scale
        x += ex * ch
        y += ex * sh
        z += ez
        pts.append([x, y, z])
        # knee
        a = self.base_angles[2] - angles[2] - angles[1]
        ex = math.cos(a) * self.lengths[2] * scale
        ez = -math.sin(a) * self.lengths[2] * scale
        x += ex * ch
        y += ex * sh
        z += ez
        pts.append([x, y, z])
        # rotate pts about hip
        #for i in xrange(len(pts)):
        #    pts[i][1] = pts[i][0] * math.sin(angles[0])
        #    pts[i][0] *= math.cos(angles[0])

        # draw aa lines
        pp = [0, 0, 0]
        sx0 = self.xy[0] + 10.
        sy0 = self.xy[1] + self.height / 2.
        sx1 = self.xy[0] + self.width + 10.
        sy1 = sy0
        for i in xrange(len(pts) - 1):
            pp = pts[len(pts) - i - 2]
            p = pts[len(pts) - i - 1]
            pygame.draw.aaline(
                self.screen, self.colors[len(colors) - i - 1],
                (pp[0] + sx0, pp[2] + sy0), (p[0] + sx0, p[2] + sy0))
            pygame.draw.aaline(
                self.screen, self.colors[len(colors) - i - 1],
                (pp[0] + sx1, pp[1] + sy1), (p[0] + sx1, p[1] + sy1))
            pp = p

        """
        # draw right view
        # draw top view
        x = self.xy[0] + 10.
        y = self.xy[1] + self.height / 2.
        # draw hip
        ex = x + self.lengths[0] * self.scale
        ey = y
        pygame.draw.aaline(self.screen, self.colors[0], (x, y), (ex, ey))
        x, y = ex, ey

        # draw thigh
        a = self.base_angles[1] - angles[1]
        ex = x + math.cos(a) * self.lengths[1] * scale
        ey = y - math.sin(a) * self.lengths[1] * scale
        pygame.draw.aaline(self.screen, self.colors[1], (x, y), (ex, ey))
        x, y = ex, ey

        # draw knee
        a = self.base_angles[2] - angles[2] - angles[1]
        ex = x + math.cos(a) * self.lengths[2] * scale
        ey = y - math.sin(a) * self.lengths[2] * scale
        pygame.draw.aaline(self.screen, self.colors[2], (x, y), (ex, ey))
        """

        self.screen.blit(
            myfont.render(
                "H %0.2f" % math.degrees(angles[0]),
                False, colors[0]), (20 + self.xy[0], 20 + self.xy[1]))
        self.screen.blit(
            myfont.render(
                "T %0.2f" % math.degrees(angles[1]),
                False, self.colors[1]), (20 + self.xy[0], 40 + self.xy[1]))
        self.screen.blit(
            myfont.render(
                "K %0.2f" % math.degrees(angles[2]),
                False, self.colors[2]), (20 + self.xy[0], 60 + self.xy[1]))

    def draw(self, angles):
        self.clear()
        self.draw_outline()
        self.draw_leg(angles)
        pass

scale = 2.0  # pixels per inches
lengths = [11., 54., 72.]  # inches
base_angles = [0., 1.4663, -1.4485]  # radians, + = up
speeds = [0.01, 0.01, 0.01]
limits = [
    [-0.706, 0.706],
    [0., 1.5708],
    [-2.3736, 0.0]]
angles = [0., 0., 0.]
colors = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255)]

sw, sh = (1024, 768)
sx = 10.
sy = sh / 2.


pygame.init()
myfont = pygame.font.SysFont('ubuntu', 18)

screen = pygame.display.set_mode((sw, sh))

ld = LegDisplay(screen)

clock = pygame.time.Clock()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

    # update angles
    for i in xrange(len(angles)):
        angles[i] += speeds[i]
        if angles[i] > limits[i][1] or angles[i] < limits[i][0]:
            speeds[i] *= -1

    # draw leg
    screen.fill((0, 0, 0))
    """
    x, y = sx, sy
    # draw hip
    ex = x + lengths[0] * scale
    ey = y
    pygame.draw.aaline(screen, colors[0], (x, y), (ex, ey))
    x, y = ex, ey

    # draw thigh
    a = base_angles[1] - angles[1]
    ex = x + math.cos(a) * lengths[1] * scale
    ey = y - math.sin(a) * lengths[1] * scale
    pygame.draw.aaline(screen, colors[1], (x, y), (ex, ey))
    x, y = ex, ey

    # draw knee
    a = base_angles[2] - angles[2] - angles[1]
    ex = x + math.cos(a) * lengths[2] * scale
    ey = y - math.sin(a) * lengths[2] * scale
    pygame.draw.aaline(screen, colors[2], (x, y), (ex, ey))

    screen.blit(
        myfont.render(
            "H %0.2f" % math.degrees(angles[0]),
            False, colors[0]), (20, 20))
    screen.blit(
        myfont.render(
            "T %0.2f" % math.degrees(angles[1]),
            False, colors[1]), (20, 40))
    screen.blit(
        myfont.render(
            "K %0.2f" % math.degrees(angles[2]),
            False, colors[2]), (20, 60))
    """
    ld.draw(angles)

    pygame.display.update()
    clock.tick(60)  # throttle to 60 fps
