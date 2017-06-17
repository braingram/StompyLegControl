#!/usr/bin/env python

import sys
import time

import pygame
import serial

import pycomando


cmds = {
    0: 'estop(byte)',  # 0 = off, 1 = soft, 2 = hard
    1: 'heartbeat',
    2: 'pwm(float,float,float)',  # hip thigh knee
    3: 'adc=uint32,uint32,uint32',
    4: 'adc_target(uint32,uint32,uint32)',
    5: 'pwm=int32,int32,int32',
    6: 'pid=float,float,float',
}

com = pycomando.Comando(serial.Serial('/dev/ttyACM0', 9600))
cmd = pycomando.protocols.command.CommandProtocol()
com.register_protocol(0, cmd)
mgr = pycomando.protocols.command.EventManager(cmd, cmds)
ns = mgr.build_namespace()

pygame.init()
myfont = pygame.font.SysFont('ubuntu', 18)

if pygame.joystick.get_count() < 1:
    raise IOError("No joysticks found")
j = pygame.joystick.Joystick(0)
j.init()

w, h = (640, 480)
hs = h / 65535.
npts = 200

mode = 'pwm'
#mode = 'target'

jmap = {
    0: lambda x, y, z: -z * 0.5,  # hip pwm
    1: lambda x, y, z: -y * 0.5,  # thigh pwm
    2: lambda x, y, z: -x * 0.5,  # knee pwm
}
screen = pygame.display.set_mode((w, h))
delay = 0.010  # s
mini_delay = 1  # ms

ns.heartbeat()
last_heartbeat = time.time()
#ns.estop(0)  turn estop off (enable)

pots = []


def on_adc(hip, thigh, knee):
    pots.append((hip.value, thigh.value, knee.value))


def on_pwm(hip, thigh, knee):
    #print("PWM", hip, thigh, knee)
    pass


def on_pid(hip, thigh, knee):
    #print("PID", hip, thigh, knee)
    pass


mgr.on('adc', on_adc)
mgr.on('pwm', on_pwm)
mgr.on('pid', on_pid)

last_update = time.time() - delay
while True:
    if (time.time() - last_heartbeat > 0.5):
        ns.heartbeat()
        last_heartbeat = time.time()
    com.handle_stream()
    if (time.time() - last_update < delay):
        pygame.time.delay(mini_delay)
        continue
    last_update = time.time()
    modified = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        elif event.type == pygame.JOYAXISMOTION:
            #print event
            if mode == 'target':
                if event.axis == 2:
                    t = abs(event.value) * (64017 - 947) + 947
                    print("Target:", t)
                    ns.adc_target(int(t), int(t), int(t))
            elif mode == 'pwm':
                v = j.get_axis(0)
                x, y, z = j.get_axis(0), j.get_axis(1), j.get_axis(2)
                print('xyz', x, y, z)
                hpwm = jmap[0](x, y, z)
                tpwm = jmap[1](x, y, z)
                kpwm = jmap[2](x, y, z)
                print('pwm', hpwm, tpwm, kpwm)
                ns.pwm(hpwm, tpwm, kpwm)
        elif event.type == pygame.JOYBUTTONDOWN:
            #print event
            if event.button == 1:
                ns.estop(0)
        elif event.type == pygame.JOYBUTTONUP:
            if event.button == 1:
                ns.estop(1)
    if len(pots) > npts:
        modified = True
        pots = pots[-npts:]
    if modified:
        screen.fill((0, 0, 0))
        hpts = []
        tpts = []
        kpts = []
        sums = [0, 0, 0]
        # draw pots
        for (i, p) in enumerate(pots):
            x = int(i * (w / float(npts)))
            hpts.append((x, h - int(p[0] * hs)))
            tpts.append((x, h - int(p[1] * hs)))
            kpts.append((x, h - int(p[2] * hs)))
            sums[0] += p[0]
            sums[1] += p[1]
            sums[2] += p[2]
        #print("Hip   Mean: %s" % (sums[0] / float(npts), ))
        #print("Thigh Mean: %s" % (sums[1] / float(npts), ))
        #print("Knee  Mean: %s" % (sums[2] / float(npts), ))
        screen.blit(
            myfont.render(
                "Hip   Mean: %s" % (sums[0] / float(npts), ),
                False, (255, 0, 0)), (20, 20))
        screen.blit(
            myfont.render(
                "Thigh Mean: %s" % (sums[1] / float(npts), ),
                False, (0, 255, 0)), (20, 40))
        screen.blit(
            myfont.render(
                "Knee  Mean: %s" % (sums[2] / float(npts), ),
                False, (0, 0, 255)), (20, 60))
        pygame.draw.lines(
            screen, (255, 0, 0), False, hpts)
        pygame.draw.lines(
            screen, (0, 255, 0), False, tpts)
        pygame.draw.lines(
            screen, (0, 0, 255), False, kpts)
        pygame.display.update()
