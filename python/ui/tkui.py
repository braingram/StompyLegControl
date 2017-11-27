#!/usr/bin/env python
"""
leg name (label) [stringvar]
state (label) [stringvar]
estop (button) [booleanvar]

Leg top view (canvas)
Leg side view (canvas)

hip value/length/angle? (slider)
thigh value/length/angle? (slider)
knee value/length/angle? (slider)
calf value/length/angle? (slider)

pid settings p/i/d/min/max (numeric inputs)

calibration (per leg)
[done]adc min/max of hip/thigh/knee
calf slope and offset
[done] pwm extend/retract min/max of hip/thigh/knee
"""

import datetime
import math
import sys
import time
import Tkinter as tk

import serial

import numpy
import pycomando

foot_centers = {  # body coords in inches
    'fl': (-77.0, 126.0),
    'fr': (77.0, 126.0),
    'ml': (-93.0, 0.0),
    'mr': (93.0, 0.0),
    'rl': (-77.0, -126.0),
    'rr': (77.0, -126.0),
    'fake': (93.0, 0.0),
}

neighbors = {
    'fr': ('fl', 'mr', 'ml', 'rr', 'rl'),
    'mr': ('fr', 'rr', 'fl', 'ml', 'rl'),
    'rr': ('mr', 'rl', 'fr', 'fl', 'ml'),
    'fl': ('fr', 'ml', 'mr', 'rl', 'rr'),
    'ml': ('fl', 'rl', 'fr', 'mr', 'rr'),
    'rl': ('ml', 'rr', 'mr', 'fl', 'fr'),
}


class Foot(object):
    max_restriction = 0.9
    restriction_threshold = 0.15
    step_size = 20.0  # inches

    center = (66.0, 0.0)  # foot center in foot coords
    radius = 42.0  # radius of movement circle (inches)

    # centered on foot_center (self.center)
    eps = numpy.log(0.1) / radius

    stance_velocity = 8.0  # inches per second
    swing_velocity = 16.0
    lower_velocity = 16.0
    lift_velocity = 16.0

    lower_height = -15.0
    lift_height = -5.0
    #velocity_scale = 1.0

    close_enough = 5.0

    def __init__(self, name):
        self.name = name
        #self.center = foot_centers[name]
        self.state = 'stance'
        self.target = None
        self.last_update = time.time()

    def restriction(self, position):
        cx, cy = self.center
        x, y, z = position
        d = ((cx - x) ** 2. + (cy - y) ** 2.) ** 0.5
        return numpy.exp(-self.eps * (d - self.radius))

    def update(self, position, t):
        dt = t - self.last_update
        r = self.restriction(position)
        x, y, z = position
        ns = None
        if self.state == 'swing':
            # check against target position
            tx, ty = self.target
            d = ((tx - x) ** 2. + (ty - y) ** 2.) ** 0.5
            # TODO also check for increase in distance
            if d < self.close_enough:
                # if there, move to lower
                ns = 'lower'
        elif self.state == 'lower':
            # lower leg until z at lower_height
            if z <= self.lower_height:
                # if there, enter stance
                ns = 'stance'
        elif self.state == 'lift':
            # lift leg until z at lift_height
            if z >= self.lift_height:
                # if there, enter swing
                ns = 'swing'
        elif self.state == 'stance':
            # continue moving...
            pass
        self.last_update = t
        return r, ns


# setting this to true sends plans in leg coordinates (multi-joint)
# if this is false, plans will be in sensor coordinates (single joint)
leg_coords = True
log_data = True

log_filename = None
if log_data and log_filename is None:
    dt = datetime.datetime.now()
    log_filename = dt.strftime('%y%m%d_%H%M%S.csv')

log_file = None
if log_data:
    print("Logging to %s" % log_filename)
    log_file = open(log_filename, 'w')

    def lf(k, *vs):
        t = time.time()
        log_file.write("%s,%s,%s\n" % (t, k, ",".join(map(str, vs))))
else:
    lf = lambda k, *vs: None

cmds = {
    0: 'estop(byte)',  # 0 = off, 1 = soft, 2 = hard
    1: 'heartbeat',
    2: 'pwm(float,float,float)',  # hip thigh knee
    3: 'adc=uint32,uint32,uint32,uint32',
    4: 'adc_target(uint32,uint32,uint32)',
    5: 'pwm_value=int32,int32,int32',
    6: 'pid=float,float,float,float,float,float,float,float,float',
    #7: 'plan(byte,byte,float,float,float,float,float,float,float,uint32)',
    7: 'plan(byte,byte,float,float,float,float,float,float,float)',
    8: 'enable_pid(bool)',
    9: 'xyz_values=float,float,float',
    10: 'angles=float,float,float,float,bool',
    11: 'set_pid(byte,float,float,float,float,float)',
    12: 'loop_time=uint32',
    13: 'leg_number(byte)=byte',
    14: 'pwm_limits(byte,float,float,float,float)',
    15: 'adc_limits(byte,float,float)',
    16: 'calf_scale(float,float)',
}

calibrations = {
    5: [  # mr calibration: 170807
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        ('adc_limits', (0, 7072, 52161)),
        ('adc_limits', (1, 3587, 59747)),
        ('adc_limits', (2, 9040, 59069)),
        ('set_pid', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        ('set_pid', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        ('set_pid', (2, 2.0, 3.0, 0.0, -8192, 8192)),
    ],
    6: [  # fr calibration: 170904
        # 20% symmetric deadband, 100% max, 13 bit
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        ('adc_limits', (0, 6155, 51979)),
        ('adc_limits', (1, 2218, 60632)),
        ('adc_limits', (2, 3417, 54144)),
        ('set_pid', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        ('set_pid', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        ('set_pid', (2, 2.0, 3.0, 0.0, -8192, 8192)),
        # calf scale
    ],
    7: [  # fake leg
        ('pwm_limits', (0, 0, 8192, 0, 8192)),
        ('pwm_limits', (1, 0, 8192, 0, 8192)),
        ('pwm_limits', (2, 0, 8192, 0, 8192)),
        ('pwm_limits', (2, 0, 8192, 0, 8192)),
        ('adc_limits', (0, 0, 65535)),
        ('adc_limits', (1, 0, 65535)),
        ('adc_limits', (2, 0, 65535)),
        ('set_pid', (0, 5.0, 0.0, 0.0, -8192, 8192)),
        ('set_pid', (1, 5.0, 0.0, 0.0, -8192, 8192)),
        ('set_pid', (2, 5.0, 0.0, 0.0, -8192, 8192)),
        # calf scale
    ],
}

LIMITS = [
    [-0.706, 0.706],
    [0., 1.5708],
    [-2.3736, 0.0]]  # radians

BASE_ANGLES = [0., 1.4663, -1.4485]  # radians, + = up

LENGTHS = [11., 54., 72.]  # inches

#COLORS = [
#    (255, 0, 0),
#    (0, 255, 0),
#    (0, 0, 255)]
COLORS = [
    'red',
    'green',
    'blue']

LEG_NAMES = {
    0: 'Undefined',
    1: 'Front-Left',
    2: 'Middle-Left',
    3: 'Rear-Left',
    4: 'Rear-Right',
    5: 'Middle-Right',
    6: 'Front-Right',
    7: 'Fake',
}


class LegDisplay(object):
    """two canvases, draw leg from angles"""
    def __init__(self, root):
        self.scale = 2.0
        self.root = root
        self.width = 300
        self.height = 300
        self.sx = 10.
        self.sy = self.height / 2.
        self.top_canvas = tk.Canvas(
            self.root, width=self.width, height=self.height,
            background='black')
        self.side_canvas = tk.Canvas(
            self.root, width=self.width, height=self.height,
            background='black')
        self.top_canvas.pack(side=tk.LEFT)
        self.side_canvas.pack(side=tk.LEFT)
        self.top_lines = {}
        self.side_lines = {}
        self._inited = False
        self._xyz = None

    def init(self):
        if self._inited:
            return
        # TODO draw limits
        #self.top_canvas
        # restriction circle
        #self.side_canvas
        # restriction circle (as lines)
        # lift height, lower height
        self.side_canvas.create_line(
            0, self.sy - Foot.lift_height * self.scale,
            self.width, self.sy - Foot.lift_height * self.scale,
            fill='yellow')
        self.side_canvas.create_line(
            0, self.sy - Foot.lower_height * self.scale,
            self.width, self.sy - Foot.lower_height * self.scale,
            fill='yellow')
        self._inited = True

    def draw_restriction(self, r):
        if self._xyz is None:
            return
        if 'restriction' not in self.top_lines:
            pass

    def draw_state(self, state):
        pass

    def draw_leg_angles(self, angles):
        # TODO restriction, state, plan
        # angles [hip, thigh, knee, calf, is_valid?]
        if not self._inited:
            self.init()
        # compute points [xyz, xyz, xyz]
        pts = [[0, 0, 0], ]
        x = 0
        y = 0
        z = 0
        # hip
        ex = LENGTHS[0] * self.scale
        ez = 0
        sh = math.sin(angles[0])
        ch = math.cos(angles[0])
        x += ex * ch
        y += ex * sh
        z += ez
        pts.append([x, y, z])
        # thigh
        a = BASE_ANGLES[1] - angles[1]
        ex = math.cos(a) * LENGTHS[1] * self.scale
        ez = -math.sin(a) * LENGTHS[1] * self.scale
        x += ex * ch
        y += ex * sh
        z += ez
        pts.append([x, y, z])
        # knee
        a = BASE_ANGLES[2] - angles[2] - angles[1]
        ex = math.cos(a) * LENGTHS[2] * self.scale
        ez = -math.sin(a) * LENGTHS[2] * self.scale
        x += ex * ch
        y += ex * sh
        z += ez
        pts.append([x, y, z])
        # rotate pts about hip
        #for i in xrange(len(pts)):
        #    pts[i][1] = pts[i][0] * math.sin(angles[0])
        #    pts[i][0] *= math.cos(angles[0])

        # draw or move lines
        # draw aa lines
        pp = [0, 0, 0]
        ns = ['hip', 'thigh', 'knee']
        for i in xrange(len(pts) - 1):
            pp = pts[i]
            p = pts[i+1]
            n = ns[i]
            if n not in self.side_lines:
                self.side_lines[n] = self.side_canvas.create_line(
                    pp[0] + self.sx, pp[2] + self.sy,
                    p[0] + self.sx, p[2] + self.sy,
                    fill=COLORS[i])
            else:  # else move line
                self.side_canvas.coords(
                    self.side_lines[n],
                    pp[0] + self.sx, pp[2] + self.sy,
                    p[0] + self.sx, p[2] + self.sy)
            if n not in self.top_lines:
                self.top_lines[n] = self.top_canvas.create_line(
                    pp[0] + self.sx, self.sy - pp[1],
                    p[0] + self.sx, self.sy - p[1],
                    fill=COLORS[i])
            else:  # else move line
                self.top_canvas.coords(
                    self.top_lines[n],
                    pp[0] + self.sx, self.sy - pp[1],
                    p[0] + self.sx, self.sy - p[1])
        # draw load as rectangle (height ~ load)
        # side: p[0] + self.sx, p[2] + self.sy
        # top: p[0] + self.sx, self.sy - p[1]
        lbs = angles[3]
        self._xyz = (p[0], p[1], p[2])
        sx, sy = p[0] + self.sx, p[2] + self.sy
        tx, ty = p[0] + self.sx, self.sy - p[1]
        sx += 10
        w = 10
        h = max(1, int(lbs * 0.1))
        if lbs > 50:
            fc = 'red'
        else:
            fc = 'blue'
        if 'calf' not in self.top_lines:
            # top left bottom right
            self.top_lines['calf'] = self.top_canvas.create_rectangle(
                (tx, ty - h, tx + w, ty), fill=fc)
        else:
            self.top_canvas.coords(
                self.top_lines['calf'],
                tx, ty - h, tx + w, ty)
            self.top_canvas.itemconfig(
                self.top_lines['calf'],
                fill=fc)
        if 'calf' not in self.side_lines:
            self.side_lines['calf'] = self.side_canvas.create_rectangle(
                (sx, sy - h, sx + w, sy), fill=fc)
        else:
            self.side_canvas.coords(
                self.side_lines['calf'],
                sx, sy - h, sx + w, sy)
            self.side_canvas.itemconfig(
                self.side_lines['calf'],
                fill=fc)


class SliderDisplay(object):
    def __init__(self, label, root, min_value, max_value, value=0):
        self.root = root
        self.var = tk.DoubleVar()
        self.var.set(value)
        self.scale = tk.Scale(
            self.root, variable=self.var,
            from_=min_value, to=max_value,
            state=tk.DISABLED, label=label)
        self.scale.pack(side=tk.LEFT)


class PIDFrame(object):
    def __init__(self, root, label, cb=None):
        # pid settings
        self.frame = tk.Frame(root)
        self.frame.pack(side=tk.TOP)
        self.label = label
        tk.Label(self.frame, text=self.label + "  ").pack(side=tk.LEFT)
        # p, i, d, minv, maxv
        tk.Label(self.frame, text="P:").pack(side=tk.LEFT)
        self.p_entry = tk.Entry(self.frame)
        self.p_entry.pack(side=tk.LEFT)
        tk.Label(self.frame, text="I:").pack(side=tk.LEFT)
        self.i_entry = tk.Entry(self.frame)
        self.i_entry.pack(side=tk.LEFT)
        tk.Label(self.frame, text="D:").pack(side=tk.LEFT)
        self.d_entry = tk.Entry(self.frame)
        self.d_entry.pack(side=tk.LEFT)
        #tk.Label(self.frame, text="MIN:").pack(side=tk.LEFT)
        #self.min_entry = tk.Entry(self.frame)
        #self.min_entry.pack(side=tk.LEFT)
        #tk.Label(self.frame, text="MAX:").pack(side=tk.LEFT)
        #self.max_entry = tk.Entry(self.frame)
        #self.max_entry.pack(side=tk.LEFT)
        self.commit = tk.Button(
            self.frame, text="Commit", command=self.commit_values)
        self.commit.pack(side=tk.LEFT)
        self.cb = cb

    def commit_values(self):
        if self.cb is None:
            return
        p = float(self.p_entry.get())
        i = float(self.i_entry.get())
        d = float(self.d_entry.get())
        #minv = float(self.min_entry.get())
        #maxv = float(self.max_entry.get())
        minv = -8192
        maxv = 8192
        self.cb(self.label, p, i, d, minv, maxv)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = '/dev/ttyACM0'
    com = pycomando.Comando(serial.Serial(port, 9600))
    cmd = pycomando.protocols.command.CommandProtocol()
    com.register_protocol(0, cmd)
    mgr = pycomando.protocols.command.EventManager(cmd, cmds)
    ns = mgr.build_namespace()
    ns.estop(2)  # set estop to disable leg

    foot = Foot('unknown')
    foot.restriction_control = False

    def send_plan(axis, speed):
        if axis < 0 or axis > 2:
            return
        if leg_coords:
            p = [0, 2, 0., 0., 0., 0., 0., 0., 0.]
        else:
            p = [0, 0, 0., 0., 0., 0., 0., 0., 0.]
        if speed != 0:
            p[0] = 1  # velocity mode
            if speed > 0:
                p[axis + 2] = 1.
            elif speed < 0:
                p[axis + 2] = -1.
            p[-1] = abs(speed)
        print("plan", p)
        lf('plan', *p)
        ns.plan(*p)

    def stop():
        p = [0, 0, 0., 0., 0., 0., 0., 0., 0.]
        ns.plan(*p)

    root = tk.Tk()

    root.wm_title('leg name here [estop setting]')
    header_frame = tk.Frame(root)
    header_frame.pack(side=tk.TOP)
    # estop [button]
    estop = tk.BooleanVar()
    estop.set(True)

    def estop_changed(*args):
        if estop.get():
            nv = 1
        else:
            nv = 0
        print("ESTOP: %s" % nv)
        lf('estop', nv)
        ns.estop(nv)
        if nv == 0:
            ns.enable_pid(True)

    def set_pid(l, p, i, d, minv, maxv):
        ls = ['HIP', 'THIGH', 'KNEE']
        if l not in ls:
            return
        li = ls.index(l)
        lf('set_pid', li, p, i, d, minv, maxv)
        ns.set_pid(li, p, i, d, minv, maxv)

    estop.trace("w", estop_changed)
    estop_check = tk.Checkbutton(header_frame, text="Estop", variable=estop)
    estop_check.pack(side=tk.LEFT)
    # state
    state_title = tk.Label(header_frame, text="State:")
    state = tk.StringVar()
    state_label = tk.Label(header_frame, textvariable=state)
    state_title.pack(side=tk.LEFT)
    state_label.pack(side=tk.LEFT)
    # leg name?
    legname_title = tk.Label(header_frame, text="Leg Name:")
    legname = tk.StringVar()
    legname_label = tk.Label(header_frame, textvariable=legname)
    legname_title.pack(side=tk.LEFT)
    legname_label.pack(side=tk.LEFT)
    foot_frame = tk.Frame(root)
    fx = tk.StringVar()
    fx_title = tk.Label(foot_frame, text="X:")
    fx_label = tk.Label(foot_frame, textvariable=fx)
    fx_title.pack(side=tk.LEFT)
    fx_label.pack(side=tk.LEFT)
    fy = tk.StringVar()
    fy_title = tk.Label(foot_frame, text="Y:")
    fy_label = tk.Label(foot_frame, textvariable=fy)
    fy_title.pack(side=tk.LEFT)
    fy_label.pack(side=tk.LEFT)
    fz = tk.StringVar()
    fz_title = tk.Label(foot_frame, text="Z:")
    fz_label = tk.Label(foot_frame, textvariable=fz)
    fz_title.pack(side=tk.LEFT)
    fz_label.pack(side=tk.LEFT)
    fl = tk.StringVar()
    fl_title = tk.Label(foot_frame, text="L:")
    fl_label = tk.Label(foot_frame, textvariable=fl)
    fl_title.pack(side=tk.LEFT)
    fl_label.pack(side=tk.LEFT)
    fr = tk.StringVar()
    fr_title = tk.Label(foot_frame, text="R:")
    fr_label = tk.Label(foot_frame, textvariable=fr)
    fr_title.pack(side=tk.LEFT)
    fr_label.pack(side=tk.LEFT)
    foot_frame.pack(side=tk.TOP)
    leg_frame = tk.Frame(root)
    leg_frame.pack(side=tk.TOP)
    leg_display = LegDisplay(leg_frame)
    hadc = SliderDisplay('hip', leg_frame, 0, 65535)
    tadc = SliderDisplay('thigh', leg_frame, 0, 65535)
    kadc = SliderDisplay('knee', leg_frame, 0, 65535)
    cadc = SliderDisplay('calf', leg_frame, 0, 65535)
    # pid settings
    hip_pid_frame = PIDFrame(root, "HIP", set_pid)
    thigh_pid_frame = PIDFrame(root, "THIGH", set_pid)
    knee_pid_frame = PIDFrame(root, "KNEE", set_pid)
    # TODO calibration settings
    speed_frame = tk.Frame(root)
    tk.Label(speed_frame, text="Speed: ").pack(side=tk.LEFT)
    speed = tk.Entry(speed_frame)
    speed.delete(0, tk.END)
    if leg_coords:
        speed.insert(0, "1.5")
    else:
        speed.insert(0, "1500")
    speed.pack(side=tk.LEFT)
    speed_frame.pack(side=tk.TOP)

    root.bind("<KeyPress-Control_L>", lambda e: estop.set(False))
    root.bind("<KeyRelease-Control_L>", lambda e: estop.set(True))
    # fuck steve jobs
    root.bind("<KeyPress-Left>", lambda e: estop.set(False))
    root.bind("<KeyRelease-Left>", lambda e: estop.set(True))
    root.bind("<FocusOut>", lambda e: estop.set(True))

    def keypress(event):
        #print(vars(event))
        c = event.keysym.lower()
        if c in '1234567890':
            print("execute canned motion")
            s = float(speed.get())
            if c == '1':
                # move +x
                a = 0
            elif c == '2':
                a = 0
                s *= -1.0
            elif c == '3':
                a = 1
            elif c == '4':
                a = 1
                s *= -1.0
            elif c == '5':
                a = 2
            elif c == '6':
                a = 2
                s *= -1.0
            else:
                return
            send_plan(a, s)
            root.after(5000, stop)
            return
        if c in 'rty':  # hip thigh knee forward
            a = 'rty'.index(c)
            d = 1.
        elif c in 'fgh':  # hip thigh knee backward
            a = 'fgh'.index(c)
            d = -1.
        elif c == 'p':
            if foot.restriction_control:
                print("Disabling restriction control")
                foot.restriction_control = False
                lf("restriction_control", 0)
                stop()
                return
            else:
                print("Enabling restriction control")
                foot.restriction_control = True
                lf("restriction_control", 1)
                p = [1, 2, 0., 0., 0., 0., 0., 0., 1.]
                p[3] = -foot.stance_velocity
                ns.plan(*p)
                foot.state = 'stance'
                print("new state: %s[%s]" % (foot.state, p))
                return
        else:
            return
        sd = bool(event.state & 0x001)  # check shift
        s = float(speed.get())
        if sd:
            s *= 2.0
        send_plan(a, s * d)

    def keyrelease(event):
        if event.keysym.lower() in 'rtyfgh':  # hip, thigh, knee
            send_plan(0, 0)  # send stop

    root.bind("<KeyPress>", keypress)
    root.bind("<KeyRelease>", keyrelease)

    def on_adc(hip, thigh, knee, calf):
        h = hip.value
        t = thigh.value
        k = knee.value
        c = calf.value
        lf("adc", h, t, k, c)
        hadc.var.set(h)
        tadc.var.set(t)
        kadc.var.set(k)
        cadc.var.set(c)

    global max_loop_time
    max_loop_time = 0

    def on_loop_time(t0):
        global max_loop_time
        if t0.value > max_loop_time:
            max_loop_time = t0.value
        print("Loop time (us): %s[%s]" % (t0.value, max_loop_time))

    def on_pid(ho, to, ko, hs, ts, ks, he, te, ke):
        lf(
            "pid", ho.value, to.value, ko.value,
            hs.value, ts.value, ks.value,
            he.value, te.value, ke.value)

    def on_angles(hip, thigh, knee, calf, is_valid):
        hip = hip.value
        thigh = thigh.value
        knee = knee.value
        calf = calf.value
        fl.set('%03.1f' % calf)
        is_valid = bool(is_valid)
        lf("angles", hip, thigh, knee, calf, is_valid)
        # log angles
        leg_display.draw_leg_angles([hip, thigh, knee, calf, is_valid])

    def on_xyz_values(x, y, z):
        x = x.value
        y = y.value
        z = z.value
        lf("xyz", x, y, z)
        fx.set('%03.2f' % x)
        fy.set('%03.2f' % y)
        fz.set('%03.2f' % z)
        # update foot
        r, new_state = foot.update((x, y, z), time.time())
        lf('restriction', r)
        fr.set('%01.3f' % r)
        if not foot.restriction_control:
            return
        if r > foot.max_restriction:  # TODO hold instead of stop
            print("restriction too high, stopping")
            stop()
        if foot.state == 'stance' and r > Foot.restriction_threshold:
            new_state = 'lift'
        if new_state is not None:
            # transition to new state
            # [mode[vel], frame[leg], x, y, z, ax, ay, az, spd]
            p = [1, 2, 0., 0., 0., 0., 0., 0., 1.]
            if new_state == 'swing':
                ## set target
                foot.target = (foot.center[0], foot.center[1] + foot.step_size)
                # send swing command
                #dx = foot.target[0] - x
                #dy = foot.target[1] - y
                #l = (dx * dx + dy * dy) ** 0.5
                #p[2] = dx / l * foot.swing_velocity
                #p[3] = dy / l * foot.swing_velocity
                p[0] = 3
                p[2] = foot.target[0]
                p[3] = foot.target[1]
                p[4] = foot.lift_height
                p[-1] = foot.swing_velocity
            elif new_state == 'stance':
                p[3] = -foot.stance_velocity
            elif new_state == 'lift':
                p[3] = -foot.stance_velocity
                p[4] = foot.lift_velocity
            elif new_state == 'lower':
                p[3] = -foot.stance_velocity
                p[4] = -foot.lower_velocity
            ns.plan(*p)
            lf('plan', *p)
            print("new state: %s[%s]" % (new_state, p))
            foot.state = new_state

    def on_estop(estop):
        # TODO estop
        pass

    def on_heartbeat():
        root.last_heartbeat = time.time()

    def on_leg_number(number):
        number = number.value
        lf("leg_number", number)
        print("Leg number: %s" % number)
        # load calibration
        vs = calibrations.get(number, [])
        for v in vs:
            f, args = v
            print("Calibration: %s, %s" % (f, args))
            getattr(ns, f)(*args)
        # set speed?
        root.wm_title("%s" % LEG_NAMES.get(number, 'INVALID'))

    mgr.on('heartbeat', on_heartbeat)
    mgr.on('adc', on_adc)
    mgr.on('loop_time', on_loop_time)
    mgr.on('pid', on_pid)
    mgr.on('angles', on_angles)
    mgr.on('xyz_values', on_xyz_values)
    mgr.on('leg_number', on_leg_number)
    print("setting leg number")
    #ns.leg_number(7)
    #ns.leg_number(2)
    #ns.leg_number(6)  # front right
    print("leg number set")
    ns.leg_number()  # request leg number

    def send_heartbeat():
        ns.heartbeat()
        root.after(500, send_heartbeat)

    root.after(500, send_heartbeat)

    def fake_move_leg(angles=None, da=None):
        if da is None:
            da = [0.05, 0.05, 0.05]
        if angles is None:
            angles = [0., 0., 0.]
        angles = [a + d for (a, d) in zip(angles, da)]
        va = [hadc, tadc, kadc]
        for i in xrange(len(angles)):
            a = angles[i]
            if a > LIMITS[i][1] or a < LIMITS[i][0]:
                da[i] *= -1
            na = (a - LIMITS[i][0]) / (LIMITS[i][1] - LIMITS[i][0])
            va[i].var.set(na * 65535)
        leg_display.draw_leg(angles)
        root.after(30, fake_move_leg, angles, da)

    #root.after(30, fake_move_leg)
    def handle_stream():
        com.handle_stream()
        root.after(20, handle_stream)
    root.after(20, handle_stream)
    root.mainloop()

    if log_file is not None:
        log_file.close()
