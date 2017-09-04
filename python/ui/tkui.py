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
"""

import datetime
import math
import sys
import time
import Tkinter as tk

import serial

import pycomando

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
    3: 'adc=uint32,uint32,uint32',
    4: 'adc_target(uint32,uint32,uint32)',
    5: 'pwm_value=int32,int32,int32',
    6: 'pid=float,float,float,float,float,float,float,float,float',
    #7: 'plan(byte,byte,float,float,float,float,float,float,float,uint32)',
    7: 'plan(byte,byte,float,float,float,float,float,float,float)',
    8: 'enable_pid(bool)',
    9: 'xyz_values=float,float,float',
    10: 'angles=float,float,float',
    11: 'set_pid(byte,float,float,float,float,float)',
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
        self.top_lines = [None, None, None]
        self.side_lines = [None, None, None]
        self._inited = False

    def init(self):
        # TODO draw limits
        pass

    def draw_leg(self, angles):
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
        for i in xrange(len(pts) - 1):
            pp = pts[i]
            p = pts[i+1]
            if self.top_lines[i] is None:  # draw line
                # x0 y0 x1 y1
                self.top_lines[i] = self.top_canvas.create_line(
                    pp[0] + self.sx, pp[2] + self.sy,
                    p[0] + self.sx, p[2] + self.sy,
                    fill=COLORS[i])
            else:  # else move line
                self.top_canvas.coords(
                    self.top_lines[i],
                    pp[0] + self.sx, pp[2] + self.sy,
                    p[0] + self.sx, p[2] + self.sy)
            if self.side_lines[i] is None:  # draw line
                self.side_lines[i] = self.side_canvas.create_line(
                    pp[0] + self.sx, pp[1] + self.sy,
                    p[0] + self.sx, p[1] + self.sy,
                    fill=COLORS[i])
            else:  # else move line
                self.side_canvas.coords(
                    self.side_lines[i],
                    pp[0] + self.sx, pp[1] + self.sy,
                    p[0] + self.sx, p[1] + self.sy)


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
        tk.Label(self.frame, text="MIN:").pack(side=tk.LEFT)
        self.min_entry = tk.Entry(self.frame)
        self.min_entry.pack(side=tk.LEFT)
        tk.Label(self.frame, text="MAX:").pack(side=tk.LEFT)
        self.max_entry = tk.Entry(self.frame)
        self.max_entry.pack(side=tk.LEFT)
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
        minv = float(self.min_entry.get())
        maxv = float(self.max_entry.get())
        self.cb(self.label, p, i, d, minv, maxv)


if __name__ == '__main__':
    com = pycomando.Comando(serial.Serial('/dev/ttyACM0', 9600))
    cmd = pycomando.protocols.command.CommandProtocol()
    com.register_protocol(0, cmd)
    mgr = pycomando.protocols.command.EventManager(cmd, cmds)
    ns = mgr.build_namespace()
    ns.estop(2)  # set estop to disable leg

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

    def on_adc(hip, thigh, knee):
        h = hip.value
        t = thigh.value
        k = knee.value
        lf("adc", h, t, k)
        hadc.var.set(h)
        tadc.var.set(t)
        kadc.var.set(k)
        # TODO calf

    def on_pid(ho, to, ko, hs, ts, ks, he, te, ke):
        lf(
            "pid", ho.value, to.value, ko.value,
            hs.value, ts.value, ks.value,
            he.value, te.value, ke.value)

    def on_angles(hip, thigh, knee):
        hip = hip.value
        thigh = thigh.value
        knee = knee.value
        lf("angles", hip, thigh, knee)
        # log angles
        leg_display.draw_leg([hip, thigh, knee])

    def on_xyz_values(x, y, z):
        x = x.value
        y = y.value
        z = z.value
        lf("xyz", x, y, z)
        fx.set('%03.2f' % x)
        fy.set('%03.2f' % y)
        fz.set('%03.2f' % z)

    def on_estop(estop):
        # TODO estop
        pass

    def on_heartbeat():
        root.last_heartbeat = time.time()

    mgr.on('heartbeat', on_heartbeat)
    mgr.on('adc', on_adc)
    mgr.on('angles', on_angles)
    mgr.on('xyz_values', on_xyz_values)

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
