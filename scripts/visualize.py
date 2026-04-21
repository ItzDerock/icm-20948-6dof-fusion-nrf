#!/usr/bin/env python3
"""
Real-time IMU visualizer for gimbal-dmp.
Reads COBS/postcard telemetry from UART and shows:
  - 3D cube oriented by the IMU quaternion (top)
  - Roll / Pitch / Yaw time-series graphs (bottom)

Left-drag in the cube viewport to orbit the camera.

Usage:
  python scripts/visualize.py [--port /dev/ttyACM0] [--baud 115200]
  python scripts/visualize.py --demo        # simulated data, no hardware needed
"""

import argparse
import math
import struct
import sys
import threading
from collections import deque

import pygame
import serial
from OpenGL.GL import (
    GL_BLEND,
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_LINE_LOOP,
    GL_LINE_STRIP,
    GL_LINEAR,
    GL_LINES,
    GL_MODELVIEW,
    GL_ONE_MINUS_SRC_ALPHA,
    GL_PROJECTION,
    GL_QUADS,
    GL_RGBA,
    GL_SCISSOR_TEST,
    GL_SRC_ALPHA,
    GL_TEXTURE_2D,
    GL_TEXTURE_MAG_FILTER,
    GL_TEXTURE_MIN_FILTER,
    GL_UNSIGNED_BYTE,
    GLfloat,
    glBegin,
    glBindTexture,
    glBlendFunc,
    glClear,
    glClearColor,
    glColor3f,
    glColor4f,
    glDeleteTextures,
    glDisable,
    glEnable,
    glEnd,
    glGenTextures,
    glLineWidth,
    glLoadIdentity,
    glMatrixMode,
    glMultMatrixf,
    glOrtho,
    glPopMatrix,
    glPushMatrix,
    glRotatef,
    glScissor,
    glTexCoord2f,
    glTexImage2D,
    glTexParameteri,
    glTranslatef,
    glVertex2f,
    glVertex3fv,
    glViewport,
)
from OpenGL.GLU import gluPerspective
from pygame.locals import (
    DOUBLEBUF,
    K_ESCAPE,
    KEYDOWN,
    MOUSEBUTTONDOWN,
    MOUSEBUTTONUP,
    MOUSEMOTION,
    OPENGL,
    QUIT,
)

# ── Window layout ─────────────────────────────────────────────────────────────

WIN_W = 1280
WIN_H = 800
GRAPH_H = 230  # height of the graph strip at the bottom
CUBE_H = WIN_H - GRAPH_H
HISTORY = 400  # number of samples kept in rolling graph

# ── CRC-16/CCITT-FALSE ────────────────────────────────────────────────────────


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ── COBS decode ───────────────────────────────────────────────────────────────


def cobs_decode(data: bytes):
    """Decode a COBS frame (without the trailing 0x00 delimiter)."""
    out, i = bytearray(), 0
    try:
        while i < len(data):
            code = data[i]
            i += 1
            for _ in range(1, code):
                out.append(data[i])
                i += 1
            if code < 0xFF:
                out.append(0)
    except IndexError:
        return None
    if out and out[-1] == 0:
        out.pop()
    return bytes(out)


# ── Packet parsing ────────────────────────────────────────────────────────────


def parse_packet(raw: bytes):
    """
    Packet layout (30 bytes):
      accel: [f32; 3]  →  12 bytes LE
      quat:  [f32; 4]  →  16 bytes LE  (w, i, j, k)
      crc:   u16       →   2 bytes LE  CRC-16/CCITT-FALSE over first 28 bytes
    Returns ((ax,ay,az), (qw,qx,qy,qz)) or None.
    """
    if len(raw) < 30:
        return None
    expected_crc = crc16(raw[:28])
    actual_crc = struct.unpack_from("<H", raw, 28)[0]
    if expected_crc != actual_crc:
        return None
    v = struct.unpack_from("<7f", raw)
    return v[:3], v[3:]


# ── Math helpers ──────────────────────────────────────────────────────────────


def quat_to_euler(w, x, y, z):
    """Returns (roll, pitch, yaw) in degrees."""
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def quat_to_gl_matrix(w, x, y, z):
    """4×4 column-major rotation matrix suitable for glMultMatrixf."""
    return (GLfloat * 16)(
        1 - 2 * (y * y + z * z),
        2 * (x * y + w * z),
        2 * (x * z - w * y),
        0,
        2 * (x * y - w * z),
        1 - 2 * (x * x + z * z),
        2 * (y * z + w * x),
        0,
        2 * (x * z + w * y),
        2 * (y * z - w * x),
        1 - 2 * (x * x + y * y),
        0,
        0,
        0,
        0,
        1,
    )


# ── Serial reader (background thread) ────────────────────────────────────────


class SerialReader:
    def __init__(self, port: str, baud: int):
        self._port = port
        self._baud = baud
        self._latest = None
        self._lock = threading.Lock()
        self._buf = bytearray()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.05)
            print(f"[serial] opened {self._port} @ {self._baud} baud")
        except serial.SerialException as e:
            print(f"[serial] {e}", file=sys.stderr)
            return
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            self._buf.extend(chunk)
            while 0 in self._buf:
                idx = self._buf.index(0)
                frame, self._buf = bytes(self._buf[:idx]), self._buf[idx + 1 :]
                if not frame:
                    continue
                decoded = cobs_decode(frame)
                if decoded is None:
                    continue
                pkt = parse_packet(decoded)
                if pkt is None:
                    continue
                with self._lock:
                    self._latest = pkt

    def get(self):
        with self._lock:
            return self._latest


# ── Cube geometry ─────────────────────────────────────────────────────────────

_VERTS = [
    (-1, -1, -1),
    (+1, -1, -1),
    (+1, +1, -1),
    (-1, +1, -1),  # back (z-)
    (-1, -1, +1),
    (+1, -1, +1),
    (+1, +1, +1),
    (-1, +1, +1),  # front (z+)
]
_FACES = [
    ((0, 1, 2, 3), (0.82, 0.22, 0.22)),  # -z  red
    ((4, 5, 6, 7), (0.22, 0.74, 0.32)),  # +z  green
    ((0, 4, 7, 3), (0.22, 0.34, 0.84)),  # -x  blue
    ((1, 5, 6, 2), (0.90, 0.74, 0.10)),  # +x  yellow
    ((0, 1, 5, 4), (0.72, 0.22, 0.72)),  # -y  purple
    ((3, 2, 6, 7), (0.18, 0.82, 0.82)),  # +y  cyan
]
_EDGES = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
]


def draw_cube(quat):
    glEnable(GL_DEPTH_TEST)
    glPushMatrix()
    glMultMatrixf(quat_to_gl_matrix(*quat))

    for idx, col in _FACES:
        glColor3f(*col)
        glBegin(GL_QUADS)
        for i in idx:
            glVertex3fv(_VERTS[i])
        glEnd()

    glColor3f(0.05, 0.05, 0.05)
    glLineWidth(2)
    glBegin(GL_LINES)
    for a, b in _EDGES:
        glVertex3fv(_VERTS[a])
        glVertex3fv(_VERTS[b])
    glEnd()

    # axis arrows  X=red  Y=green  Z=blue
    glLineWidth(3)
    for tip, col in [
        ((1.9, 0, 0), (1, 0.3, 0.3)),
        ((0, 1.9, 0), (0.3, 1, 0.3)),
        ((0, 0, 1.9), (0.3, 0.3, 1)),
    ]:
        glColor3f(*col)
        glBegin(GL_LINES)
        glVertex3fv((0, 0, 0))
        glVertex3fv(tip)
        glEnd()
    glLineWidth(1)
    glPopMatrix()


# ── Rolling graph ─────────────────────────────────────────────────────────────


class Graph:
    def __init__(self, x, y, w, h, label, color, ymin=-180, ymax=180):
        self.x, self.y, self.w, self.h = x, y, w, h
        self.label = label
        self.color = color
        self.ymin = ymin
        self.ymax = ymax
        self.data = deque(maxlen=HISTORY)

    def push(self, v):
        self.data.append(v)

    def _vy(self, v):
        return self.y + self.h * (v - self.ymin) / (self.ymax - self.ymin)

    def draw(self):
        x, y, w, h = self.x, self.y, self.w, self.h

        # background
        glColor3f(0.04, 0.04, 0.07)
        glBegin(GL_QUADS)
        glVertex2f(x, y)
        glVertex2f(x + w, y)
        glVertex2f(x + w, y + h)
        glVertex2f(x, y + h)
        glEnd()

        # grid lines at ymin, 0, midpoints, ymax
        for gv in (self.ymin, self.ymin / 2, 0, self.ymax / 2, self.ymax):
            gy = self._vy(gv)
            bright = 0.25 if gv == 0 else 0.13
            glColor3f(bright, bright, bright)
            glBegin(GL_LINES)
            glVertex2f(x, gy)
            glVertex2f(x + w, gy)
            glEnd()

        # border
        glColor3f(0.30, 0.30, 0.36)
        glBegin(GL_LINE_LOOP)
        glVertex2f(x, y)
        glVertex2f(x + w, y)
        glVertex2f(x + w, y + h)
        glVertex2f(x, y + h)
        glEnd()

        if len(self.data) < 2:
            return

        r, g, b = self.color
        glColor3f(r, g, b)
        glLineWidth(1.5)
        glBegin(GL_LINE_STRIP)
        n = len(self.data)
        for i, v in enumerate(self.data):
            px = x + w * i / (HISTORY - 1)
            glVertex2f(px, self._vy(v))
        glEnd()
        glLineWidth(1)


# ── Text (pygame surface → ephemeral GL texture) ──────────────────────────────

_font_cache: dict = {}


def _get_font(size: int):
    if size not in _font_cache:
        _font_cache[size] = pygame.font.SysFont("monospace", size)
    return _font_cache[size]


def gl_draw_text(text: str, x: float, y: float, color=(220, 220, 220), size: int = 14):
    surf = _get_font(size).render(text, True, color)
    tw, th = surf.get_size()
    raw = pygame.image.tostring(surf, "RGBA", True)
    tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw)
    glEnable(GL_TEXTURE_2D)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glColor4f(1, 1, 1, 1)
    glBegin(GL_QUADS)
    glTexCoord2f(0, 0)
    glVertex2f(x, y)
    glTexCoord2f(1, 0)
    glVertex2f(x + tw, y)
    glTexCoord2f(1, 1)
    glVertex2f(x + tw, y + th)
    glTexCoord2f(0, 1)
    glVertex2f(x, y + th)
    glEnd()
    glDisable(GL_TEXTURE_2D)
    glDisable(GL_BLEND)
    glDeleteTextures([tex])


# ── Main ──────────────────────────────────────────────────────────────────────


def main():
    ap = argparse.ArgumentParser(description="gimbal-dmp IMU visualizer")
    ap.add_argument("--port", default="/dev/ttyACM0", help="serial port")
    ap.add_argument("--baud", type=int, default=115200, help="baud rate")
    ap.add_argument("--demo", action="store_true", help="simulated data (no hardware)")
    args = ap.parse_args()

    reader = None
    if not args.demo:
        reader = SerialReader(args.port, args.baud)
        reader.start()
    else:
        print("[demo] running with simulated data")

    pygame.init()
    pygame.display.set_mode((WIN_W, WIN_H), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("gimbal-dmp — IMU Visualizer")
    clock = pygame.time.Clock()

    pad = 12
    gw = (WIN_W - 4 * pad) // 3
    gh = GRAPH_H - 2 * pad
    graphs = [
        Graph(pad + 0 * (gw + pad), pad, gw, gh, "Roll", (1.00, 0.35, 0.35)),
        Graph(pad + 1 * (gw + pad), pad, gw, gh, "Pitch", (0.35, 1.00, 0.35)),
        Graph(pad + 2 * (gw + pad), pad, gw, gh, "Yaw", (0.35, 0.72, 1.00)),
    ]
    graph_colors_rgb = [(220, 90, 90), (90, 220, 90), (90, 185, 255)]

    quat = (1.0, 0.0, 0.0, 0.0)
    t_sim = 0.0

    # Camera orbit state
    cam_yaw = 30.0  # degrees, rotated around Y
    cam_pitch = 20.0  # degrees, rotated around X
    dragging = False
    last_mouse = (0, 0)

    while True:
        dt = clock.tick(60) / 1000.0

        for ev in pygame.event.get():
            if ev.type == QUIT:
                pygame.quit()
                sys.exit()
            if ev.type == KEYDOWN and ev.key == K_ESCAPE:
                pygame.quit()
                sys.exit()

            # Camera orbit: only when cursor is in the cube viewport (y < CUBE_H in window coords)
            if ev.type == MOUSEBUTTONDOWN and ev.button == 1:
                mx, my = ev.pos
                if my < CUBE_H:
                    dragging = True
                    last_mouse = ev.pos
            elif ev.type == MOUSEBUTTONUP and ev.button == 1:
                dragging = False
            elif ev.type == MOUSEMOTION and dragging:
                mx, my = ev.pos
                dx = mx - last_mouse[0]
                dy = my - last_mouse[1]
                cam_yaw += dx * 0.4
                cam_pitch += dy * 0.4
                cam_pitch = max(-180.0, min(180.0, cam_pitch))
                last_mouse = ev.pos

        # ── Update state ──────────────────────────────────────────────────────
        if args.demo:
            t_sim += dt
            roll_d = math.sin(t_sim * 0.55) * 45
            pitch_d = math.sin(t_sim * 0.38) * 30
            yaw_d = (t_sim * 22) % 360 - 180
            cr, sr = (
                math.cos(math.radians(roll_d / 2)),
                math.sin(math.radians(roll_d / 2)),
            )
            cp, sp = (
                math.cos(math.radians(pitch_d / 2)),
                math.sin(math.radians(pitch_d / 2)),
            )
            cy, sy = (
                math.cos(math.radians(yaw_d / 2)),
                math.sin(math.radians(yaw_d / 2)),
            )
            quat = (
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            )
        elif reader:
            pkt = reader.get()
            if pkt:
                _, quat = pkt

        roll, pitch, yaw = quat_to_euler(*quat)
        for g, v in zip(graphs, (roll, pitch, yaw)):
            g.push(v)

        # ── 3D cube viewport (top) ────────────────────────────────────────────
        glViewport(0, GRAPH_H, WIN_W, CUBE_H)
        glScissor(0, GRAPH_H, WIN_W, CUBE_H)
        glEnable(GL_SCISSOR_TEST)
        glClearColor(0.05, 0.05, 0.10, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glDisable(GL_SCISSOR_TEST)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, WIN_W / CUBE_H, 0.1, 50.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0, 0, -5)
        # Apply camera orbit before drawing
        glRotatef(cam_pitch, 1, 0, 0)
        glRotatef(cam_yaw, 0, 1, 0)
        draw_cube(quat)

        # Overlay: current angle readout in cube viewport (top-left, 2D ortho)
        glDisable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, WIN_W, 0, CUBE_H, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        status = f"Roll {roll:+7.1f}°   Pitch {pitch:+7.1f}°   Yaw {yaw:+7.1f}°   " + (
            "demo" if args.demo else f"{args.port}"
        )
        gl_draw_text(status, 10, CUBE_H - 22, (180, 180, 210), 14)
        gl_draw_text("drag to orbit", WIN_W - 115, CUBE_H - 22, (100, 100, 130), 13)

        # ── Graph viewport (bottom) ───────────────────────────────────────────
        glViewport(0, 0, WIN_W, GRAPH_H)
        glScissor(0, 0, WIN_W, GRAPH_H)
        glEnable(GL_SCISSOR_TEST)
        glClear(GL_DEPTH_BUFFER_BIT)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_SCISSOR_TEST)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, WIN_W, 0, GRAPH_H, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glColor3f(0.03, 0.03, 0.05)
        glBegin(GL_QUADS)
        glVertex2f(0, 0)
        glVertex2f(WIN_W, 0)
        glVertex2f(WIN_W, GRAPH_H)
        glVertex2f(0, GRAPH_H)
        glEnd()

        labels = ["Roll (°)", "Pitch (°)", "Yaw (°)"]
        vals = (roll, pitch, yaw)
        for g, lbl, val, col in zip(graphs, labels, vals, graph_colors_rgb):
            g.draw()
            gl_draw_text(lbl, g.x + 5, g.y + g.h - 18, col, 13)
            gl_draw_text(f"{val:+7.1f}", g.x + g.w - 62, g.y + g.h - 18, col, 13)
            gl_draw_text("-180", g.x + 3, g.y + 2, (120, 120, 130), 11)
            gl_draw_text("+180", g.x + 3, g.y + g.h - 30, (120, 120, 130), 11)
            gl_draw_text("0", g.x + 3, g.y + gh // 2 - 7, (120, 120, 130), 11)

        pygame.display.flip()


if __name__ == "__main__":
    main()
