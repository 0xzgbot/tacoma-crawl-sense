#!/usr/bin/env python3
"""Crawl Sense — body-clearance display for a 2000 Tacoma.

Four Benewake-protocol UART lidars at the bumper corners, aimed out at
trees and rocks that would meet the fenders, doors, and bedside. An
optional MPU6050 on the cab shows pitch and roll.

  q      quit
  1-4    mute FL, FR, RL, RR
  esc    quit

Bench window (instead of fullscreen):
  CRAWL_WINDOW=1 python3 crawl_sense.py
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import pygame
import serial

HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "config.json"
CORNERS = ("FL", "FR", "RL", "RR")
CM_TO_IN = 1 / 2.54

# Inches from the lens to the nearest return. Mount the lens at the
# widest paint and these are clearance to the panel.
CLOSE_IN = 36.0
HIT_IN = 18.0
MIN_STRENGTH = 100

INK = (243, 236, 223)
MUTED = (168, 152, 128)
GROUND = (20, 17, 14)
PANEL = (32, 28, 23)
AMBER = (231, 162, 58)
COLORS = {
    "none": (110, 102, 90),
    "clear": (125, 186, 106),
    "close": (231, 162, 58),
    "hit": (225, 90, 58),
}
LABEL = {
    "none": "NO LINK",
    "clear": "CLEAR",
    "close": "CLOSE",
    "hit": "HIT",
}


def load_json(path: Path, default):
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return default


def signed16(hi: int, lo: int) -> int:
    value = (hi << 8) | lo
    return value - 65536 if value >= 32768 else value


def classify(inches):
    """Distance from the corner lens to a tree, rock, or other body strike."""
    if inches is None:
        return "none"
    if inches <= HIT_IN:
        return "hit"
    if inches <= CLOSE_IN:
        return "close"
    return "clear"


class Lidar:
    def __init__(self, name: str, port: str, baud: int):
        self.name = name
        self.ok = False
        self.ser = None
        self.buf = bytearray()
        self.inches = None
        self.strength = 0
        self.age = 999.0
        self.last = time.monotonic()
        if not port:
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
            self.ok = True
        except serial.SerialException as exc:
            print(f"{name}: {exc}")

    def poll(self):
        if self.ser is None:
            self.age = 999.0
            return
        try:
            chunk = self.ser.read(256)
        except serial.SerialException as exc:
            print(f"{self.name}: {exc}")
            self.ok = False
            self.ser = None
            return
        now = time.monotonic()
        self.age = now - self.last
        if not chunk:
            return
        self.buf.extend(chunk)
        if len(self.buf) > 512:
            del self.buf[:-64]
        while True:
            start = self.buf.find(b"\x59\x59")
            if start < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                return
            if start:
                del self.buf[:start]
            if len(self.buf) < 9:
                return
            frame = bytes(self.buf[:9])
            del self.buf[:9]
            if (sum(frame[:8]) & 0xFF) != frame[8]:
                continue
            dist_cm = frame[2] | (frame[3] << 8)
            strength = frame[4] | (frame[5] << 8)
            if dist_cm <= 0 or strength < MIN_STRENGTH:
                continue
            self.inches = dist_cm * CM_TO_IN
            self.strength = strength
            self.last = now
            self.age = 0.0
            self.ok = True

    def close(self):
        if self.ser is not None:
            self.ser.close()


class Attitude:
    """Cab pitch and roll from an MPU6050 on I2C bus 1. Optional."""

    def __init__(self):
        self.ok = False
        self.pitch = 0.0
        self.roll = 0.0
        self.bus = None
        try:
            import smbus2

            bus = smbus2.SMBus(1)
            bus.write_byte_data(0x68, 0x6B, 0)
            self.bus = bus
            self.ok = True
        except Exception as exc:  # missing chip, missing i2c, missing library
            print(f"IMU off: {exc}")

    def poll(self):
        if self.bus is None:
            return
        try:
            raw = self.bus.read_i2c_block_data(0x68, 0x3B, 6)
        except Exception as exc:
            print(f"IMU read: {exc}")
            self.ok = False
            self.bus = None
            return
        ax = signed16(raw[0], raw[1])
        ay = signed16(raw[2], raw[3])
        az = signed16(raw[4], raw[5])
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az) or 1.0))
        roll = math.degrees(math.atan2(ay, az or 1.0))
        self.pitch = self.pitch * 0.85 + pitch * 0.15
        self.roll = self.roll * 0.85 + roll * 0.15


def draw_horizon(surface, font, attitude: Attitude):
    rect = pygame.Rect(0, 0, surface.get_width(), surface.get_height())
    pygame.draw.rect(surface, PANEL, rect, border_radius=16)
    cx, cy = rect.centerx, rect.centery + 10
    sky = pygame.Rect(rect.x + 16, rect.y + 48, rect.w - 32, rect.h - 78)
    pygame.draw.rect(surface, (28, 36, 44), sky, border_radius=10)
    roll = math.radians(attitude.roll if attitude.ok else 0.0)
    pitch_px = (attitude.pitch if attitude.ok else 0.0) * 3.2
    horizon_y = sky.centery + pitch_px
    ground_poly = []
    for x in (sky.left, sky.right):
        y = horizon_y + math.tan(roll) * (x - sky.centerx)
        ground_poly.append((x, y))
    ground_poly.extend([(sky.right, sky.bottom), (sky.left, sky.bottom)])
    prev = surface.get_clip()
    surface.set_clip(sky)
    pygame.draw.polygon(surface, (92, 64, 36), ground_poly)
    x1 = sky.left
    x2 = sky.right
    y1 = horizon_y + math.tan(roll) * (x1 - sky.centerx)
    y2 = horizon_y + math.tan(roll) * (x2 - sky.centerx)
    pygame.draw.line(surface, AMBER, (x1, y1), (x2, y2), 3)
    pygame.draw.circle(surface, INK, (cx, sky.centery), 4)
    surface.set_clip(prev)
    title = font.render("CAB", True, MUTED)
    surface.blit(title, (rect.x + 18, rect.y + 14))
    if attitude.ok:
        line = font.render(
            f"P {attitude.pitch:+.0f}°   R {attitude.roll:+.0f}°",
            True,
            INK,
        )
    else:
        line = font.render("IMU NOT FITTED", True, MUTED)
    surface.blit(line, (rect.x + 18, rect.bottom - 36))


def draw_truck(surface, font, readings, states):
    rect = pygame.Rect(0, 0, surface.get_width(), surface.get_height())
    pygame.draw.rect(surface, PANEL, rect, border_radius=16)
    cx, cy = rect.center
    body = pygame.Rect(0, 0, 120, 230)
    body.center = (cx, cy + 6)
    pygame.draw.rect(surface, (54, 48, 40), body, border_radius=16)
    cab = pygame.Rect(body.x + 14, body.y + 78, body.w - 28, 70)
    pygame.draw.rect(surface, (70, 62, 52), cab, border_radius=8)
    bed = pygame.Rect(body.x + 18, body.y + 156, body.w - 36, 58)
    pygame.draw.rect(surface, (44, 40, 34), bed, border_radius=4)
    # Bumpers
    pygame.draw.rect(surface, (90, 78, 60), (body.x - 8, body.y - 16, body.w + 16, 14), border_radius=3)
    pygame.draw.rect(surface, (90, 78, 60), (body.x - 8, body.bottom + 2, body.w + 16, 14), border_radius=3)

    spots = {
        "FL": (body.x + 18, body.y - 6),
        "FR": (body.right - 18, body.y - 6),
        "RL": (body.x + 18, body.bottom + 6),
        "RR": (body.right - 18, body.bottom + 6),
    }
    beam = {
        "FL": (-48, -48),
        "FR": (48, -48),
        "RL": (-48, 48),
        "RR": (48, 48),
    }
    for name, (x, y) in spots.items():
        color = COLORS[states[name]]
        dx, dy = beam[name]
        pygame.draw.line(surface, color, (x, y), (x + dx, y + dy), 10)
        pygame.draw.circle(surface, color, (x, y), 9)
        pygame.draw.circle(surface, GROUND, (x, y), 4)
        tag = font.render(name, True, INK)
        surface.blit(tag, (x - tag.get_width() / 2, y + (16 if dy > 0 else -28)))
    nose = font.render("NOSE", True, MUTED)
    surface.blit(nose, (cx - nose.get_width() / 2, rect.y + 14))


def draw_readout(surface, font, big, name, reading, state, muted_corner):
    pygame.draw.rect(surface, PANEL, surface.get_rect(), border_radius=16)
    color = MUTED if muted_corner else COLORS[state]
    head = font.render(name, True, MUTED)
    surface.blit(head, (16, 12))
    word = font.render("MUTE" if muted_corner else LABEL[state], True, color)
    surface.blit(word, (surface.get_width() - word.get_width() - 16, 12))
    if reading is None or reading.inches is None or reading.age > 0.6:
        value = big.render("--", True, MUTED)
        surface.blit(value, (16, 40))
        return
    text = big.render(f"{reading.inches:4.0f}\"", True, INK)
    surface.blit(text, (16, 36))
    sub = font.render("to the nearest panel strike", True, color)
    surface.blit(sub, (16, surface.get_height() - 32))


def draw_spark(surface, history, state):
    pygame.draw.rect(surface, PANEL, surface.get_rect(), border_radius=12)
    if len(history) < 2:
        return
    vals = [v for v in history if v is not None]
    if not vals:
        return
    lo, hi = min(vals + [0]), max(vals + [48])
    span = hi - lo or 1
    w, h = surface.get_size()
    pts = []
    for i, value in enumerate(history):
        if value is None:
            continue
        x = 10 + (w - 20) * (i / max(1, len(history) - 1))
        y = h - 10 - (h - 20) * ((value - lo) / span)
        pts.append((x, y))
    if len(pts) > 1:
        pygame.draw.lines(surface, COLORS[state], False, pts, 2)


def main():
    config = load_json(
        CONFIG_PATH,
        {"baud": 115200, "ports": {name: "" for name in CORNERS}},
    )
    baud = int(config.get("baud", 115200))
    ports = config.get("ports", {})
    insets = config.get("inset_in", {})
    lidars = [Lidar(name, ports.get(name, ""), baud) for name in CORNERS]
    attitude = Attitude()
    muted = {name: False for name in CORNERS}
    history = {name: [] for name in CORNERS}

    pygame.init()
    pygame.display.set_caption("Crawl Sense")
    flags = 0 if os.environ.get("CRAWL_WINDOW") else pygame.FULLSCREEN
    screen = pygame.display.set_mode((1024, 600), flags)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("sans", 22)
    big = pygame.font.SysFont("sans", 42, bold=True)
    small = pygame.font.SysFont("sans", 16)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.unicode in "1234":
                    name = CORNERS[int(event.unicode) - 1]
                    muted[name] = not muted[name]

        for lidar in lidars:
            lidar.poll()
        attitude.poll()

        readings = {lidar.name: lidar for lidar in lidars}
        states = {}
        for name in CORNERS:
            lidar = readings[name]
            fresh = lidar.inches is not None and lidar.age < 0.6
            inset = float(insets.get(name, 0) or 0)
            inches = None if not fresh else max(0.0, lidar.inches - inset)
            if fresh:
                lidar.inches = inches
            states[name] = "none" if muted[name] or not fresh else classify(inches)
            history[name].append(inches)
            del history[name][:-120]

        screen.fill(GROUND)
        horizon_box = pygame.Rect(24, 24, 280, 300)
        truck_box = pygame.Rect(320, 24, 380, 420)
        draw_horizon(screen.subsurface(horizon_box), font, attitude)
        draw_truck(screen.subsurface(truck_box), small, readings, states)

        card_w, card_h = 280, 108
        for i, name in enumerate(CORNERS):
            card = pygame.Rect(720, 18 + i * (card_h + 8), card_w, card_h)
            draw_readout(
                screen.subsurface(card),
                font,
                big,
                name,
                readings[name],
                states[name],
                muted[name],
            )

        hint = small.render(
            "1-4 mute    under 36 in CLOSE    under 18 in HIT    Q quit",
            True,
            MUTED,
        )
        screen.blit(hint, (24, 560))
        for i, name in enumerate(CORNERS):
            spark = pygame.Rect(24 + i * 168, 470, 156, 78)
            draw_spark(screen.subsurface(spark), history[name], states[name])
            tag = small.render(name, True, MUTED)
            screen.blit(tag, (spark.x + 8, spark.y + 6))

        pygame.display.flip()
        clock.tick(30)

    for lidar in lidars:
        lidar.close()
    pygame.quit()


if __name__ == "__main__":
    main()
