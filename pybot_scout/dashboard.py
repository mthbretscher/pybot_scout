# -*- coding: utf-8 -*-
"""Optional curses dashboard for live robot script telemetry."""

import math
import os
import time

try:
    import curses
except Exception:
    curses = None

try:
    from collections import OrderedDict
except Exception:
    OrderedDict = dict


def _env_truthy(name, default="0"):
    raw = os.environ.get(name, default)
    if raw is None:
        return False
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def _env_float(name, default):
    raw = os.environ.get(name, "")
    if not raw:
        return float(default)
    try:
        value = float(raw)
    except Exception:
        return float(default)
    return value


def _safe_float(value):
    try:
        val = float(value)
    except Exception:
        return None
    if math.isinf(val) or math.isnan(val):
        return None
    return val


def _as_text(value):
    if value is None:
        return "-"
    if isinstance(value, float):
        if math.isinf(value):
            return "inf"
        return "%.3f" % value
    return str(value)


class ScriptDashboard(object):
    """Shared helper for script-owned state + sensor telemetry rendering."""

    SPARK_CHARS = " .:-=+*#%@"

    def __init__(self, script_name, logger=None):
        self.script_name = script_name
        self.logger = logger
        self.enabled = _env_truthy("PYBOT_SCOUT_TUI", "0")
        self.history_secs = max(5.0, _env_float("PYBOT_SCOUT_TUI_HISTORY_SECS", 45.0))
        self.refresh_secs = max(0.1, _env_float("PYBOT_SCOUT_TUI_REFRESH_SECS", 0.5))

        self._screen = None
        self._last_draw_ts = 0.0
        self._started = False
        self._state = OrderedDict()
        self._sensors = OrderedDict()
        self._series = {}

    def start(self):
        if self._started:
            return
        self._started = True
        if not self.enabled:
            return
        if curses is None:
            self.enabled = False
            self._log("dashboard_disabled", reason="curses_import_failed")
            return
        try:
            if not os.isatty(0):
                self.enabled = False
                self._log("dashboard_disabled", reason="stdin_not_tty")
                return
        except Exception:
            pass
        try:
            self._screen = curses.initscr()
            curses.noecho()
            curses.cbreak()
            self._screen.nodelay(1)
            self._screen.keypad(0)
            self._screen.erase()
            self._screen.refresh()
            self._log("dashboard_started",
                      history_secs=self.history_secs,
                      refresh_secs=self.refresh_secs)
        except Exception as exc:
            self.enabled = False
            self._screen = None
            self._log("dashboard_disabled", reason="curses_init_failed", error=str(exc))
            try:
                curses.endwin()
            except Exception:
                pass

    def close(self):
        if self._screen is not None:
            try:
                self._screen.erase()
                self._screen.refresh()
            except Exception:
                pass
            try:
                curses.nocbreak()
                curses.echo()
                curses.endwin()
            except Exception:
                pass
            self._screen = None
            self._log("dashboard_stopped")

    def update_sensors(self, sensor_map):
        if not sensor_map:
            return
        now = time.time()
        for key in sorted(sensor_map.keys()):
            val = sensor_map.get(key)
            self._sensors[key] = val
            self._append_numeric(key, val, now)

    def update_state(self, **kwargs):
        self.update_state_map(kwargs)

    def update_state_map(self, state_map):
        if not state_map:
            return
        now = time.time()
        for key in sorted(state_map.keys()):
            val = state_map.get(key)
            self._state[key] = val
            self._append_numeric("state:" + key, val, now)

    def tick(self, force=False):
        if not self.enabled or self._screen is None:
            return
        now = time.time()
        if not force and (now - self._last_draw_ts) < self.refresh_secs:
            return
        self._last_draw_ts = now
        self._trim_history(now)
        self._render(now)

    def _append_numeric(self, key, value, ts):
        num = _safe_float(value)
        if num is None:
            return
        points = self._series.get(key)
        if points is None:
            points = []
            self._series[key] = points
        points.append((ts, num))

    def _trim_history(self, now):
        cutoff = now - self.history_secs
        for key in list(self._series.keys()):
            pts = self._series.get(key, [])
            while pts and pts[0][0] < cutoff:
                pts.pop(0)
            if not pts:
                del self._series[key]

    def _render(self, now):
        try:
            rows, cols = self._screen.getmaxyx()
            self._screen.erase()
            title = "%s | TUI ON | history=%ss" % (self.script_name, int(self.history_secs))
            self._addstr(0, 0, title[:max(0, cols - 1)])
            if rows < 4 or cols < 30:
                self._addstr(1, 0, "Terminal too small")
                self._screen.refresh()
                return

            left_w = max(28, int(cols * 0.52))
            if left_w >= cols - 2:
                left_w = cols - 2
            right_x = left_w + 1
            right_w = max(1, cols - right_x - 1)

            self._addstr(1, 0, "Values")
            self._addstr(1, right_x, "History")

            line = 2
            keys = []
            for k in sorted(self._sensors.keys()):
                keys.append(("sensor", k))
            for k in sorted(self._state.keys()):
                keys.append(("state", k))

            max_lines = rows - 2
            for kind, key in keys[:max_lines]:
                if kind == "sensor":
                    val = self._sensors.get(key)
                    history_key = key
                    label = "S " + key
                else:
                    val = self._state.get(key)
                    history_key = "state:" + key
                    label = "V " + key
                left = "%-24s %s" % (label[:24], _as_text(val))
                self._addstr(line, 0, left[:max(0, left_w - 1)])
                spark = self._sparkline(self._series.get(history_key, []), right_w)
                self._addstr(line, right_x, spark[:right_w])
                line += 1
                if line >= rows:
                    break
            self._screen.refresh()
        except Exception as exc:
            self._log("dashboard_render_failed", error=str(exc))
            self.close()
            self.enabled = False

    def _sparkline(self, points, width):
        if width <= 0:
            return ""
        if not points:
            return " " * width
        vals = [v for _, v in points]
        if not vals:
            return " " * width
        if len(vals) > width:
            vals = vals[-width:]
        if len(vals) < width:
            vals = ([vals[0]] * (width - len(vals))) + vals
        vmin = min(vals)
        vmax = max(vals)
        if vmax <= vmin:
            idx = len(self.SPARK_CHARS) // 2
            return self.SPARK_CHARS[idx] * width
        scale = float(len(self.SPARK_CHARS) - 1) / float(vmax - vmin)
        out = []
        for val in vals:
            pos = int((val - vmin) * scale)
            if pos < 0:
                pos = 0
            elif pos >= len(self.SPARK_CHARS):
                pos = len(self.SPARK_CHARS) - 1
            out.append(self.SPARK_CHARS[pos])
        return "".join(out)

    def _addstr(self, y, x, text):
        try:
            self._screen.addstr(y, x, text)
        except Exception:
            pass

    def _log(self, event, **fields):
        if self.logger is not None:
            self.logger.log(event, **fields)
