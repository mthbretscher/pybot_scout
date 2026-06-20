# -*- coding: utf-8 -*-
"""
Helpers for collecting run feedback as structured JSONL files.
"""

import datetime
import json
import os
import threading


class FeedbackLogger(object):
    def __init__(self, script_name, output_dir="run_feedback"):
        self.script_name = script_name
        self.output_dir = output_dir
        self._lock = threading.Lock()

        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        ts = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        filename = "%s_%s.jsonl" % (ts, script_name)
        self.path = os.path.join(self.output_dir, filename)
        self._fp = open(self.path, "a")

        self.log("logger_started", script=script_name, file=self.path)

    def log(self, event, **fields):
        payload = {
            "ts_utc": datetime.datetime.utcnow().isoformat() + "Z",
            "event": event,
        }
        payload.update(fields)

        line = json.dumps(payload, sort_keys=True)
        with self._lock:
            self._fp.write(line + "\n")
            self._fp.flush()

    def close(self):
        with self._lock:
            if self._fp is not None:
                payload = {
                    "ts_utc": datetime.datetime.utcnow().isoformat() + "Z",
                    "event": "logger_stopped",
                }
                self._fp.write(json.dumps(payload, sort_keys=True) + "\n")
                self._fp.flush()
                self._fp.close()
                self._fp = None
