"""Threaded joint feedback collection with peak analysis and CSV export."""

from __future__ import annotations

import csv
import threading
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping

import numpy as np

from ..config import JOINT_KEYS
from ..errors import OK


@dataclass(frozen=True)
class TelemetrySample:
    elapsed_s: float
    timestamp_iso: str
    arm: str
    joint: str
    angle_deg: float | None
    speed_rpm: float | None
    current_a: float | None
    error_code: int = OK


@dataclass(frozen=True)
class PeakSummary:
    arm: str
    joint: str
    sample_count: int
    angle_min_deg: float | None
    angle_max_deg: float | None
    max_abs_speed_rpm: float | None
    max_abs_current_a: float | None


class TelemetryRecorder:
    """Poll one or two controllers without coupling recording to the GUI."""

    def __init__(
        self,
        controllers: Mapping[str, object],
        *,
        sample_period_s: float = 0.10,
        max_samples_per_joint: int = 10_000,
    ) -> None:
        if sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be positive")
        if max_samples_per_joint < 2:
            raise ValueError("max_samples_per_joint must be at least two")
        self.controllers = dict(controllers)
        self.sample_period_s = float(sample_period_s)
        self.max_samples_per_joint = int(max_samples_per_joint)
        self._samples: dict[tuple[str, str], deque[TelemetrySample]] = defaultdict(
            lambda: deque(maxlen=self.max_samples_per_joint)
        )
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._started_monotonic = time.monotonic()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._stop_event.clear()
        self._started_monotonic = time.monotonic()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="sanpo-telemetry",
        )
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=timeout_s)

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()
        self._started_monotonic = time.monotonic()

    def poll_once(self) -> list[TelemetrySample]:
        """Read every joint once; useful for tests and explicit snapshots."""

        elapsed = max(0.0, time.monotonic() - self._started_monotonic)
        timestamp = datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")
        batch: list[TelemetrySample] = []
        for arm_name, controller in self.controllers.items():
            if not getattr(controller, "connected", False):
                continue
            error, feedback = controller.refresh_all()
            for joint in JOINT_KEYS:
                data = feedback.get(joint) or {}
                sample = TelemetrySample(
                    elapsed_s=elapsed,
                    timestamp_iso=timestamp,
                    arm=arm_name,
                    joint=joint,
                    angle_deg=_optional_float(data.get("angle_deg")),
                    speed_rpm=_optional_float(data.get("speed_rpm")),
                    current_a=_optional_float(data.get("current_a")),
                    error_code=int(error),
                )
                batch.append(sample)
        with self._lock:
            for sample in batch:
                self._samples[(sample.arm, sample.joint)].append(sample)
        return batch

    def _run(self) -> None:
        deadline = time.monotonic()
        while not self._stop_event.is_set():
            self.poll_once()
            deadline += self.sample_period_s
            wait_s = max(0.0, deadline - time.monotonic())
            if self._stop_event.wait(wait_s):
                break
            if wait_s <= 0.0:
                deadline = time.monotonic()

    def snapshot(
        self,
        arm: str | None = None,
        joint: str | None = None,
    ) -> list[TelemetrySample]:
        with self._lock:
            result = [
                sample
                for (arm_name, joint_name), values in self._samples.items()
                if (arm is None or arm == arm_name)
                and (joint is None or joint == joint_name)
                for sample in values
            ]
        return sorted(result, key=lambda sample: (sample.elapsed_s, sample.arm, sample.joint))

    def latest(self) -> dict[tuple[str, str], TelemetrySample]:
        with self._lock:
            return {
                key: values[-1]
                for key, values in self._samples.items()
                if values
            }

    def peak_summaries(self) -> list[PeakSummary]:
        summaries: list[PeakSummary] = []
        with self._lock:
            items = [(key, list(values)) for key, values in self._samples.items()]
        for (arm, joint), samples in sorted(items):
            angles = _finite_values(sample.angle_deg for sample in samples)
            speeds = _finite_values(sample.speed_rpm for sample in samples)
            currents = _finite_values(sample.current_a for sample in samples)
            summaries.append(
                PeakSummary(
                    arm=arm,
                    joint=joint,
                    sample_count=len(samples),
                    angle_min_deg=None if not angles else min(angles),
                    angle_max_deg=None if not angles else max(angles),
                    max_abs_speed_rpm=None if not speeds else max(abs(value) for value in speeds),
                    max_abs_current_a=None
                    if not currents
                    else max(abs(value) for value in currents),
                )
            )
        return summaries

    def export_csv(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        samples = self.snapshot()
        with destination.open("w", encoding="utf-8-sig", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "timestamp",
                    "elapsed_s",
                    "arm",
                    "joint",
                    "angle_deg",
                    "speed_rpm",
                    "current_a",
                    "error_code",
                ]
            )
            for sample in samples:
                writer.writerow(
                    [
                        sample.timestamp_iso,
                        f"{sample.elapsed_s:.6f}",
                        sample.arm,
                        sample.joint,
                        _csv_value(sample.angle_deg),
                        _csv_value(sample.speed_rpm),
                        _csv_value(sample.current_a),
                        sample.error_code,
                    ]
                )
        return destination

    def export_peak_csv(self, path: str | Path) -> Path:
        """Export one peak-summary row for every recorded arm/joint pair."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("w", encoding="utf-8-sig", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "arm",
                    "joint",
                    "sample_count",
                    "angle_min_deg",
                    "angle_max_deg",
                    "max_abs_speed_rpm",
                    "max_abs_current_a",
                ]
            )
            for summary in self.peak_summaries():
                writer.writerow(
                    [
                        summary.arm,
                        summary.joint,
                        summary.sample_count,
                        _csv_value(summary.angle_min_deg),
                        _csv_value(summary.angle_max_deg),
                        _csv_value(summary.max_abs_speed_rpm),
                        _csv_value(summary.max_abs_current_a),
                    ]
                )
        return destination


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    number = float(value)
    return number if np.isfinite(number) else None


def _finite_values(values: object) -> list[float]:
    result: list[float] = []
    for value in values:
        if value is not None and np.isfinite(value):
            result.append(float(value))
    return result


def _csv_value(value: float | None) -> str:
    return "" if value is None else f"{value:.9g}"


__all__ = ["PeakSummary", "TelemetryRecorder", "TelemetrySample"]
