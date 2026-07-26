"""Threaded Gloria-M telemetry recording and CSV export."""

from __future__ import annotations

import csv
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from .dual import DualGripperController
from .models import GripperState


@dataclass(frozen=True)
class GripperTelemetrySample:
    timestamp_iso: str
    elapsed_s: float
    side: str
    position_rad: float
    opening_fraction: float | None
    velocity_rad_s: float
    torque_nm: float
    mos_temperature_c: int
    rotor_temperature_c: int
    status_code: int
    status: str


@dataclass(frozen=True)
class GripperPeakSummary:
    side: str
    sample_count: int
    position_min_rad: float | None
    position_max_rad: float | None
    max_abs_velocity_rad_s: float | None
    max_abs_torque_nm: float | None
    max_mos_temperature_c: int | None
    max_rotor_temperature_c: int | None


class GripperTelemetryRecorder:
    """Poll two independent grippers without coupling recording to the GUI."""

    def __init__(
        self,
        grippers: DualGripperController,
        *,
        sample_period_s: float = 0.1,
        max_samples_per_side: int = 10_000,
    ) -> None:
        if sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be positive")
        if max_samples_per_side < 2:
            raise ValueError("max_samples_per_side must be at least two")
        self.grippers = grippers
        self.sample_period_s = float(sample_period_s)
        self._samples = {
            "left": deque(maxlen=max_samples_per_side),
            "right": deque(maxlen=max_samples_per_side),
        }
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._started_monotonic = time.monotonic()
        self.last_error = ""

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
            name="gripper-telemetry",
        )
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=timeout_s)

    def clear(self) -> None:
        with self._lock:
            for values in self._samples.values():
                values.clear()
        self._started_monotonic = time.monotonic()
        self.last_error = ""

    def poll_once(self) -> list[GripperTelemetrySample]:
        result, states = self.grippers.refresh_both()
        if not result.success:
            self.last_error = (
                f"left={result.left_error or 'OK'}, "
                f"right={result.right_error or 'OK'}"
            )
        elapsed = max(0.0, time.monotonic() - self._started_monotonic)
        timestamp = datetime.now(timezone.utc).astimezone().isoformat(
            timespec="milliseconds"
        )
        batch: list[GripperTelemetrySample] = []
        for side in ("left", "right"):
            state = states.get(side)
            if isinstance(state, GripperState):
                batch.append(
                    GripperTelemetrySample(
                        timestamp_iso=timestamp,
                        elapsed_s=elapsed,
                        side=side,
                        position_rad=state.position_rad,
                        opening_fraction=state.opening_fraction,
                        velocity_rad_s=state.velocity_rad_s,
                        torque_nm=state.torque_nm,
                        mos_temperature_c=state.mos_temperature_c,
                        rotor_temperature_c=state.rotor_temperature_c,
                        status_code=state.status_code,
                        status=state.status,
                    )
                )
        with self._lock:
            for sample in batch:
                self._samples[sample.side].append(sample)
        return batch

    def _run(self) -> None:
        deadline = time.monotonic()
        while not self._stop_event.is_set():
            try:
                self.poll_once()
            except Exception as exc:
                self.last_error = str(exc)
            deadline += self.sample_period_s
            wait_s = max(0.0, deadline - time.monotonic())
            if self._stop_event.wait(wait_s):
                break
            if wait_s <= 0.0:
                deadline = time.monotonic()

    def snapshot(self, side: str | None = None) -> list[GripperTelemetrySample]:
        with self._lock:
            samples = [
                sample
                for name, values in self._samples.items()
                if side is None or side == name
                for sample in values
            ]
        return sorted(samples, key=lambda sample: (sample.elapsed_s, sample.side))

    def latest(self) -> dict[str, GripperTelemetrySample]:
        with self._lock:
            return {
                side: values[-1]
                for side, values in self._samples.items()
                if values
            }

    def peak_summaries(self) -> list[GripperPeakSummary]:
        with self._lock:
            items = {
                side: list(values)
                for side, values in self._samples.items()
            }
        result: list[GripperPeakSummary] = []
        for side, samples in items.items():
            result.append(
                GripperPeakSummary(
                    side=side,
                    sample_count=len(samples),
                    position_min_rad=min(
                        (sample.position_rad for sample in samples),
                        default=None,
                    ),
                    position_max_rad=max(
                        (sample.position_rad for sample in samples),
                        default=None,
                    ),
                    max_abs_velocity_rad_s=max(
                        (abs(sample.velocity_rad_s) for sample in samples),
                        default=None,
                    ),
                    max_abs_torque_nm=max(
                        (abs(sample.torque_nm) for sample in samples),
                        default=None,
                    ),
                    max_mos_temperature_c=max(
                        (sample.mos_temperature_c for sample in samples),
                        default=None,
                    ),
                    max_rotor_temperature_c=max(
                        (sample.rotor_temperature_c for sample in samples),
                        default=None,
                    ),
                )
            )
        return result

    def export_csv(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open(
            "w",
            encoding="utf-8-sig",
            newline="",
        ) as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "timestamp",
                    "elapsed_s",
                    "side",
                    "position_rad",
                    "opening_fraction",
                    "velocity_rad_s",
                    "torque_nm",
                    "mos_temperature_c",
                    "rotor_temperature_c",
                    "status_code",
                    "status",
                ]
            )
            for sample in self.snapshot():
                writer.writerow(
                    [
                        sample.timestamp_iso,
                        f"{sample.elapsed_s:.6f}",
                        sample.side,
                        f"{sample.position_rad:.9g}",
                        ""
                        if sample.opening_fraction is None
                        else f"{sample.opening_fraction:.9g}",
                        f"{sample.velocity_rad_s:.9g}",
                        f"{sample.torque_nm:.9g}",
                        sample.mos_temperature_c,
                        sample.rotor_temperature_c,
                        sample.status_code,
                        sample.status,
                    ]
                )
        return destination

    def export_peak_csv(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open(
            "w",
            encoding="utf-8-sig",
            newline="",
        ) as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "side",
                    "sample_count",
                    "position_min_rad",
                    "position_max_rad",
                    "max_abs_velocity_rad_s",
                    "max_abs_torque_nm",
                    "max_mos_temperature_c",
                    "max_rotor_temperature_c",
                ]
            )
            for summary in self.peak_summaries():
                writer.writerow(
                    [
                        summary.side,
                        summary.sample_count,
                        _csv_optional(summary.position_min_rad),
                        _csv_optional(summary.position_max_rad),
                        _csv_optional(summary.max_abs_velocity_rad_s),
                        _csv_optional(summary.max_abs_torque_nm),
                        _csv_optional(summary.max_mos_temperature_c),
                        _csv_optional(summary.max_rotor_temperature_c),
                    ]
                )
        return destination


def _csv_optional(value: float | int | None) -> str:
    return "" if value is None else f"{value:.9g}"


__all__ = [
    "GripperPeakSummary",
    "GripperTelemetryRecorder",
    "GripperTelemetrySample",
]
