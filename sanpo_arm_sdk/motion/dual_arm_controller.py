"""Synchronized planning and execution for two independent F4 arm controllers."""

from __future__ import annotations

import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, replace
from typing import Optional

import numpy as np

from ..errors import ERR_INVALID_ARGUMENT, ERR_MOTION_BUSY, OK
from ..kinematics.guiji_quintic import JointTrajectory
from ..kinematics.kinematic_5dof import IKRecommendConfig
from .arm_controller import ArmController, PreparedMotion


@dataclass(frozen=True)
class DualMotionResult:
    left_error: int
    right_error: int

    @property
    def success(self) -> bool:
        return self.left_error == OK and self.right_error == OK


def _retime_trajectory(
    trajectory: JointTrajectory,
    duration_s: float,
) -> JointTrajectory:
    old_duration = float(trajectory.time_s[-1])
    if old_duration <= 0.0 or abs(old_duration - duration_s) <= 1e-9:
        return trajectory
    scale = float(duration_s) / old_duration
    return replace(
        trajectory,
        time_s=np.asarray(trajectory.time_s, dtype=float) * scale,
        qd_deg_s=np.asarray(trajectory.qd_deg_s, dtype=float) / scale,
        qdd_deg_s2=np.asarray(trajectory.qdd_deg_s2, dtype=float) / (scale**2),
    )


def _retime_motion(motion: PreparedMotion, duration_s: float) -> PreparedMotion:
    trajectory = _retime_trajectory(motion.trajectory, duration_s)
    cartesian_plan = motion.cartesian_plan
    line_plan = motion.line_plan
    if cartesian_plan is not None:
        cartesian_plan = replace(cartesian_plan, joint_trajectory=trajectory)
    if line_plan is not None:
        line_plan = replace(
            line_plan,
            joint_trajectory=trajectory,
            adjusted_duration_s=float(duration_s),
        )
    return replace(
        motion,
        trajectory=trajectory,
        cartesian_plan=cartesian_plan,
        line_plan=line_plan,
    )


class DualArmController:
    """Coordinate left/right plans and release both F4 streams together."""

    def __init__(self, left: ArmController, right: ArmController) -> None:
        self.left = left
        self.right = right
        self.last_result = DualMotionResult(OK, OK)
        self.last_prepared: dict[str, PreparedMotion] = {}
        self._motion_thread: threading.Thread | None = None
        self._cancel = threading.Event()
        self._lock = threading.Lock()

    @property
    def is_moving(self) -> bool:
        coordinator_running = (
            self._motion_thread is not None and self._motion_thread.is_alive()
        )
        return coordinator_running or self.left.is_moving or self.right.is_moving

    def connect(self) -> DualMotionResult:
        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(self.left.connect)
            right_future = executor.submit(self.right.connect)
            result = DualMotionResult(left_future.result(), right_future.result())
        self.last_result = result
        return result

    def sync_state(self) -> DualMotionResult:
        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(self.left.sync_state)
            right_future = executor.submit(self.right.sync_state)
            result = DualMotionResult(left_future.result(), right_future.result())
        self.last_result = result
        return result

    def close(self) -> DualMotionResult:
        self.stop(disable=False)
        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(self.left.close)
            right_future = executor.submit(self.right.close)
            result = DualMotionResult(left_future.result(), right_future.result())
        self.last_result = result
        return result

    def stop(self, *, disable: bool = True) -> DualMotionResult:
        self._cancel.set()
        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(self.left.stop_motion, disable=disable)
            right_future = executor.submit(self.right.stop_motion, disable=disable)
            result = DualMotionResult(left_future.result(), right_future.result())
        thread = self._motion_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self.last_result = result
        return result

    def _prepare_one(
        self,
        arm: ArmController,
        mode: str,
        target: object,
        *,
        speed: object,
        accel: object,
        sample_period_s: float,
        total_time_s: Optional[float],
        allow_recommendation: bool,
        recommendation_config: Optional[IKRecommendConfig],
        position_tolerance_mm: float,
        max_branch_jump_deg: float,
    ) -> tuple[int, Optional[PreparedMotion]]:
        if mode == "joint":
            return arm.prepare_move_j(
                target,
                speed=speed,
                accel=accel,
                sample_period_s=sample_period_s,
            )
        if mode in {"cartesian", "cartesian_ptp", "ptp"}:
            return arm.prepare_move_cart(
                target,
                speed=speed,
                accel=accel,
                sample_period_s=sample_period_s,
                allow_recommendation=allow_recommendation,
                recommendation_config=recommendation_config,
            )
        if mode in {"line", "cartesian_line"}:
            return arm.prepare_move_line(
                target,
                speed=speed,
                accel=accel,
                total_time_s=total_time_s,
                sample_period_s=sample_period_s,
                position_tolerance_mm=position_tolerance_mm,
                max_branch_jump_deg=max_branch_jump_deg,
            )
        return ERR_INVALID_ARGUMENT, None

    def prepare_both(
        self,
        left_target: object,
        right_target: object,
        *,
        mode: str,
        speed: object = 10.0,
        accel: object = 20.0,
        sample_period_s: float = 0.05,
        total_time_s: Optional[float] = None,
        allow_recommendation: bool = False,
        recommendation_config: Optional[IKRecommendConfig] = None,
        position_tolerance_mm: float = 0.1,
        max_branch_jump_deg: float = 25.0,
        synchronize_finish: bool = True,
    ) -> tuple[DualMotionResult, Optional[dict[str, PreparedMotion]]]:
        """Plan both sides concurrently and optionally stretch them to finish together."""

        if self.is_moving:
            result = DualMotionResult(ERR_MOTION_BUSY, ERR_MOTION_BUSY)
            return result, None
        mode_key = str(mode).strip().lower()
        arguments = dict(
            speed=speed,
            accel=accel,
            sample_period_s=sample_period_s,
            total_time_s=total_time_s,
            allow_recommendation=allow_recommendation,
            recommendation_config=recommendation_config,
            position_tolerance_mm=position_tolerance_mm,
            max_branch_jump_deg=max_branch_jump_deg,
        )
        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(
                self._prepare_one,
                self.left,
                mode_key,
                left_target,
                **arguments,
            )
            right_future = executor.submit(
                self._prepare_one,
                self.right,
                mode_key,
                right_target,
                **arguments,
            )
            left_error, left_motion = left_future.result()
            right_error, right_motion = right_future.result()
        result = DualMotionResult(left_error, right_error)
        if not result.success or left_motion is None or right_motion is None:
            self.last_result = result
            return result, None

        if synchronize_finish:
            duration = max(
                float(left_motion.trajectory.time_s[-1]),
                float(right_motion.trajectory.time_s[-1]),
            )
            left_motion = _retime_motion(left_motion, duration)
            right_motion = _retime_motion(right_motion, duration)
        prepared = {"left": left_motion, "right": right_motion}
        self.last_prepared = prepared
        self.last_result = result
        return result, prepared

    def _run_prepared_pair(
        self,
        prepared: dict[str, PreparedMotion],
        *,
        timeout_s: Optional[float],
        tol_deg: float,
    ) -> DualMotionResult:
        start_event = threading.Event()
        ready = {"left": threading.Event(), "right": threading.Event()}
        errors = {"left": ERR_MOTION_BUSY, "right": ERR_MOTION_BUSY}

        def worker(name: str, arm: ArmController) -> None:
            ready[name].set()
            errors[name] = arm.execute_prepared_motion(
                prepared[name],
                blocking=True,
                timeout_s=timeout_s,
                tol_deg=tol_deg,
                start_event=start_event,
                configured=True,
            )

        threads = [
            threading.Thread(
                target=worker,
                args=("left", self.left),
                daemon=True,
                name="left-synchronized-motion",
            ),
            threading.Thread(
                target=worker,
                args=("right", self.right),
                daemon=True,
                name="right-synchronized-motion",
            ),
        ]
        for thread in threads:
            thread.start()
        ready["left"].wait(1.0)
        ready["right"].wait(1.0)
        start_event.set()
        for thread in threads:
            thread.join()
        result = DualMotionResult(errors["left"], errors["right"])
        self.last_result = result
        return result

    def execute_prepared_both(
        self,
        prepared: dict[str, PreparedMotion],
        *,
        blocking: bool = True,
        timeout_s: Optional[float] = None,
        tol_deg: float = 1.0,
    ) -> DualMotionResult:
        """Configure both arms, then launch both trajectories from one event."""

        if self.is_moving:
            return DualMotionResult(ERR_MOTION_BUSY, ERR_MOTION_BUSY)
        left_error = self.left.configure_prepared_motion(prepared["left"])
        right_error = self.right.configure_prepared_motion(prepared["right"])
        configured = DualMotionResult(left_error, right_error)
        if not configured.success:
            self.last_result = configured
            return configured

        self._cancel.clear()
        if blocking:
            return self._run_prepared_pair(
                prepared,
                timeout_s=timeout_s,
                tol_deg=tol_deg,
            )

        def coordinator() -> None:
            self._run_prepared_pair(
                prepared,
                timeout_s=timeout_s,
                tol_deg=tol_deg,
            )

        with self._lock:
            self._motion_thread = threading.Thread(
                target=coordinator,
                daemon=True,
                name="dual-arm-coordinator",
            )
            self._motion_thread.start()
        return DualMotionResult(OK, OK)

    def move_both(
        self,
        left_target: object,
        right_target: object,
        *,
        mode: str,
        speed: object = 10.0,
        accel: object = 20.0,
        sample_period_s: float = 0.05,
        total_time_s: Optional[float] = None,
        allow_recommendation: bool = False,
        recommendation_config: Optional[IKRecommendConfig] = None,
        position_tolerance_mm: float = 0.1,
        max_branch_jump_deg: float = 25.0,
        synchronize_finish: bool = True,
        blocking: bool = True,
        timeout_s: Optional[float] = None,
        tol_deg: float = 1.0,
    ) -> DualMotionResult:
        result, prepared = self.prepare_both(
            left_target,
            right_target,
            mode=mode,
            speed=speed,
            accel=accel,
            sample_period_s=sample_period_s,
            total_time_s=total_time_s,
            allow_recommendation=allow_recommendation,
            recommendation_config=recommendation_config,
            position_tolerance_mm=position_tolerance_mm,
            max_branch_jump_deg=max_branch_jump_deg,
            synchronize_finish=synchronize_finish,
        )
        if not result.success or prepared is None:
            return result
        return self.execute_prepared_both(
            prepared,
            blocking=blocking,
            timeout_s=timeout_s,
            tol_deg=tol_deg,
        )


__all__ = ["DualArmController", "DualMotionResult"]
