# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file.

"""Isaac Sim window and articulation bridge for the gripper fault injector."""

from __future__ import annotations

import random
from dataclasses import replace
from pathlib import Path

import numpy as np
import omni.ext
import omni.physx
import omni.timeline
import omni.ui as ui
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.xforms import get_world_pose

from gangelia.fault_injector.core import (
    CUSTOM_PROFILE_INDEX,
    DEMO_FAULT_NOTICE,
    FAULT_CHOICES,
    PRESETS,
    AutomaticDrop,
    AutomaticDropConfig,
    CarrySample,
    ParallelGripperProfile,
    fault_notice,
    inject_drop,
    parse_parallel_gripper_profile,
    profile_for_selection,
    resolve_joint_indices,
)
from gangelia.fault_injector.regrasp import (
    RegraspCommand,
    RegraspSequence,
    grasp_pinch_orientation,
    object_clear_of_hand,
    pinch_from_panda_hand,
    resume_drop_watch,
)
from gangelia.fault_injector.theme import BUTTON, BUTTON_LABEL, CARD, HAIRLINE, INK, MUTED, STATUS, SURFACE

READY_STATUS = "Ready. Start the timeline first."
_LOGO_PATH = Path(__file__).resolve().parents[2] / "data" / "preview.png"
_LOGO_WIDTH = 92
_LOGO_HEIGHT = 28


def _fault_row_style(selected: bool) -> dict:
    return {
        "Button": {
            "background_color": CARD,
            "border_color": INK if selected else HAIRLINE,
            "border_width": 2 if selected else 1,
            "border_radius": 8,
            "margin_height": 2,
        },
        "Button.Label": {"color": INK, "font_size": 15},
    }


def _field_style() -> dict:
    return {
        "Field": {
            "background_color": CARD,
            "color": INK,
            "border_color": HAIRLINE,
            "border_width": 1,
            "border_radius": 6,
            "font_size": 14,
        }
    }


class IsaacArticulationDriver:
    """Apply named joint targets to one live Isaac Sim articulation."""

    def __init__(self, articulation_path: str) -> None:
        """Remember which articulation this driver will command.

        Args:
            articulation_path: USD path of the robot articulation.
        """
        self._articulation_path = articulation_path

    def set_joint_positions(self, joint_names: tuple[str, ...], positions: tuple[float, ...]) -> None:
        """Open the named joints on the articulation at ``articulation_path``."""
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            raise RuntimeError("Start the simulation timeline before injecting a drop")

        articulation = SingleArticulation(self._articulation_path)
        articulation.initialize()
        indices = resolve_joint_indices(tuple(articulation.dof_names), joint_names)
        articulation.apply_action(
            ArticulationAction(
                joint_positions=np.asarray(positions, dtype=np.float32),
                joint_indices=np.asarray(indices, dtype=np.int32),
            )
        )


def _world_translation(prim_path: str) -> tuple[float, float, float]:
    """Return the current world translation of a USD prim."""
    position, _orientation = get_world_pose(prim_path)
    return (float(position[0]), float(position[1]), float(position[2]))


_EXTENSION: Extension | None = None


def get_extension() -> Extension:
    """Return the running Gripper Fault Injector, if Isaac Sim enabled it."""
    if _EXTENSION is None:
        raise RuntimeError("Gripper Fault Injector is not enabled")
    return _EXTENSION


class Extension(omni.ext.IExt):
    """Dockable panel that opens a parallel gripper."""

    def on_startup(self, ext_id: str) -> None:
        """Create the Inject Drop window."""
        self._ext_id = ext_id
        self._window = ui.Window(
            "Gangelia Fault Injector",
            width=460,
            height=640,
            visible=True,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        self._path_model = ui.SimpleStringModel("/World/Franka")
        self._object_path_model = ui.SimpleStringModel("")
        self._probability_model = ui.SimpleStringModel("0.0001")
        self._automatic_model = ui.SimpleBoolModel(False)
        self._automatic_regrasp_model = ui.SimpleBoolModel(False)
        self._grasp_frame_model = ui.SimpleStringModel("")
        self._joint_names_model = ui.SimpleStringModel("")
        self._open_positions_model = ui.SimpleStringModel("")
        self._closed_positions_model = ui.SimpleStringModel("")
        self._min_closure_model = ui.SimpleStringModel("")
        self._distance_model = ui.SimpleStringModel("")
        self._gripper_index = 0
        self._fault_index = 0
        self._gripper_combo = None
        self._custom_stack = None
        self._drop_stack = None
        self._demo_stack = None
        self._fault_buttons: list[ui.Button] = []
        self._status_label = None
        self._status_mark = None
        self._drop: AutomaticDrop | None = None
        self._profile: ParallelGripperProfile | None = None
        self._physics_subscription = None
        self._articulation: SingleArticulation | None = None
        self._reported_phase = ""
        self._grasp_frame_override: str | None = None
        self._rearm_after_drop = True
        self._updating_checkbox = False
        self._regrasp = RegraspSequence()
        self._regrasp_pending = False
        self._regrasp_profile: ParallelGripperProfile | None = None
        self._regrasp_orientation: tuple[float, float, float, float] | None = None
        self._regrasp_reported = ""
        self._articulation_path = ""
        self._ik_solver = None
        self._physics_dt = 0.0
        self._copy_profile(PRESETS[0])
        self._build_ui()
        self._automatic_model.add_value_changed_fn(self._on_automatic_toggled)
        self._automatic_regrasp_model.add_value_changed_fn(self._on_automatic_regrasp_toggled)
        global _EXTENSION
        _EXTENSION = self

    def on_shutdown(self) -> None:
        """Drop the window when the extension is disabled."""
        global _EXTENSION
        _EXTENSION = None
        self._cancel_regrasp_motion()
        self._stop_automatic()
        self._window = None
        self._path_model = None
        self._object_path_model = None
        self._probability_model = None
        self._automatic_model = None
        self._automatic_regrasp_model = None
        self._grasp_frame_model = None
        self._joint_names_model = None
        self._open_positions_model = None
        self._closed_positions_model = None
        self._min_closure_model = None
        self._distance_model = None
        self._gripper_combo = None
        self._custom_stack = None
        self._drop_stack = None
        self._demo_stack = None
        self._fault_buttons = []
        self._status_label = None
        self._status_mark = None

    def _build_ui(self) -> None:
        self._window.frame.style = {
            "background_color": SURFACE,
            "ComboBox": {
                "color": INK,
                "background_color": CARD,
                "secondary_color": CARD,
                "border_color": HAIRLINE,
                "border_radius": 6,
            },
            "CheckBox": {"background_color": BUTTON, "color": BUTTON_LABEL, "border_radius": 3},
        }
        with self._window.frame, ui.VStack(spacing=8):
            ui.Spacer(height=8)
            with ui.ScrollingFrame(
                height=ui.Fraction(1),
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                self._build_scrolled_fields()
            ui.Spacer(height=8)
        self._fit_window()

    def _build_scrolled_fields(self) -> None:
        with ui.VStack(spacing=8, height=0):
            with ui.HStack(height=32):
                ui.Label("FAULT", width=0, height=32, style={"color": MUTED, "font_size": 22})
                ui.Spacer()
                ui.Image(
                    str(_LOGO_PATH),
                    width=_LOGO_WIDTH,
                    height=_LOGO_HEIGHT,
                    alignment=ui.Alignment.RIGHT_CENTER,
                    fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                )
            self._fault_buttons = []
            for index, choice in enumerate(FAULT_CHOICES):
                label = choice.name if choice.available else f"{choice.name}    Demo"
                button = ui.Button(
                    label,
                    height=32,
                    clicked_fn=lambda i=index: self._select_fault(i),
                    style=_fault_row_style(index == 0),
                )
                self._fault_buttons.append(button)
            self._drop_stack = ui.VStack(spacing=8, height=0)
            with self._drop_stack:
                ui.Label("Gripper", height=18, style={"color": MUTED, "font_size": 14})
                self._gripper_combo = ui.ComboBox(0, *[profile.name for profile in PRESETS], "Custom")
                self._custom_stack = ui.VStack(spacing=8, height=0, visible=False)
                with self._custom_stack:
                    self._labeled_field("Grasp frame", self._grasp_frame_model)
                    self._labeled_field("Joint names", self._joint_names_model)
                    self._labeled_field("Open positions", self._open_positions_model)
                    self._labeled_field("Closed positions", self._closed_positions_model)
                    self._labeled_field("Minimum closure (0 to 1)", self._min_closure_model)
                    self._labeled_field("Pickup distance (m)", self._distance_model)
                self._gripper_combo.model.add_item_changed_fn(self._on_gripper_changed)
                self._labeled_field("Articulation prim path", self._path_model)
                self._labeled_field("Target object prim path", self._object_path_model)
                self._labeled_field("Drop probability per physics step", self._probability_model)
                with ui.HStack(spacing=8, height=24):
                    ui.CheckBox(
                        model=self._automatic_model,
                        width=18,
                        height=18,
                        style={
                            "background_color": BUTTON,
                            "color": BUTTON_LABEL,
                            "border_radius": 3,
                        },
                    )
                    ui.Label("Automatic drop after pickup", height=22, style={"color": INK, "font_size": 14})
                with ui.HStack(spacing=8, height=24):
                    ui.CheckBox(
                        model=self._automatic_regrasp_model,
                        width=18,
                        height=18,
                        style={
                            "background_color": BUTTON,
                            "color": BUTTON_LABEL,
                            "border_radius": 3,
                        },
                    )
                    ui.Label("Automatic regrasp after drop", height=22, style={"color": INK, "font_size": 14})
                with ui.HStack(height=32):
                    ui.Button(
                        "Inject Drop",
                        width=160,
                        height=32,
                        clicked_fn=self._inject_drop,
                        style_type_name_override="InjectDrop",
                        style={
                            "InjectDrop": {
                                "background_color": CARD,
                                "border_color": INK,
                                "border_width": 1,
                                "border_radius": 8,
                            },
                            "InjectDrop.Label": {"color": INK, "font_size": 15},
                        },
                    )
                    ui.Spacer()
                with ui.HStack(height=32):
                    ui.Button(
                        "Regrasp",
                        width=160,
                        height=32,
                        clicked_fn=self._on_regrasp,
                        style_type_name_override="Regrasp",
                        style={
                            "Regrasp": {
                                "background_color": CARD,
                                "border_color": INK,
                                "border_width": 1,
                                "border_radius": 8,
                            },
                            "Regrasp.Label": {"color": INK, "font_size": 15},
                        },
                    )
                    ui.Spacer()
            self._demo_stack = ui.VStack(spacing=8, height=0, visible=False)
            with self._demo_stack:
                ui.Label(
                    DEMO_FAULT_NOTICE,
                    word_wrap=True,
                    height=48,
                    style={"color": INK, "font_size": 18},
                )
            with ui.HStack(spacing=8, height=48):
                self._status_mark = ui.Rectangle(
                    width=9,
                    height=9,
                    visible=False,
                    style={"background_color": STATUS, "border_radius": 5},
                )
                self._status_label = ui.Label(
                    READY_STATUS,
                    word_wrap=True,
                    height=48,
                    style={"color": INK, "font_size": 14},
                )

    def _labeled_field(self, label: str, model: ui.SimpleStringModel) -> None:
        ui.Label(label, height=18, style={"color": MUTED, "font_size": 14})
        ui.StringField(model=model, height=24, style=_field_style())

    def _select_fault(self, index: int, *, announce: bool = True) -> None:
        if index == self._fault_index:
            return
        notice = fault_notice(index)
        leaving_drop = self._fault_index == 0
        self._fault_index = index
        for button_index, button in enumerate(self._fault_buttons):
            button.style = _fault_row_style(button_index == index)
        self._drop_stack.visible = notice is None
        self._demo_stack.visible = notice is not None
        if leaving_drop:
            self._cancel_regrasp_motion()
            if self._automatic_model.get_value_as_bool():
                self._automatic_model.set_value(False)
            else:
                self._stop_automatic()
        if announce:
            self._set_status(READY_STATUS if notice is None else notice)
        self._fit_window()

    def _fit_window(self) -> None:
        if fault_notice(self._fault_index) is not None:
            self._window.height = 360
        elif self._gripper_index == CUSTOM_PROFILE_INDEX:
            self._window.height = 1052
        else:
            self._window.height = 712

    def _on_gripper_changed(self, model: ui.AbstractItemModel, _item: ui.AbstractItem) -> None:
        index = model.get_item_value_model().get_value_as_int()
        if index == CUSTOM_PROFILE_INDEX and self._gripper_index != CUSTOM_PROFILE_INDEX:
            self._copy_profile(PRESETS[self._gripper_index])
        self._gripper_index = index
        self._custom_stack.visible = index == CUSTOM_PROFILE_INDEX
        self._fit_window()

    def _copy_profile(self, profile: ParallelGripperProfile) -> None:
        self._grasp_frame_model.set_value(profile.grasp_frame)
        self._joint_names_model.set_value(", ".join(profile.joint_names))
        self._open_positions_model.set_value(", ".join(f"{value:.6g}" for value in profile.open_positions))
        self._closed_positions_model.set_value(
            ", ".join(f"{value:.6g}" for value in profile.closed_positions)
        )
        self._min_closure_model.set_value(f"{profile.min_closure:.6g}")
        self._distance_model.set_value(f"{profile.distance_threshold_m:.6g}")

    def _selected_profile(self) -> ParallelGripperProfile:
        custom = None
        if self._gripper_index == CUSTOM_PROFILE_INDEX:
            custom = parse_parallel_gripper_profile(
                name="Custom",
                grasp_frame=self._grasp_frame_model.get_value_as_string(),
                joint_names=self._joint_names_model.get_value_as_string(),
                open_positions=self._open_positions_model.get_value_as_string(),
                closed_positions=self._closed_positions_model.get_value_as_string(),
                min_closure=self._min_closure_model.get_value_as_string(),
                distance_threshold_m=self._distance_model.get_value_as_string(),
            )
        return profile_for_selection(self._gripper_index, custom)

    def _on_automatic_toggled(self, model: ui.SimpleBoolModel) -> None:
        if self._updating_checkbox:
            return
        if model.get_value_as_bool():
            self._start_automatic()
        else:
            self._stop_automatic()
            self._rearm_after_drop = True

    def _start_automatic(self) -> None:
        object_path = self._object_path_model.get_value_as_string().strip()
        robot_path = self._path_model.get_value_as_string().strip()
        if not robot_path or not object_path:
            self._reject_automatic("Enter the articulation path and the target object prim path.")
            return
        try:
            probability = float(self._probability_model.get_value_as_string().strip())
            config = AutomaticDropConfig(
                probability_per_step=probability, rearm_after_drop=self._rearm_after_drop
            )
            profile = self._selected_profile()
            if self._grasp_frame_override:
                profile = replace(profile, grasp_frame=self._grasp_frame_override)
        except ValueError as exc:
            self._reject_automatic(str(exc))
            return
        self._cancel_regrasp_motion()
        self._stop_automatic()
        self._profile = profile
        self._drop = AutomaticDrop(config, profile, random.Random())
        self._articulation = None
        self._reported_phase = "watching"
        self._physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )
        frame = profile.grasp_frame_path(robot_path)
        self._set_status(f"Watching for pickup at {frame}. Play the timeline.", live=True)

    def arm(
        self,
        robot_path: str,
        object_path: str,
        probability: float,
        profile_index: int = 0,
        grasp_frame: str | None = None,
        rearm_after_drop: bool = False,
    ) -> None:
        """Turn on automatic drop for one named object and gripper profile.

        ``grasp_frame`` replaces the profile frame when the gripper is mounted
        deeper than the preset suffix, for example a 2F-85 on a Franka.
        A scripted arming drops once unless ``rearm_after_drop`` is set.
        Checking the box in the window always re-arms after each drop.
        """
        self._select_fault(0, announce=False)
        self._grasp_frame_override = grasp_frame
        self._rearm_after_drop = rearm_after_drop
        self._path_model.set_value(robot_path)
        self._object_path_model.set_value(object_path)
        self._probability_model.set_value(str(probability))
        self._gripper_combo.model.get_item_value_model().set_value(profile_index)
        if self._automatic_model.get_value_as_bool():
            self._start_automatic()
        else:
            self._automatic_model.set_value(True)

    def disarm(self) -> None:
        """Turn automatic drop off and forget the current pickup."""
        if self._automatic_model.get_value_as_bool():
            self._automatic_model.set_value(False)
        else:
            self._stop_automatic()

    @property
    def phase(self) -> str:
        """Detector phase, or ``off`` when automatic drop is not armed."""
        if self._drop is None:
            return "off"
        return self._drop.phase

    def _reject_automatic(self, text: str) -> None:
        self._stop_automatic()
        self._updating_checkbox = True
        self._automatic_model.set_value(False)
        self._updating_checkbox = False
        self._set_status(text)

    def _stop_automatic(self) -> None:
        if not self._regrasp.active:
            self._physics_subscription = None
            self._articulation = None
            self._articulation_path = ""
            self._ik_solver = None
        self._drop = None
        self._profile = None
        self._reported_phase = ""

    def _on_physics_step(self, step_size: float) -> None:
        self._physics_dt = float(step_size)
        if self._regrasp_pending and not self._regrasp.active:
            self._regrasp_pending = False
            if self._automatic_regrasp_model.get_value_as_bool() and self._regrasp.released:
                self._arm_regrasp(allow_clear=False)
        if self._regrasp.active:
            if self._physics_dt <= 0:
                return
            try:
                self._step_regrasp()
            except Exception as exc:
                self._finish_regrasp(f"Regrasp stopped: {exc}", resume_watch=False)
            return
        drop = self._drop
        if drop is None:
            return
        try:
            sample = self._read_sample()
            should_open = drop.observe(sample)
            if should_open:
                inject_drop(
                    IsaacArticulationDriver(self._path_model.get_value_as_string().strip()),
                    self._profile.as_adapter(),
                )
        except Exception as exc:
            self._reject_automatic(f"Automatic drop stopped: {exc}")
            return
        if drop.phase != self._reported_phase:
            self._reported_phase = drop.phase
            if drop.phase == "armed":
                self._cancel_regrasp_motion()
                self._set_status(
                    "Pickup confirmed. Rolling the drop probability every physics step.", live=True
                )
            elif drop.phase == "latching":
                self._set_status("Drop hit. Holding the gripper open.", live=True)
            elif drop.phase == "done":
                self._regrasp.note_drop_released()
                if self._automatic_regrasp_model.get_value_as_bool():
                    self._request_automatic_regrasp()
                elif self._rearm_after_drop:
                    self._set_status(
                        "Drop finished. Waiting for the object to leave the jaws.", live=True
                    )
                else:
                    self._physics_subscription = None
                    self._set_status("Drop finished.")
            elif drop.phase == "watching":
                self._set_status("Watching for the next pickup.", live=True)

    def _read_sample(self) -> CarrySample:
        object_path = self._object_path_model.get_value_as_string().strip()
        gripper_path = self._profile.grasp_frame_path(self._path_model.get_value_as_string().strip())
        return CarrySample(
            object_position=_world_translation(object_path),
            gripper_position=_world_translation(gripper_path),
            gripper_joint_positions=self._finger_positions(),
        )

    def _finger_positions(self) -> tuple[float, ...]:
        robot_path = self._path_model.get_value_as_string().strip()
        articulation = self._ensure_articulation(robot_path)
        positions = articulation.get_joint_positions()
        indices = resolve_joint_indices(tuple(articulation.dof_names), self._profile.joint_names)
        return tuple(float(positions[index]) for index in indices)

    def _set_status(self, text: str, *, live: bool = False) -> None:
        if self._status_label is not None:
            self._status_label.text = text
        if self._status_mark is not None:
            self._status_mark.visible = live

    def _inject_drop(self) -> None:
        path = self._path_model.get_value_as_string().strip()
        if not path:
            self._set_status("Enter an articulation prim path.")
            return
        try:
            profile = self._selected_profile()
            result = inject_drop(IsaacArticulationDriver(path), profile.as_adapter())
            self._regrasp.note_drop_released()
        except Exception as exc:
            self._set_status(f"Injection failed: {exc}")
            return
        self._set_status(f"Open command sent through {result.adapter_name}.")
        self._request_automatic_regrasp()

    def _on_regrasp(self) -> None:
        self._arm_regrasp(allow_clear=True)

    def _on_automatic_regrasp_toggled(self, model: ui.SimpleBoolModel) -> None:
        if self._updating_checkbox or self._regrasp.active:
            return
        if not model.get_value_as_bool():
            self._regrasp_pending = False
            return
        if not self._regrasp.released:
            self._set_status("Automatic regrasp will run after the next drop.", live=True)
            return
        self._request_automatic_regrasp()

    def _request_automatic_regrasp(self) -> None:
        if (
            not self._automatic_regrasp_model.get_value_as_bool()
            or self._regrasp.active
            or not self._regrasp.released
        ):
            return
        if not omni.timeline.get_timeline_interface().is_playing():
            self._regrasp_pending = True
            self._ensure_physics()
            self._set_status(
                "Automatic regrasp will start when the timeline is playing.", live=True
            )
            return
        self._arm_regrasp(allow_clear=False)

    def _arm_regrasp(self, *, allow_clear: bool) -> None:
        if self._regrasp.active:
            return
        if not omni.timeline.get_timeline_interface().is_playing():
            self._set_status("Start the simulation timeline before regrasping.")
            return
        try:
            profile = self._profile if self._profile is not None else self._selected_profile()
        except ValueError as exc:
            self._set_status(str(exc))
            return
        frame = profile.grasp_frame.strip().rstrip("/").split("/")[-1]
        if profile.name != "Panda Hand" or frame != "panda_hand":
            self._set_status("Regrasp is only implemented for Panda Hand.")
            return
        robot_path = self._path_model.get_value_as_string().strip()
        object_path = self._object_path_model.get_value_as_string().strip()
        if not robot_path or not object_path:
            self._set_status("Enter the articulation path and the target object prim path.")
            return
        clear = False
        if allow_clear and not self._regrasp.released:
            try:
                clear = object_clear_of_hand(
                    _world_translation(object_path),
                    _world_translation(profile.grasp_frame_path(robot_path)),
                    profile.distance_threshold_m,
                )
            except Exception as exc:
                self._set_status(f"Regrasp stopped: {exc}")
                return
        self._regrasp_profile = profile
        self._regrasp_orientation = None
        self._regrasp_reported = ""
        message = self._regrasp.arm(object_clear=clear)
        if message is not None:
            self._regrasp_profile = None
            self._set_status(message)
            return
        self._ensure_physics()
        self._set_status("Regrasp armed. Waiting for the object to settle.", live=True)

    def _cancel_regrasp_motion(self) -> None:
        self._regrasp_pending = False
        self._regrasp.reset()
        self._regrasp_profile = None
        self._regrasp_orientation = None
        self._regrasp_reported = ""
        self._ik_solver = None

    def _ensure_physics(self) -> None:
        if self._physics_subscription is None:
            self._physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step
            )

    def _finish_regrasp(self, text: str, *, resume_watch: bool) -> None:
        self._cancel_regrasp_motion()
        if resume_watch and self._automatic_model.get_value_as_bool():
            self._start_automatic()
            self._set_status(text, live=True)
            return
        if self._automatic_model.get_value_as_bool():
            self._updating_checkbox = True
            self._automatic_model.set_value(False)
            self._updating_checkbox = False
        self._stop_automatic()
        self._set_status(text)

    def _step_regrasp(self) -> None:
        profile = self._regrasp_profile
        robot_path = self._path_model.get_value_as_string().strip()
        object_path = self._object_path_model.get_value_as_string().strip()
        hand_position, hand_orientation = self._hand_pose(robot_path)
        pinch_position, pinch_orientation = pinch_from_panda_hand(hand_position, hand_orientation)
        command = self._regrasp.step(
            dt_s=self._physics_dt,
            object_position=_world_translation(object_path),
            pinch_position=pinch_position,
            hand_position=hand_position,
            distance_threshold_m=profile.distance_threshold_m,
        )
        self._announce_regrasp(command)
        if command.phase == "done":
            resume = resume_drop_watch(
                automatic_regrasp=self._automatic_regrasp_model.get_value_as_bool(),
                drop_enabled=self._automatic_model.get_value_as_bool(),
            )
            text = (
                "Regrasp finished. Watching for the next pickup."
                if resume
                else "Regrasp finished."
            )
            self._finish_regrasp(text, resume_watch=resume)
            return
        if command.phase == "failed":
            self._finish_regrasp(command.reason, resume_watch=False)
            return
        if command.phase == "settle":
            inject_drop(IsaacArticulationDriver(robot_path), profile.as_adapter())
            return
        if self._regrasp_orientation is None:
            base_position, _base_orientation = get_world_pose(robot_path)
            obj = _world_translation(object_path)
            self._regrasp_orientation = grasp_pinch_orientation(
                pinch_orientation,
                (float(base_position[0]), float(base_position[1])),
                (float(obj[0]), float(obj[1])),
            )
        self._drive_pinch(robot_path, command)

    def _announce_regrasp(self, command: RegraspCommand) -> None:
        if command.phase == self._regrasp_reported:
            return
        self._regrasp_reported = command.phase
        text = {
            "settle": "Regrasp armed. Waiting for the object to settle.",
            "hover": "Regrasp: moving above the object.",
            "descend": "Regrasp: moving down to the object.",
            "close": "Regrasp: closing the gripper.",
            "lift": "Regrasp: lifting the object.",
        }.get(command.phase)
        if text is not None:
            self._set_status(text, live=True)

    def _hand_pose(self, robot_path: str) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        hand_path = self._regrasp_profile.grasp_frame_path(robot_path)
        position, orientation = get_world_pose(hand_path)
        hand = (float(position[0]), float(position[1]), float(position[2]))
        quat = (
            float(orientation[0]),
            float(orientation[1]),
            float(orientation[2]),
            float(orientation[3]),
        )
        return hand, quat

    def _drive_pinch(self, robot_path: str, command: RegraspCommand) -> None:
        articulation = self._ensure_articulation(robot_path)
        action, reached = self._panda_solver(articulation).compute_inverse_kinematics(
            target_position=np.asarray(command.target_position, dtype=np.float64),
            target_orientation=np.asarray(self._regrasp_orientation, dtype=np.float64),
            position_tolerance=0.01,
            orientation_tolerance=0.3,
        )
        finger_positions = (
            self._regrasp_profile.closed_positions
            if command.fingers_closed
            else self._regrasp_profile.open_positions
        )
        self._apply_arm_and_fingers(articulation, action if reached else None, finger_positions)

    def _panda_solver(self, articulation: SingleArticulation):
        if self._ik_solver is not None:
            return self._ik_solver
        import isaacsim.robot_motion.motion_generation.interface_config_loader as interface_config_loader
        from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import (
            ArticulationKinematicsSolver,
        )
        from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver

        config = interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
        if not config:
            raise RuntimeError("Isaac Sim has no Franka kinematics config")
        kinematics = LulaKinematicsSolver(**config)
        if "right_gripper" not in kinematics.get_all_frame_names():
            raise RuntimeError("Franka kinematics has no right_gripper frame")
        self._ik_solver = ArticulationKinematicsSolver(articulation, kinematics, "right_gripper")
        return self._ik_solver

    def _ensure_articulation(self, robot_path: str) -> SingleArticulation:
        if self._articulation is not None and self._articulation_path == robot_path:
            return self._articulation
        self._articulation = SingleArticulation(robot_path)
        self._articulation.initialize()
        self._articulation_path = robot_path
        self._ik_solver = None
        return self._articulation

    def _apply_arm_and_fingers(
        self,
        articulation: SingleArticulation,
        arm_action: ArticulationAction | None,
        finger_positions: tuple[float, ...],
    ) -> None:
        finger_indices = resolve_joint_indices(
            tuple(articulation.dof_names), self._regrasp_profile.joint_names
        )
        if arm_action is None:
            articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.asarray(finger_positions, dtype=np.float32),
                    joint_indices=np.asarray(finger_indices, dtype=np.int32),
                )
            )
            return
        positions = np.concatenate(
            [
                np.asarray(arm_action.joint_positions, dtype=np.float32).reshape(-1),
                np.asarray(finger_positions, dtype=np.float32).reshape(-1),
            ]
        )
        indices = np.concatenate(
            [
                np.asarray(arm_action.joint_indices, dtype=np.int32).reshape(-1),
                np.asarray(finger_indices, dtype=np.int32).reshape(-1),
            ]
        )
        articulation.apply_action(ArticulationAction(joint_positions=positions, joint_indices=indices))
