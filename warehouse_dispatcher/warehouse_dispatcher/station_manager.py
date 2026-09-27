"""Warehouse station definitions and validation utilities."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi
from typing import Dict, Iterable, Tuple


@dataclass(frozen=True)
class StationPose:
    """Immutable 2-D navigation pose in the map frame."""

    name: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class WarehouseOrder:
    """One pickup-to-drop mission."""

    order_id: str
    pickup: str
    drop_zone: str


ARENA_HALF_WIDTH = 6.0
ARENA_HALF_HEIGHT = 5.0
ROBOT_RADIUS = 0.20

HOME_BASE = StationPose("HOME", 0.0, -3.5, pi / 2.0)

PICKUP_STATIONS: Dict[str, StationPose] = {
    "A": StationPose("A", -4.0, 4.0, pi / 2.0),
    "B": StationPose("B", 0.0, 4.0, pi / 2.0),
    "C": StationPose("C", 4.0, 4.0, pi / 2.0),
}

DROP_ZONES: Dict[str, StationPose] = {
    "1": StationPose("1", -4.0, -4.0, -pi / 2.0),
    "2": StationPose("2", 0.0, -4.0, -pi / 2.0),
    "3": StationPose("3", 4.0, -4.0, -pi / 2.0),
}

DEFAULT_ORDERS: Tuple[WarehouseOrder, ...] = (
    WarehouseOrder("ORDER-001", "A", "1"),
    WarehouseOrder("ORDER-002", "B", "2"),
    WarehouseOrder("ORDER-003", "C", "3"),
)



def validate_station_pose(pose: StationPose) -> None:
    """Validate finite coordinates and keep the robot center inside the room."""
    if not pose.name.strip():
        raise ValueError("Station name must not be empty")
    if not all(isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
        raise ValueError(f"Station {pose.name!r} contains a non-finite pose value")
    if abs(pose.x) > ARENA_HALF_WIDTH - ROBOT_RADIUS:
        raise ValueError(f"Station {pose.name!r} x={pose.x} is outside the safe arena")
    if abs(pose.y) > ARENA_HALF_HEIGHT - ROBOT_RADIUS:
        raise ValueError(f"Station {pose.name!r} y={pose.y} is outside the safe arena")


def validate_layout() -> None:
    """Validate all named warehouse poses and reject accidental duplicates."""
    poses = (HOME_BASE, *PICKUP_STATIONS.values(), *DROP_ZONES.values())
    seen_names = set()
    for pose in poses:
        validate_station_pose(pose)
        if pose.name in seen_names:
            raise ValueError(f"Duplicate station name: {pose.name!r}")
        seen_names.add(pose.name)

def get_pickup(name: str) -> StationPose:
    """Return a validated pickup station pose."""
    key = str(name).strip().upper()
    if key not in PICKUP_STATIONS:
        raise ValueError(
            f"Unknown pickup station '{name}'. Valid stations: "
            f"{', '.join(sorted(PICKUP_STATIONS))}"
        )
    return PICKUP_STATIONS[key]


def get_drop_zone(name: str) -> StationPose:
    """Return a validated drop-zone pose."""
    key = str(name).strip()
    if key not in DROP_ZONES:
        raise ValueError(
            f"Unknown drop zone '{name}'. Valid zones: "
            f"{', '.join(sorted(DROP_ZONES))}"
        )
    return DROP_ZONES[key]


def validate_order(order: WarehouseOrder) -> None:
    """Raise ValueError when an order references an unknown station."""
    if not order.order_id.strip():
        raise ValueError("Order ID must not be empty")
    get_pickup(order.pickup)
    get_drop_zone(order.drop_zone)


def validate_orders(orders: Iterable[WarehouseOrder]) -> Tuple[WarehouseOrder, ...]:
    """Validate and normalize an iterable of orders."""
    normalized = tuple(orders)
    if not normalized:
        raise ValueError("At least one warehouse order is required")
    for order in normalized:
        validate_order(order)
    return normalized


# Fail fast during startup if coordinates are edited into an invalid layout.
validate_layout()
