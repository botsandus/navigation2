# Copyright (c) 2026 nav2_nav_tester contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Scenario data model and YAML loader.

Scenario YAML format::

    scenario:
      name: straight_line
      description: "Robot navigates in a straight line forward"
      timeout_sec: 30.0
      start:
        x: -2.0
        y: -0.5
        yaw: 0.0
      goal:
        x: 0.0
        y: -0.5
        yaw: 0.0
      # Optional overrides (omit or leave empty to use defaults)
      behavior_tree: ""
      params_file: ""
      map: ""
"""

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass
class Scenario:
    """A navigation test scenario loaded from YAML."""

    name: str
    description: str
    timeout_sec: float

    start_x: float
    start_y: float
    start_yaw: float

    goal_x: float
    goal_y: float
    goal_yaw: float

    # Optional overrides — empty string means "use backend default"
    behavior_tree: str = ''
    params_file: str = ''
    map_yaml: str = ''

    # Path to the YAML file this was loaded from (for diagnostics)
    source_file: str = ''


def load_scenario(path: str | Path) -> Scenario:
    """Load a single scenario from a YAML file."""
    path = Path(path)
    with open(path) as f:
        data = yaml.safe_load(f)

    s = data['scenario']
    start = s['start']
    goal = s['goal']

    return Scenario(
        name=s['name'],
        description=s.get('description', ''),
        timeout_sec=float(s.get('timeout_sec', 60.0)),
        start_x=float(start['x']),
        start_y=float(start['y']),
        start_yaw=float(start['yaw']),
        goal_x=float(goal['x']),
        goal_y=float(goal['y']),
        goal_yaw=float(goal['yaw']),
        behavior_tree=s.get('behavior_tree', '') or '',
        params_file=s.get('params_file', '') or '',
        map_yaml=s.get('map', '') or '',
        source_file=str(path),
    )


def load_scenarios(paths: list[str | Path]) -> list[Scenario]:
    """Load multiple scenarios from a list of YAML file paths."""
    return [load_scenario(p) for p in paths]
