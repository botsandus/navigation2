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
Scenario discovery.

Find YAML scenario files from two sources.

Sources (checked in order):

1. Built-in scenarios shipped with ``nav2_nav_tester``.
2. Paths listed in the ``NAV_TEST_SCENARIO_PATH`` environment variable
   (colon-separated list of directories or individual ``.yaml`` files).

For ``colcon test`` integration, prefer ``pytest.mark.parametrize`` with a
glob over the scenario directory rather than relying on runtime discovery.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def _builtin_scenarios() -> list[Path]:
    """Return built-in scenario YAML files shipped with this package."""
    scenario_dir = Path(
        get_package_share_directory('nav2_nav_tester')
    ) / 'scenarios'
    if scenario_dir.is_dir():
        return sorted(scenario_dir.glob('*.yaml'))
    return []


def _env_scenarios() -> list[Path]:
    """Find scenarios from the NAV_TEST_SCENARIO_PATH env variable."""
    env = os.environ.get('NAV_TEST_SCENARIO_PATH', '')
    if not env:
        return []

    paths: list[Path] = []
    for entry in env.split(':'):
        entry = entry.strip()
        if not entry:
            continue
        p = Path(entry)
        if p.is_file() and p.suffix in ('.yaml', '.yml'):
            paths.append(p)
        elif p.is_dir():
            paths.extend(sorted(p.glob('*.yaml')))
    return paths


def discover_scenarios(include_builtin: bool = True) -> list[Path]:
    """Discover all available scenario YAML files."""
    seen: set[Path] = set()
    result: list[Path] = []

    sources = []
    if include_builtin:
        sources.append(_builtin_scenarios())
    sources.append(_env_scenarios())

    for paths in sources:
        for p in paths:
            resolved = p.resolve()
            if resolved not in seen:
                seen.add(resolved)
                result.append(p)

    return result
