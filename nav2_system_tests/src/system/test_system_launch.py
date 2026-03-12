#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
# Copyright (c) 2020 Florian Gramss
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

import os
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch_testing.legacy import LaunchTestService
from nav2_nav_tester.backends.gazebo import generate_nav_actions


def generate_launch_description() -> LaunchDescription:
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    params_file = os.path.join(launch_dir, 'nav2_system_params.yaml')

    actions = generate_nav_actions(
        params_file=params_file,
        bt_xml=os.getenv('BT_NAVIGATOR_XML', ''),
        controller_plugin=os.getenv('CONTROLLER', ''),
        planner_plugin=os.getenv('PLANNER', ''),
        use_astar=os.getenv('ASTAR') == 'True',
        groot_monitoring=os.getenv('GROOT_MONITORING') == 'True',
        inflation_layer_plugin=os.getenv('INFLATION_LAYER', ''),
    )
    return LaunchDescription(actions)


def main(argv: list[str] = sys.argv[1:]):  # type: ignore[no-untyped-def]
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[
            os.path.join(os.getenv('TEST_DIR', ''), os.getenv('TESTER', '')),
            '-r',
            '-2.0',
            '-0.5',
            '0.0',
            '2.0',
            '-e',
            'True',
        ],
        name='tester_node',
        output='screen',
    )

    lts = LaunchTestService()  # type: ignore[no-untyped-call]
    lts.add_test_action(ld, test1_action)  # type: ignore[no-untyped-call]
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return_code = lts.run(ls)  # type: ignore[no-untyped-call]
    return return_code


if __name__ == '__main__':
    sys.exit(main())
