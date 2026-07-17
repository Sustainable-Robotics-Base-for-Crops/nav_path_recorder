# nav_path_recorder

Lifecycle node that records the path travelled by the robot. It collects `/loc/odom` poses into a `nav_util::Path2D` buffer and saves them as an Agri JSON file with `file_type: work_performed` (`path_performed.json`). That file is not replayed directly by `nav_replay` (which expects `file_type: mission_order`).

## Overview

On each `/loc/odom` message (when active):

1. Append the `(x, y)` position to the path (minimum spacing `0.09` m via `Path2D`).
2. Store the rounded linear speed on the last point (magnitude forced to at least `0.1` m/s).
3. Set `working_zone` to `false` for every point.

A 10 s timer saves the current path to `path_performed.json` while the node is configured.

### Saved file

The JSON (Agri format version 3) contains:

| Field                    | Value                              |
| ------------------------ | ---------------------------------- |
| `file_type`              | `work_performed`                   |
| `vehicle_id`             | Parameter `vehicle_id`             |
| `origin`                 | WGS84 anchor from TF at activation |
| `points[0].section_type` | `row_path`                         |
| `points[0].columns`      | `x`, `y`, `speed`, `working_zone`  |

## Parameters

| Parameter    | Default | Description                                            |
| ------------ | ------- | ------------------------------------------------------ |
| `vehicle_id` | `""`    | Vehicle identifier stored in the saved file (required) |

## Topics

| Topic       | Type                    | Direction | Description                                        |
| ----------- | ----------------------- | --------- | -------------------------------------------------- |
| `/loc/odom` | `nav_msgs/msg/Odometry` | In        | Robot pose and linear speed used to build the path |

The `/loc/odom` subscription is created only in the active lifecycle state.
