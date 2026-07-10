# nav_path_recorder

Lifecycle node that records the path travelled by the robot. It collects `/loc/odom` poses into a `nav_util::Path2D` buffer and saves them as an Agri `work_performed` JSON file consumable by [`nav_replay`](../nav_replay/README.md).

## Overview

On each `/loc/odom` message (when active):

1. Append the `(x, y)` position to the path (minimum spacing `0.09` m via `Path2D`).
2. Store the rounded linear speed on the last point (magnitude at least `0.1` m/s).
3. Set `working_zone` to `false` for every point (`tools_state_` is never updated).

A 10 s timer saves the current path to `path_performed.json` while recording is active.

On deactivation, recording stops. The path is smoothed with `filtering(1)`, saved to `path_performed.json`, then cleared. The file uses file type `work_performed`, stores the WGS84 anchor (`lat`, `lon`, `alt`) resolved at activation, and per-point position, speed, and `working_zone`.

At configure, the node reads `vehicle_id` and starts the autosave timer. Configure fails if `vehicle_id` is empty. At activate, the WGS84 anchor is resolved from TF (`lookup_tf_to_wgs84`). Activation fails if the TF lookup fails.

## Parameters

| Parameter    | Default | Description                                            |
| ------------ | ------- | ------------------------------------------------------ |
| `vehicle_id` | `""`    | Vehicle identifier stored in the saved file (required) |

## Topics

| Topic       | Type                    | Direction | Description                                        |
| ----------- | ----------------------- | --------- | -------------------------------------------------- |
| `/loc/odom` | `nav_msgs/msg/Odometry` | In        | Robot pose and linear speed used to build the path |

The `/loc/odom` subscription is created in the active lifecycle state only.
