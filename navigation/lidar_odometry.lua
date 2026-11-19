-- Local lidar odometry only. AMCL owns map->odom for a standalone YAML map.
include "lidar_2d.lua"
options.map_frame = "odom"
options.provide_odom_frame = false
options.published_frame = "base_link"
-- Disable loop-closure corrections in the continuous odometry frame.
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0
return options
