# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| [README.md](README.md) | Indexes this folder and its maintained subfolders. | None | Developers |
| [export_to_mat.py](export_to_mat.py) | Provides the export to mat Python module. | Python standard library; `geometry_msgs`; `numpy`; `rospy`; 3 other declared imports/includes | None |
| [export_to_svg.py](export_to_svg.py) | Exports planner feedback scenes to SVG. | Python standard library; `geometry_msgs`; `roslib`; `rospy`; 2 other declared imports/includes | None |
| [publish_dynamic_obstacle.py](publish_dynamic_obstacle.py) | Provides the publish dynamic obstacle Python module. | Python standard library; `costmap_converter`; `geometry_msgs`; `rospy`; 1 other declared imports/includes | None |
| [publish_test_obstacles.py](publish_test_obstacles.py) | Provides the publish test obstacles Python module. | Python standard library; `costmap_converter`; `geometry_msgs`; `rospy` | None |
| [publish_viapoints.py](publish_viapoints.py) | Provides the publish viapoints Python module. | Python standard library; `geometry_msgs`; `nav_msgs`; `rospy` | [CHANGELOG.rst](../CHANGELOG.rst) |
| [visualize_velocity_profile.py](visualize_velocity_profile.py) | Provides the visualize velocity profile Python module. | Python standard library; `geometry_msgs`; `matplotlib`; `numpy`; 2 other declared imports/includes | None |
