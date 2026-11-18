# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| [CHANGELOG.rst](CHANGELOG.rst) | Records release changes and contributor history. | None | None |
| [CMakeLists.txt](CMakeLists.txt) | Configures this folder's build, installation or validation targets. | CMake | None |
| [LICENSE](LICENSE) | Preserves the applicable license and contributor attribution. | None | None |
| [README.md](README.md) | Indexes this folder and its maintained subfolders. | None | Developers |
| [cfg/](cfg/README.md) | Indexes this maintained subfolder. | None | Developers |
| [cmake_modules/](cmake_modules/README.md) | Indexes this maintained subfolder. | None | Developers |
| [include/](include/README.md) | Indexes this maintained subfolder. | None | Developers |
| [package.xml](package.xml) | Declares teb_local_planner's ROS package identity and dependencies. | `ament_cmake`; `builtin_interfaces`; `costmap_converter_msgs`; `costmap_converter`; 20 other declared imports/includes | None |
| [params/](params/README.md) | Indexes this maintained subfolder. | None | Developers |
| [scripts/](scripts/README.md) | Indexes this maintained subfolder. | None | Developers |
| [src/](src/README.md) | Indexes this maintained subfolder. | None | Developers |
| [teb_local_planner_plugin.xml](teb_local_planner_plugin.xml) | Registers the declared plugin classes and libraries. | None | [teb_local_planner/CMakeLists.txt](CMakeLists.txt); [teb_local_planner/package.xml](package.xml) |
