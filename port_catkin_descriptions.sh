#!/usr/bin/env bash
# Port ROS 1 catkin description packages in src/ to ROS 2 ament_cmake.
# Non-description catkin packages are skipped with COLCON_IGNORE.
set -euo pipefail
cd "$(dirname "$0")"

mapfile -t PKGS < <(colcon list 2>/dev/null | awk '$3=="(ros.catkin)"{print $1" "$2}')
if [[ ${#PKGS[@]} -eq 0 ]]; then
  echo "No catkin packages found in src/. Nothing to do."
  exit 0
fi

for entry in "${PKGS[@]}"; do
  name="${entry%% *}"
  dir="${entry#* }"

  if [[ -d "$dir/urdf" || -d "$dir/meshes" ]]; then
    echo "[port]   $name  ($dir)"
    [[ -f "$dir/package.xml.ros1" ]] || cp "$dir/package.xml" "$dir/package.xml.ros1"
    if [[ -f "$dir/CMakeLists.txt" && ! -f "$dir/CMakeLists.txt.ros1" ]]; then
      cp "$dir/CMakeLists.txt" "$dir/CMakeLists.txt.ros1"
    fi

    cat > "$dir/package.xml" <<EOF
<?xml version="1.0"?>
<package format="3">
  <name>${name}</name>
  <version>0.1.0</version>
  <description>${name} (ROS 2 port of the ROS 1 description package)</description>
  <maintainer email="tatha@example.com">Tathagata Chowdhury</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

    cat > "$dir/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.8)
project(${name})

find_package(ament_cmake REQUIRED)

foreach(d urdf meshes params config launch)
  if(EXISTS \${CMAKE_CURRENT_SOURCE_DIR}/\${d})
    install(DIRECTORY \${d} DESTINATION share/\${PROJECT_NAME})
  endif()
endforeach()

ament_package()
EOF
  else
    echo "[ignore] $name  ($dir)  - not a description package"
    touch "$dir/COLCON_IGNORE"
  fi
done

echo
echo "Packages referenced by xacro/URDF files in src/:"
grep -rhoE '\$\(find [A-Za-z0-9_]+\)|package://[A-Za-z0-9_]+' src --include='*.xacro' --include='*.urdf' 2>/dev/null \
  | sed -E 's/^\$\(find //; s/\)$//; s#^package://##' | sort -u
