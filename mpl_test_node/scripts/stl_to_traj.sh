#!/usr/bin/env bash

STL_DIR="/root/maps"  # Directory containing STL files
OUTPUT_DIR="/root/output"  # Directory to save output files

mkdir -p "$OUTPUT_DIR"

for stl_file in "$STL_DIR"/*.stl; do
    [ -e "$stl_file" ] || continue

    base_name=$(basename "$stl_file" .stl)
    echo "Processing $base_name.stl..."

    # The corresponding coordinates file
    coords_file="${STL_DIR}/${base_name}_coordinates.csv"
    if [ ! -f "$coords_file" ]; then
        echo "Coordinates file $coords_file not found!"
        continue
    fi

    # Parse coordinates from the CSV file
    IFS=',' read -r HEADER < "$coords_file"
    IFS=',' read -r START_X START_Y START_Z START_VX START_VY START_VZ GOAL_X GOAL_Y GOAL_Z < <(tail -n 1 "$coords_file")

    # 1. Convert STL to voxel map
    roslaunch mpl_test_node mesh_to_map.launch stl_file:="$stl_file" &
    MESH_TO_MAP_PID=$!
    sleep 15

    # Check for the topic before recording
    until rostopic list | grep -q "/voxel_map"; do
        echo "Waiting for /voxel_map topic..."
        sleep 2
    done

    # 2. Record voxel_map
    BAG_NAME="${base_name}.bag"
    timeout 15s rosbag record -O "$OUTPUT_DIR/$BAG_NAME" /cloud /voxel_map

    # Terminate mesh_to_map after recording finishes
    kill $MESH_TO_MAP_PID
    wait $MESH_TO_MAP_PID 2>/dev/null

    # Verify the bag file
    if [ ! -f "$OUTPUT_DIR/$BAG_NAME" ]; then
        echo "Bag file $BAG_NAME not created!"
        continue
    fi

    # 3. Run ellipsoid planner using the generated .bag and coordinates
    roslaunch /root/catkin_ws/src/mpl_ros/mpl_test_node/launch/ellipsoid_planner_node/test.launch \
      bag_file:="$OUTPUT_DIR/$BAG_NAME" \
      start_x:=$START_X start_y:=$START_Y start_z:=$START_Z \
      start_vx:=$START_VX start_vy:=$START_VY start_vz:=$START_VZ \
      goal_x:=$GOAL_X goal_y:=$GOAL_Y goal_z:=$GOAL_Z

    # Move trajectory.csv to OUTPUT_DIR with a unique name
    TRAJ_CSV="${OUTPUT_DIR}/${base_name}_traj.csv"
    if [ -f ~/.ros/trajectory.csv ]; then
        mv ~/.ros/trajectory.csv "$TRAJ_CSV"
        echo "Trajectory saved to $TRAJ_CSV"
    else
        echo "trajectory.csv not found!"
    fi

    echo "Finished processing $base_name.stl"
done
