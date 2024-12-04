#!/usr/bin/env bash

STL_DIR="/root/maps"
OUTPUT_DIR="/root/output"

mkdir -p "$OUTPUT_DIR"

for stl_file in "$STL_DIR"/*.stl; do
    [ -e "$stl_file" ] || continue

    base_name=$(basename "$stl_file" .stl)
    echo "Processing $base_name.stl..."

    # The corresponding coordinates file
    #format(with header): start_x,start_y,start_z, start_vx, start_vy, start_vz, goal_x,goal_y,goal_z
                            #38.0,13.0,1.3,0,0,0,28.5,14.0,1.3
    coords_file="${STL_DIR}/${base_name}_coordinates.csv"
    if [ ! -f "$coords_file" ]; then
        echo "Coordinates file $coords_file not found!"
        continue
    fi

    # Parse coordinates from the CSV file
    # Assuming CSV has a header, we can skip it with 'tail -n 1'
    # If no header, remove 'tail -n 1'
    IFS=',' read -r HEADER < "$coords_file"
    IFS=',' read -r START_X START_Y START_Z START_VX START_VY START_VZ GOAL_X GOAL_Y GOAL_Z < <(tail -n 1 "$coords_file")

    # 1. Convert STL to voxel map
    roslaunch mpl_test_node mesh_to_map.launch stl_file:="$stl_file" &
    MESH_TO_MAP_PID=$!
    sleep 15
    until rostopic echo /voxel_map -n 1 >/dev/null 2>&1; do
        sleep 2
    done

    # 2. Record voxel_map
    BAG_NAME="${base_name}.bag"
    rosbag record -O "$OUTPUT_DIR/$BAG_NAME" /voxel_map -d 15  # record for 5 seconds
    # The '-d 5' records for 5 seconds then stops automatically
    # Adjust timing as needed based on your environment

    # Terminate mesh_to_map after recording finishes
   kill $MESH_TO_MAP_PID
    wait $MESH_TO_MAP_PID 2>/dev/null

    # 3. Run ellipsoid planner using the generated .bag and coordinates
    roslaunch mpl_test_node test.launch bag_file:="$OUTPUT_DIR/$BAG_NAME" \
      start_x:=$START_X start_y:=$START_Y start_z:=$START_Z \
      start_vx:=$START_VX start_vy:=$START_VY start_vz:=$START_VZ \
      goal_x:=$GOAL_X goal_y:=$GOAL_Y goal_z:=$GOAL_Z

    # Rename trajectory.csv to match the base name
    #time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z
    if [ -f ~/.ros/trajectory.csv ]; then
        mv ~/.ros/trajectory.csv "$OUTPUT_DIR/${base_name}_traj.csv"
    else
        echo "trajectory.csv not found!"
    fi

    echo "Finished processing $base_name.stl"
done
