#!/bin/bash

# Usage:
# Convert ros2 bags into ros1 bags:
#   kalibr path_to_calib_dir path_to_ros2_bags convert
#
# Calibrate cameras from converted rosbags
#   kalibr path_to_calib_dir path_to_ros2_bags calib

 # Where the calibration realted files are located, like calibration board
CALIBRATION_DIR=$1
# Where the ros2 bags are locate in order for them to be converted into ros1 bags.
ROS2_BAGS_DIR=$2 

# In case kalibr docker image is not avaialble, script will clone and build it
KALIBR_REPO_DIR="$CALIBRATION_DIR/kalibr" 
KALIBR_REPO_URL="https://github.com/ethz-asl/kalibr.git"

# Path where converted ros1 bags will be stored
ROS1_BAGS_DEST="$CALIBRATION_DIR/calib/bags"

# Whether to use python venv to install necessary PIP packages, like rosbags
USE_PYTHON_VENV=true

# Sets how many jobs can run in parallel. This is used for example to set how many
# rosbags are converted at once and calibrations too.
MAX_PARALLEL_JOBS=4

TARGET="calib_target.yaml"

# ---------------
# -    Logger   -
# ---------------

COLOR_GREEN='\033[1;32m'
COLOR_YELLOW='\033[1;33m'
COLOR_RED='\033[0;31m'
COLOR_OFF='\033[0m'

log_info(){
    echo -e "${COLOR_GREEN}INFO -${COLOR_OFF} $1 "
}
log_warn(){
    echo -e "${COLOR_YELLOW}WARNING -${COLOR_OFF} $1"
}
log_error(){
    echo -e "${COLOR_RED}ERROR -${COLOR_OFF} $1"
}

# ---------------
# -    Kalibr   -
# ---------------

is_kalibr_image_available(){
  docker images | grep -q kalibr
}

clone_kalibr(){
  if [[ ! -d $KALIBR_REPO_DIR ]]; then
    log_info "Cloning kalibr repo into $KALIBR_REPO_DIR"
    git clone $KALIBR_REPO_URL $KALIBR_REPO_DIR
  fi
}

build_kalibr_image(){
  log_info "Building kalibr docker image with name kalibr"
  pushd $KALIBR_REPO_DIR
  docker build -t kalibr -f Dockerfile_ros1_20_04 .
  popd 
}

enable_docker_gui_support(){
  xhost +local:docker
}

run_camera_calibration(){
  log_info "Starting camera calibration"
  enable_docker_gui_support

  local -a pids=()
  for file in "$ROS2_BAGS_DIR"/*; do
    local basename="$(basename $file)"
    local filename="${basename%.*}"
    local camera_name="$(echo $filename | cut -d'_' -f2-)"

    log_info "Calibrating for $camera_name Camera"

    docker run --rm \
    -e DISPLAY="$DISPLAY" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$CALIBRATION_DIR:/data" \
    --entrypoint bash kalibr \
    -c "source /catkin_ws/devel/setup.bash \
        &&  rosrun kalibr kalibr_calibrate_cameras \
      --bag /data/calib/bags/$filename.bag \
      --target /data/$TARGET \
      --models pinhole-radtan \
      --topics /camera/$camera_name/image_raw \
      --dont-show-report" >/dev/null&


    pids+=($!)

    # Limit number of parallel jobs
    if (( ${#pids[@]} >= $MAX_PARALLEL_JOBS )); then
      wait -n  # Wait for any one to finish
      # Remove finished PIDs from array
      for i in "${!pids[@]}"; do
        if ! kill -0 "${pids[i]}" 2>/dev/null; then
          unset 'pids[i]'
        fi
      done
    fi
  done
}

run_rs_camera_calibration(){
  log_info "Starting rolling-shutter camera calibration"
}


# ---------------
# -   Rosbags   -
# ---------------

create_venv(){
  if [[ ! -d ".venv" ]]; then
    log_info "Creating python VENV"
    python3 -m venv .venv
  fi
}

activate_venv(){
  log_info "Activating python VENV"
  source .venv/bin/activate
}

deactivate_venv(){
  log_info "Deactivating python VENV"
  deactivate
}

is_rosbags_pip_available(){
  pip list | grep -q rosbags
}

convert_ros2bags(){
  local base_command="${2:-rosbags-convert}"

  if [[ ! -d $ROS1_BAGS_DEST ]]; then
    log_info "Creating directory for converted ros1 bags"
    mkdir -p $ROS1_BAGS_DEST
  fi

  if [[ ! -d "$ROS2_BAGS_DIR" ]]; then
    echo "Error: '$ROS2_BAGS_DIR' is not a directory."
    return 1
  fi

  if ! [[ "$MAX_PARALLEL_JOBS" =~ ^[0-9]+$ ]] || (( $MAX_PARALLEL_JOBS < 1 )); then
    echo "Error: invalid number of parallel jobs: $MAX_PARALLEL_JOBS"
    return 1
  fi

  local -a pids=()
  for file in "$ROS2_BAGS_DIR"/*; do
    local ros2_bag_name="$(basename  $file)"
    local ros1_bag_dest="$ROS1_BAGS_DEST/$ros2_bag_name.bag"
    log_info "Converting ros2 bag $ros2_bag_name to $ros1_bag_dest "
    bash -c "$base_command --src $file --dst $ros1_bag_dest" &

    pids+=($!)

    # Limit number of parallel jobs
    if (( ${#pids[@]} >= $MAX_PARALLEL_JOBS )); then
      wait -n  # Wait for any one to finish
      # Remove finished PIDs from array
      for i in "${!pids[@]}"; do
        if ! kill -0 "${pids[i]}" 2>/dev/null; then
          unset 'pids[i]'
        fi
      done
    fi
  done

  # Wait for all remaining jobs
  wait
}


# ------------------------------------------------------------

if [[ ! -d $CALIBRATION_DIR ]]; then
  log_info "Creating directory at $CALIBRATION_DIR"
  mkdir -p $CALIBRATION_DIR
fi

pushd $CALIBRATION_DIR >/dev/null

if [[ ! -f $CALIBRATION_DIR/$TARGET ]] ; then
  log_error "Calibration target $TARGET not found in $CALIBRATION_DIR"
  exit 1
fi

if ! is_kalibr_image_available ; then
  log_info "kalibr docker image is not available."
  clone_kalibr 
  build_kalibr_image 
fi

if $USE_PYTHON_VENV ; then
  create_venv
  activate_venv
fi

if ! is_rosbags_pip_available ; then
  log_info "Installing rosbags==0.10.9 PIP package"
  pip install rosbags==0.10.9 >/dev/null
fi

case "$3" in
convert)
    echo "Argument 1 used"
  convert_ros2bags
    ;;
calib)
  run_camera_calibration 
    ;;
*)
    echo "Unknown command: $1."
    ;;
esac

# END

if $USE_PYTHON_VENV ; then
  deactivate_venv 
fi

popd >/dev/null
