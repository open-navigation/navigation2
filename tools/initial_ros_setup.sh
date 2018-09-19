#!/bin/bash

ENABLE_ROS1=true
ENABLE_BUILD=true
ENABLE_ROS2=true
if [ "$ROS1_DISTRO" = "" ]; then
	export ROS1_DISTRO=kinetic
fi
if [ "$ROS2_DISTRO" = "" ]; then
	export ROS2_DISTRO=bouncy
fi
if test "$ROS1_DISTRO" != "kinetic" && test "$ROS1_DISTRO" != "melodic" ; then
	echo "ROS1_DISTRO variable must be set to either kinetic or melodic"
	exit 1
fi
if [ "$ROS2_DISTRO" != "bouncy" ]; then
	echo "ROS2_DISTRO variable must be set to bouncy"
	exit 1
fi

for opt in "$@" ; do
	case $opt in
		--no-ros1)
			ENABLE_ROS1=false
			shift
		;;
		--no-ros2)
			ENABLE_ROS2=false
			shift
		;;
		--download-only)
			ENABLE_BUILD=false
			shift
		;;
		*)
			echo "Invalid option: $opt"
			echo "Valid options:"
			echo "--no-ros1       Disables downloading and building ROS1 dependencies and ROS1 bridge"
			echo "--no-ros2       Uses the binary distribution of ROS2 bouncy"
			echo "--download-only Skips the build step and only downloads the code"
			exit 1
		;;
	esac
done

set -e
CHECKPOINT_FILES=''

CWD=`pwd`
return_to_root_dir() {
	cd $CWD
}

download_navstack() {
	echo "Downloading the ROS 2 navstack"
	if [ -f "custom_nav2.repos" ]; then #override default location for testing
		vcs-import < custom_nav2.repos
	else
		git clone https://github.com/ros-planning/navigation2.git
	fi
	return_to_root_dir
}

download_ros2() {
	echo "Downloading ROS 2 Release Latest"
	mkdir -p ros2_ws/src
	cd ros2_ws
	wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
	vcs import src < ros2.repos
	return_to_root_dir
}

download_ros2_dependencies() {
	echo "Downloading the dependencies workspace"
	mkdir -p navstack_dependencies_ws/src
	cd navstack_dependencies_ws
	vcs import src < ${CWD}/navigation2/tools/ros2_dependencies.repos
	return_to_root_dir
}

download_ros1_dependencies() {
	echo "Downloading the ROS 1 dependencies workspace"
	mkdir -p ros1_dependencies_ws/src
	cd ros1_dependencies_ws
	vcs import src < ${CWD}/navigation2/tools/ros1_dependencies.repos.${ROS1_DISTRO}
	return_to_root_dir
}

checkpoint() {
	local CHECKPOINT_FILE_NAME=.INITIAL_SETUP_$1
	CHECKPOINT_FILES="${CHECKPOINT_FILES} ${CHECKPOINT_FILE_NAME}"
	if [ ! -f ${CHECKPOINT_FILE_NAME} ]; then
		$1
		touch ${CHECKPOINT_FILE_NAME}
	fi
}

download_all() {
	checkpoint download_navstack
	checkpoint download_ros2_dependencies
	if [ "$ENABLE_ROS2" = true ]; then
		checkpoint download_ros2
	fi
	if [ "$ENABLE_ROS1" = true ]; then
		checkpoint download_ros1_dependencies
	fi
}

echo "This script will download the ROS 2 latest release workspace, the"
echo "dependencies workspace and the ros_navstack_port workspace to the"
echo "current directory and then build them all. There should be no ROS"
echo "environment variables set at this time."
echo
echo "The current directory is $CWD"
echo
echo "Are you sure you want to continue? [yN]"
read -r REPLY
echo
if [ "$REPLY" = "y" ]; then
	download_all
	if [ "$ENABLE_BUILD" = true ]; then
		$CWD/navigation2/tools/build_all.sh
	fi

	cd ${CWD}
	rm ${CHECKPOINT_FILES}
	echo
	echo "Everything downloaded and built successfully."
	echo "To use the navstack source the setup.bash in the install folder"
	echo
	echo "> source navigation2/install/setup.bash"
	echo
	echo "To build the navstack you can either"
	echo "1. Run 'colcon build --symlink-install' from the navigation2 folder"
	echo "2. or run 'make' from navigation2/build/<project> folder"
fi
