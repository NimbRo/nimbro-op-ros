#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

ROOT="$DIR"
while [[ ! -e "$ROOT/src/CMakeLists.txt" ]]; do
	ROOT=$(dirname "$ROOT")
	if [[ "$ROOT" == "/" ]]; then
		echo "Warning: Could not find ROS catkin workspace"
		return
	fi
done	

INSTALLPATH="/nimbro"
BOT=xs2.local

. "$ROOT/devel/setup.bash"
export ROSCONSOLE_FORMAT='[${severity}][${node}->${function}]: ${message}'

# Enable the UDP hinter per default if running roslaunch or rosrun
REAL_ROSLAUNCH=$(which roslaunch)
REAL_ROSRUN=$(which rosrun)

function enable_udp() {
	if [[ -e "$ROOT/devel/lib/libudp_hinter.so" ]]; then
		echo "env.sh: Enabling UDP hinter library..."
		export LD_PRELOAD="$ROOT/devel/lib/libudp_hinter.so"
	fi
}

# If you Ctrl+C a process/script that was run with exec_with_udp before it had
# a chance to restore LD_PRELOAD to what it was again, then you can use this
# function to manually unset LD_PRELOAD and disable the UDP hinter library again.
function disable_udp() {
	if [[ $LD_PRELOAD == "$ROOT/devel/lib/libudp_hinter.so" ]]; then
		echo "env.sh: Disabling UDP hinter library..."
		unset LD_PRELOAD
	fi
}

function exec_with_udp() {
	SAVED_LD_PRELOAD="$LD_PRELOAD"
	enable_udp
	$*
	export LD_PRELOAD="$SAVED_LD_PRELOAD"
}

function roslaunch() {
	exec_with_udp "$REAL_ROSLAUNCH" $*
}

function rosrun() {
	exec_with_udp "$REAL_ROSRUN" $*
}

function pullgit() {
	if ! git pull --rebase; then
		echo "---"
		read -n1 -p "Did the pull get refused because of unstaged changes [y/N]?" response
		echo
		if [[ $response == "y" || $response == "Y" ]]; then
			echo "Yes: Ok, I'm temporarily stashing away those changes..."
			git status
			git stash save "Changes stashed by nimbro pull to allow a rebase" && {
				git pull --rebase
				git stash pop || echo "Couldn't pop the stashed changes! Please check 'git stash list'..."
				echo "The stashed changes have been reapplied to the working directory!"
			}
		else
			echo "No: Ok, then please resolve the problem and try again."
		fi
	fi
}

# Use Avahi for name resolution
export ROS_HOSTNAME=`hostname`.local

function nimbro() {
	cd $ROOT
	case $1 in
		make)
			catkin_make $2 -DCMAKE_INSTALL_PREFIX="$INSTALLPATH"
			;;
		make-doc | make-docv)
			if [[ $1 == "make-docv" ]]; then
				"$DIR"/../doc/generate.sh
			else
				"$DIR"/../doc/generate.sh | grep warning || true
			fi
			if [[ $2 == "open" ]]; then
				DOC_URL="$DIR/../doc/NimbRo_Soccer_Package.html"
				if which xdg-open > /dev/null; then
					xdg-open $DOC_URL
				elif which gnome-open > /dev/null; then
					gnome-open $DOC_URL
				else
					echo "Could not detect the web browser to open the documentation with."
				fi
			fi
			;;
		deploy)
			if nimbro make install; then
				if [ -z "$2" ];then
					rsync -avz --delete /nimbro/ nimbro@$BOT:/nimbro
				else
					target=$2
					rsync -avz --delete /nimbro/ nimbro@$target:/nimbro
				fi
			fi
			;;
		remake-all)
			disable_udp
			echo "Removing build/ and devel/ folders from nimbro project root..."
			rm -rf "$ROOT/build" "$ROOT/devel"
			echo "Running catkin_make..."
			catkin_make $2 -DCMAKE_INSTALL_PREFIX="$INSTALLPATH"
			;;
		source | src)
			case "$2" in
				"" | "nimbro")
					cd "$ROOT/src/nimbro"
					;;
				"vis")
					cd "$ROOT/src/nimbro_vis"
					;;
				"rob" | "robot" | "robotcontrol")
					cd "$ROOT/src/nimbro_robotcontrol"
					;;
				"status")
					cd "$ROOT/src/nimbro"
					echo
					echo "*** nimbro repository ***"
					git status
					cd "$ROOT/src/nimbro_vis"
					echo
					echo "*** nimbro_vis repository ***"
					git status
					cd "$ROOT/src/nimbro_robotcontrol"
					echo
					echo "*** nimbro_robotcontrol repository ***"
					git status
					echo
					cd "$ROOT/src/nimbro"
					;;
				*)
					echo "Unknown parameter '$2': Going to main nimbro repository!"
					echo "Usage: nimbro source [repository]"
					echo "[repository] can be:"
					echo "nimbro:              <omitted>, nimbro"
					echo "nimbro_vis:          vis"
					echo "nimbro_robotcontrol: rob, robot, robotcontrol"
					echo "status:              Prints the git status of all the repositories"
					cd "$ROOT/src/nimbro"
					;;
			esac
			;;
		pull)
			cd "$ROOT/src/nimbro"
			echo
			echo "*** Pulling nimbro repository ***"
			pullgit
			cd "$ROOT/src/nimbro_vis"
			echo
			echo "*** Pulling nimbro_vis repository ***"
			pullgit
			cd "$ROOT/src/nimbro_robotcontrol"
			echo
			echo "*** Pulling nimbro_robotcontrol repository ***"
			pullgit
			cd "$ROOT/src"
			;;
		host)
			host=$2

			export ROS_MASTER_URI=http://$host:11311
			if ! ping -c1 $host > /dev/null; then
				echo "Could not resolve host name '$host'."
				return 1
			fi

			if ping -c1 `hostname`.local > /dev/null; then
				export ROS_HOSTNAME=`hostname`.local
			fi

			if ! rostopic list > /dev/null; then
				echo "Could not connect to ROS master running at '$host'"
				echo "Setting it as master anyway."
				BOT=$2
				return
			fi

			echo "Connected."
			BOT=$2
			;;
		ssh)
			if [ -z "$2" ]; then
			    ssh nimbro@$BOT
			else
				target=$2
				ssh nimbro@$target
			fi
			;;
		getconfig)
			LAUNCH=`rospack find launch`
			scp "nimbro@$BOT:/nimbro/share/launch/config/config*.yaml" "$LAUNCH/config/"
			scp "nimbro@$BOT:/nimbro/share/camera_v4l2/launch/cam_settings.yaml" "`rospack find camera_v4l2`/launch/"
			;;
		help|-h|--help)
			cat <<EOS
Usage: nimbro [command]

Commands:
  deploy      Make and deploy binaries to the robot
  getconfig   Get config.yaml from robot
  help        Display this help message
  host <HOST> Use HOST as ROS master, e.g. nimbro host xs2.local
  make        Run catkin_make with correct arguments in the correct directory
  make-doc    Compile the doxygen documentation (use 'make-doc open' to automatically open the html)
  pull        Pull and rebase the latest commits for each of the source repositories
  remake-all  Hard clean build and devel folders then run catkin_make to remake entire project
  source      cd to the nimbro source directory
  ssh         Open an SSH connection to the robot

The default command just cd's into the catkin workspace.
EOS
			;;
		*)
			$*
	esac
}

complete -o nospace -W "make deploy remake-all make-doc make-docv pull src source ssh host help getconfig" nimbro
