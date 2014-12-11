#!/bin/bash

# home location lat, lon, alt, heading
LOCATION="CMAC"
TRACKER_LOCATION="CMAC_PILOTSBOX"
VEHICLE=""
BUILD_TARGET="sitl"
FRAME=""
NUM_PROCS=1

# check the instance number to allow for multiple copies of the sim running at once
INSTANCE=0
CLEAN_BUILD=0
START_ANTENNA_TRACKER=0
WIPE_EEPROM=0
NO_REBUILD=0
START_HIL=0
TRACKER_ARGS=""
EXTERNAL_SIM=0

usage()
{
cat <<EOF
Usage: sim_FG.sh [options] [mavproxy options]
Options:
	-v VEHICLE 	vehicle type (ArduPlane)
				defaults to working directory (use ArduPlane only currently)
				case sensitive
	-I INSTANCE instance of simulator (defaults to 0)
  -T      start an antenna tracker instance (not necessary)
  -A      pass arguments to antenna tracker (none that I'm aware of)
  -t      set antenna tracker start location (not useful for SITL probably)
  -L      select start location from Tools/autotest/locations.txt
  -c      do a make clean before building (usually not necessary)
  -w      wipe EEPROM and reload parameters (do this on first run)
  -f FRAME 	set aircraft frame type
    			for planes can chosse elevon or vtail (not necessary, but may be useful) -- currently no longer used
	-j NUM	number of processors to use during build (default 1) use 4 when building
	-H 			start HIL
	-e 			use external simulator (maybe very useful!!!!!!!!!)

Mavproxy Options:
	--map 		start with map (doens't seem to work?)
	--console 	start with a status console (probably helpful, only works on linux)
	--out DEST	start MAVLink output to DEST

Note:
	Always start from the vehicle directory you are simulating.
	For planes, always start in ArduPlane, although the current version defaults to ArduPlane instead of to directory (use -v to change it)
EOF
}
while getopts ":I:cj:TA:t:L:v:hw:He" opt; do
	case $opt in
		v)
			VEHICLE=$OPTARG
			;;
		I)
			INSTANCE=$OPTARG
			;;
  	H)
  		START_HIL=1
  		NO_REBUILD=1
  		;;
  	T)
  		START_ANTENNA_TRACKER=1
  		;;
  	A)
  		TRACKER_ARGS="$OPTARG"
  		;;
  	L)
  		LOCATION="$OPTARG"
  		;;
  	t)
  		TRACKER_LOCATION="$OPTARG"
  		;;
	  c)
  		CLEAN_BUILD=1
  		;;
  	j)
  		NUM_PROCS=$OPTARG
  		;;
  	w)
  		WIPE_EEPROM=1
  		;;
  	e)
  		EXTERNAL_SIM=1
  		;;
  	h)
  		usage
  		exit 0
  		;;
  	\?)
    	# allow other args to pass on to mavproxy
    	break
    	;;
    :)
    	echo "Option -$OPTARG requires an argument." >&2
    	usage
    	exit 1
  	esac
done
shift $((OPTIND-1))

# close existing copies if this is the 0 instance
# update this as more scripts get made/used/added
kill_tasks()
{
	["$INSTANCE" -eq "0"] && {
		killall -q JSBSim lt-JSBSim ArduPlane.elf ArduCopter.elf APMrover2.elf AntennaTracker.elf
		pkill -f runsim.py runFG.py
		pkill -f sim_tracker.py
	}
}

kill_tasks
trap kill_tasks SIGINT

# Ports
MAVLINK_PORT="tcp:127.0.0.1:"$((5760+10*INSTANCE))
SIMIN_PORT="10.148.13.147:"$((5501+10*INSTANCE))
SIMOUT_PORT="10.148.13.147:"$((5502+10*INSTANCE))

set -x
[ -z "$VEHICLE" ] && {
  VEHICLE="ArduPlane"
}

EXTRA_PARM=""
EXTRA_SIM=""

# modify build target based on frame type (exludes copter currently, can be updated later if needed, also lacks skid steering)
case $FRAME in
  obc)
    BUILD_TARGET="sitl-obc"
    ;;
  "")
    ;;
  *)
    echo "Unknown Frame type $FRAME, quitting sim"
    usage
    exit 1
    ;;
esac

autotest=$(dirname $(readlink -e $0))
if [ $NO_REBUILD == 0 ]; then
pushd $autotest/../../$VEHICLE || {
  echo "Failed to change to vehicle directory for $VEHICLE, quitting"
  usage
  exit 1
}
if [ $CLEAN_BUILD == 1 ]; then
  make clean
fi
make $BUILD_TARGET -j$NUM_PROCS || {
  make clean
  make $BUILD_TARGET -j$NUM_PROCS
}
popd
fi

# get start location info
SIMHOME=$(cat $autotest)
[ -z "$SIMHOME" ] && {
    echo "Unknown location $LOCATION"
    usage
    exit 1
}
echo "Starting up at $LOCATION : $SIMHOME"

TRACKER_HOME=$(cat $autotest/locations.txt | grep -i "^$TRACKER_LOCATION=" | cut -d= -f2)
[ -z "$TRACKER_HOME" ] && {
    echo "Unknown tracker location $TRACKER_LOCATION"
    usage
    exit 1
}

cmd="/tmp/$VEHICLE.build/$VEHICLE.elf -I$INSTANCE"
if [ $WIPE_EEPROM == 1 ]; then
    cmd="$cmd -w"
fi

case $VEHICLE in
    ArduPlane)
        [ "$REVERSE_THROTTLE" == 1 ] && {
            EXTRA_SIM="$EXTRA_SIM --revthr"
        }
        RUNSIM="nice $autotest/jsbsim/runsim.py --home=$SIMHOME --simin=$SIMIN_PORT --simout=$SIMOUT_PORT --fgout=$FG_PORT $EXTRA_SIM"
        PARMS="ArduPlane.parm"
        if [ $WIPE_EEPROM == 1 ]; then
            cmd="$cmd -PFORMAT_VERSION=13 -PSKIP_GYRO_CAL=1 -PRC3_MIN=1000 -PRC3_TRIM=1000"
        fi
        ;;
    *)
        echo "Unknown vehicle simulation type $VEHICLE - please specify vehicle using -v VEHICLE_TYPE"
        usage
        exit 1
        ;;
esac

trap kill_tasks SIGINT


















