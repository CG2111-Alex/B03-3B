#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/kyrixn/ros_epp/src/catkin"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/kyrixn/ros_epp/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/kyrixn/ros_epp/install/lib/python3/dist-packages:/home/kyrixn/ros_epp/build/catkin/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/kyrixn/ros_epp/build/catkin" \
    "/usr/bin/python3" \
    "/home/kyrixn/ros_epp/src/catkin/setup.py" \
    egg_info --egg-base /home/kyrixn/ros_epp/build/catkin \
    build --build-base "/home/kyrixn/ros_epp/build/catkin" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/kyrixn/ros_epp/install" --install-scripts="/home/kyrixn/ros_epp/install/bin"
