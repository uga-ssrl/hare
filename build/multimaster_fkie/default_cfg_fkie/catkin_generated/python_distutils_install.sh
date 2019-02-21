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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jackson/Development/HARE/src/multimaster_fkie/default_cfg_fkie"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jackson/Development/HARE/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jackson/Development/HARE/install/lib/python2.7/dist-packages:/home/jackson/Development/HARE/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jackson/Development/HARE/build" \
    "/usr/bin/python" \
    "/home/jackson/Development/HARE/src/multimaster_fkie/default_cfg_fkie/setup.py" \
    build --build-base "/home/jackson/Development/HARE/build/multimaster_fkie/default_cfg_fkie" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/jackson/Development/HARE/install" --install-scripts="/home/jackson/Development/HARE/install/bin"
