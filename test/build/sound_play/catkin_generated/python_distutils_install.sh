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

echo_and_run cd "/home/lzh/test/src/sound_play"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/lzh/test/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/lzh/test/install/lib/python3/dist-packages:/home/lzh/test/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/lzh/test/build" \
    "/usr/bin/python3" \
    "/home/lzh/test/src/sound_play/setup.py" \
    egg_info --egg-base /home/lzh/test/build/sound_play \
    build --build-base "/home/lzh/test/build/sound_play" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/lzh/test/install" --install-scripts="/home/lzh/test/install/bin"
