#!/bin/bash

set -o nounset
set -o errexit

VERSION=$1
if [[ $(uname -s) == "Linux" ]]; then
    SUFFIX="linux64"
else
    SUFFIX="mac64"
fi

NAME=matlab_rosbag-$VERSION-$SUFFIX
DEST=build/$NAME
rm -fr $DEST && mkdir -p $DEST
cp src/rosbag_wrapper.mex* $DEST/
cp -r src/example $DEST/
cp -r src/+ros $DEST/
cp README.md $DEST/README
cp LICENSE.txt $DEST/
cp LICENSE-Willow.txt $DEST/
cd build/ && rm -f $NAME.zip && zip -r $NAME.zip $NAME
echo Created build/$NAME.zip
