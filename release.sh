#!/bin/bash

set -o nounset
set -o errexit

DEST=build/release
rm -fr $DEST && mkdir -p $DEST
cp src/rosbag_wrapper.mex* $DEST/
cp -r src/example $DEST/
cp -r src/+ros $DEST/
cp src/README $DEST/
cp LICENSE.txt $DEST/
cp LICENSE-Willow.txt $DEST/
