#!/bin/bash

echo "[KILL] Stopping camera node"
rosnode kill /camera_nodelet
echo "[KILL] ... done"
