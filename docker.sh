#!/bin/bash
NAME=rfm69

docker build -t "$NAME" .
docker run --privileged --rm -v "$PWD":/work -w /work -it "$NAME" bash
