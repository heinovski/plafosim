#!/bin/bash

ROOT=$(pwd)/$(dirname $0)/../

export PYTHONPATH=${ROOT}/src

pytest -v ${ROOT}/tests
