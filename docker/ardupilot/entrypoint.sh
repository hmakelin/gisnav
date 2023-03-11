#!/bin/bash
set -e
cd $HOME
exec -w $HOME "$@"
