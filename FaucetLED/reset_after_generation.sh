#!/bin/bash

# This script (roughly) resets the project after generating new
# code via CubeMX. Specifically, FreeRTOS is custom (9.0.0 vs 8.x), and
# the .cproject file needs some xml fixups to avoid lots of diff noise.

git checkout -- Middlewares/Third_Party/FreeRTOS/Source
sed -i -b 's_ />_/>_' .cproject
sed -i -b 's_<?xml version="1.0" encoding="UTF-8"?>_<?xml version="1.0" encoding="UTF-8" standalone="no"?>_' .cproject

# Print git status for convenience
git status
