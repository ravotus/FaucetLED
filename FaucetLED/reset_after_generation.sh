#!/bin/bash

# This script (roughly) resets the project after generating new
# code via CubeMX. Specifically, FreeRTOS is custom (9.0.0 vs 8.x), and
# the .cproject file needs some xml fixups to avoid lots of diff noise.

git checkout -- Middlewares/Third_Party/FreeRTOS/Source
sed -i -b 's_ />_/>_' .cproject
sed -i -b 's_<?xml version="1.0" encoding="UTF-8"?>_<?xml version="1.0" encoding="UTF-8" standalone="no"?>_' .cproject

# Workaround STM32Cube being stupid and insisting on defining a task as the default task which can't be extern'd.
sed -i -b 's_^void AdcReaderTask\(.*\);_extern void AdcReaderTask\1;_' Src/main.c
sed -i -b 's#^void AdcReaderTask\(.*\)$#__weak void AdcReaderTask\1#' Src/main.c

# Print git status for convenience
git status
