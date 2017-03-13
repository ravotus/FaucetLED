#!/bin/bash

# This script (roughly) resets the project after generating new
# code via CubeMX. Specifically, FreeRTOS is custom (9.0.0 vs 8.x), and
# the .cproject file needs some xml fixups to avoid lots of diff noise.

# Using a newer version of FreeRTOS
git checkout -- Middlewares/Third_Party/FreeRTOS/Source

# Clean up the project file so it's in the format Eclipse uses.
sed -i -b 's_ />_/>_' .cproject
sed -i -b 's_<?xml version="1.0" encoding="UTF-8"?>_<?xml version="1.0" encoding="UTF-8" standalone="no"?>_' .cproject

# Fix configPRE/POST_SLEEP_PROCESSING not passing a pointer to Pre/PostSleepProcessing
sed -i -b 's/^#define configPRE_SLEEP_PROCESSING.*/#define configPRE_SLEEP_PROCESSING(x)	PreSleepProcessing(\&x)/' Inc/FreeRTOSConfig.h
sed -i -b 's/^#define configPOST_SLEEP_PROCESSING.*/#define configPOST_SLEEP_PROCESSING(x)	PostSleepProcessing(\&x)/' Inc/FreeRTOSConfig.h

# Prevent SystemClock_Config from setting up the Systick since FreeRTOS handles the setup, and it causes issues calling SystemClock_Config() after sleep.
# TODO: Enable when tickless idle is working for CM0.
# sed -i -b 's#\sHAL_SYSTICK_Config#//HAL_SYSTICK_Config#' Src/main.c
# sed -i -b 's#\sHAL_SYSTICK_CLKSourceConfig#//HAL_SYSTICK_CLKSourceConfig#' Src/main.c

# Print git status for convenience
git status
