#!/bin/bash

# This script (roughly) resets the project after generating new
# code via CubeMX. Specifically, FreeRTOS is custom (9.0.0 vs 8.x), and
# the .cproject file needs some xml fixups to avoid lots of diff noise.

# This file contains a workaround manually added.
git checkout -- Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c

# Using a newer version of FreeRTOS
git checkout -- Middlewares/Third_Party/FreeRTOS/Source

# Clean up the project file so it's in the format Eclipse uses.
sed -i -b 's_ />_/>_' .cproject
sed -i -b 's_<?xml version="1.0" encoding="UTF-8"?>_<?xml version="1.0" encoding="UTF-8" standalone="no"?>_' .cproject

# Workaround STM32Cube being stupid and insisting on defining a task as the default task which can't be extern'd.
sed -i -b 's_^void AdcReaderTask\(.*\);_extern void AdcReaderTask\1;_' Src/main.c
sed -i -b 's#^void AdcReaderTask\(.*\)$#__weak void AdcReaderTask\1#' Src/main.c

# Make ADC init method non-static so we can call it from app code.
sed -i -b 's#^static void MX_ADC1_Init\(.*\)#void MX_ADC1_Init\1#' Src/main.c

# Fix configPRE/POST_SLEEP_PROCESSING not passing a pointer to Pre/PostSleepProcessing
sed -i -b 's/^#define configPRE_SLEEP_PROCESSING.*/#define configPRE_SLEEP_PROCESSING(x)	PreSleepProcessing(\&x)/' Inc/FreeRTOSConfig.h
sed -i -b 's/^#define configPOST_SLEEP_PROCESSING.*/#define configPOST_SLEEP_PROCESSING(x)	PostSleepProcessing(\&x)/' Inc/FreeRTOSConfig.h

# Prevent SystemClock_Config from setting up the Systick since FreeRTOS handles the setup, and it causes issues calling SystemClock_Config() after sleep.
sed -i -b 's#\sHAL_SYSTICK_Config#//HAL_SYSTICK_Config#' Src/main.c
sed -i -b 's#\sHAL_SYSTICK_CLKSourceConfig#//HAL_SYSTICK_CLKSourceConfig#' Src/main.c

# Print git status for convenience
git status
