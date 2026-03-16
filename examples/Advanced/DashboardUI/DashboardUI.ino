/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <M5StamPLC.h>
#include "dashboard_ui.h"

DashboardUI dashboard_ui;

void update_time_and_date()
{
    static uint32_t time_count = 0;
    char string_buffer[100];

    if (millis() - time_count > 1000) {
        struct tm time;

        /* Get time */
        M5StamPLC.getRtcTime(&time);
        snprintf(string_buffer, sizeof(string_buffer), "%02d:%02d:%02d", time.tm_hour, time.tm_min, time.tm_sec);
        dashboard_ui.statusTime = string_buffer;

        /* Get date */
        snprintf(string_buffer, sizeof(string_buffer), "%04d.%02d.%02d", time.tm_year + 1900, time.tm_mon + 1,
                 time.tm_mday);
        dashboard_ui.statusDate = string_buffer;

        time_count = millis();
    }
}

void update_button_events()
{
    /* If button clicked, log out */
    if (M5StamPLC.BtnA().wasClicked()) {
        dashboard_ui.console_log("BtnA click");
    }
    if (M5StamPLC.BtnB().wasClicked()) {
        dashboard_ui.console_log("BtnB click");
    }
    if (M5StamPLC.BtnC().wasClicked()) {
        dashboard_ui.console_log("BtnC click");
    }

    /* Play button press and release tone */
    if (M5StamPLC.BtnA().wasPressed() || M5StamPLC.BtnB().wasPressed() || M5StamPLC.BtnC().wasPressed()) {
        M5StamPLC.tone(600, 20);
    } else if (M5StamPLC.BtnA().wasReleased() || M5StamPLC.BtnB().wasReleased() || M5StamPLC.BtnC().wasReleased()) {
        M5StamPLC.tone(800, 20);
    }
}

void update_plc_io_state()
{
    static uint32_t time_count = 0;

    if (millis() - time_count > 50) {
        for (int i = 0; i < 8; i++) {
            dashboard_ui.inputStateList[i] = M5StamPLC.readPlcInput(i);
        }
        for (int i = 0; i < 4; i++) {
            dashboard_ui.relayStateList[i] = M5StamPLC.readPlcRelay(i);
        }
        time_count = millis();
    }
}

void task_write_relay_regularly(void* param)
{
    delay(2000);

    bool relay_state = false;
    while (1) {
        relay_state = !relay_state;
        for (int i = 0; i < 4; i++) {
            M5StamPLC.writePlcRelay(i, relay_state);
            delay(500);
        }
        delay(1000);
    }
    vTaskDelete(NULL);
}

void setup()
{
    /* Init M5StamPLC */
    M5StamPLC.begin();

    /* Init dashboard UI */
    dashboard_ui.init(&M5StamPLC.Display());

    /* Create a task to write relay regularly */
    xTaskCreate(task_write_relay_regularly, "relay", 2048, NULL, 15, NULL);
}

void loop()
{
    M5StamPLC.update();

    update_time_and_date();
    update_button_events();
    update_plc_io_state();

    /* Render dashboard UI */
    dashboard_ui.render();
}
