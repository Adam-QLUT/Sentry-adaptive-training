//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_Omni.h"

#define TOTAL_FIGURE 8
#define TOTAL_STRING 0

ui_interface_figure_t ui_Omni_now_figures[TOTAL_FIGURE];
uint8_t ui_Omni_dirty_figure[TOTAL_FIGURE];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_Omni_last_figures[TOTAL_FIGURE];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, NULL, NULL, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_Omni() {
    ui_Omni_Shoot_Low_Auto_rect->figure_type = 1;
    ui_Omni_Shoot_Low_Auto_rect->operate_type = 1;
    ui_Omni_Shoot_Low_Auto_rect->layer = 0;
    ui_Omni_Shoot_Low_Auto_rect->color = 8;
    ui_Omni_Shoot_Low_Auto_rect->start_x = 685;
    ui_Omni_Shoot_Low_Auto_rect->start_y = 246;
    ui_Omni_Shoot_Low_Auto_rect->width = 5;
    ui_Omni_Shoot_Low_Auto_rect->end_x = 1247;
    ui_Omni_Shoot_Low_Auto_rect->end_y = 808;

    ui_Omni_Shoot_Low_Shoot_Round->figure_type = 2;
    ui_Omni_Shoot_Low_Shoot_Round->operate_type = 1;
    ui_Omni_Shoot_Low_Shoot_Round->layer = 0;
    ui_Omni_Shoot_Low_Shoot_Round->color = 4;
    ui_Omni_Shoot_Low_Shoot_Round->start_x = 960;
    ui_Omni_Shoot_Low_Shoot_Round->start_y = 540;
    ui_Omni_Shoot_Low_Shoot_Round->width = 2;
    ui_Omni_Shoot_Low_Shoot_Round->r = 13;

    ui_Omni_Chassis_high_Chassis_arc->figure_type = 4;
    ui_Omni_Chassis_high_Chassis_arc->operate_type = 1;
    ui_Omni_Chassis_high_Chassis_arc->layer = 0;
    ui_Omni_Chassis_high_Chassis_arc->color = 1;
    ui_Omni_Chassis_high_Chassis_arc->start_x = 960;
    ui_Omni_Chassis_high_Chassis_arc->start_y = 540;
    ui_Omni_Chassis_high_Chassis_arc->width = 5;
    ui_Omni_Chassis_high_Chassis_arc->start_angle = 150;
    ui_Omni_Chassis_high_Chassis_arc->end_angle = 210;
    ui_Omni_Chassis_high_Chassis_arc->rx = 330;
    ui_Omni_Chassis_high_Chassis_arc->ry = 330;

    ui_Omni_Chassis_high_Supercap_rect->figure_type = 1;
    ui_Omni_Chassis_high_Supercap_rect->operate_type = 1;
    ui_Omni_Chassis_high_Supercap_rect->layer = 0;
    ui_Omni_Chassis_high_Supercap_rect->color = 6;
    ui_Omni_Chassis_high_Supercap_rect->start_x = 445;
    ui_Omni_Chassis_high_Supercap_rect->start_y = 159;
    ui_Omni_Chassis_high_Supercap_rect->width = 3;
    ui_Omni_Chassis_high_Supercap_rect->end_x = 1436;
    ui_Omni_Chassis_high_Supercap_rect->end_y = 201;

    ui_Omni_Chassis_high_Supercap_Line->figure_type = 0;
    ui_Omni_Chassis_high_Supercap_Line->operate_type = 1;
    ui_Omni_Chassis_high_Supercap_Line->layer = 0;
    ui_Omni_Chassis_high_Supercap_Line->color = 2;
    ui_Omni_Chassis_high_Supercap_Line->start_x = 450;
    ui_Omni_Chassis_high_Supercap_Line->start_y = 166;
    ui_Omni_Chassis_high_Supercap_Line->width = 30;
    ui_Omni_Chassis_high_Supercap_Line->end_x = 1429;
    ui_Omni_Chassis_high_Supercap_Line->end_y = 166;

    ui_Omni_Shoot_static_Shoot_Line->figure_type = 0;
    ui_Omni_Shoot_static_Shoot_Line->operate_type = 1;
    ui_Omni_Shoot_static_Shoot_Line->layer = 0;
    ui_Omni_Shoot_static_Shoot_Line->color = 0;
    ui_Omni_Shoot_static_Shoot_Line->start_x = 960;
    ui_Omni_Shoot_static_Shoot_Line->start_y = 540;
    ui_Omni_Shoot_static_Shoot_Line->width = 2;
    ui_Omni_Shoot_static_Shoot_Line->end_x = 960;
    ui_Omni_Shoot_static_Shoot_Line->end_y = 400;

    ui_Omni_Shoot_Low_Shootend_Round->figure_type = 2;
    ui_Omni_Shoot_Low_Shootend_Round->operate_type = 1;
    ui_Omni_Shoot_Low_Shootend_Round->layer = 0;
    ui_Omni_Shoot_Low_Shootend_Round->color = 0;
    ui_Omni_Shoot_Low_Shootend_Round->start_x = 950;
    ui_Omni_Shoot_Low_Shootend_Round->start_y = 500;
    ui_Omni_Shoot_Low_Shootend_Round->width = 3;
    ui_Omni_Shoot_Low_Shootend_Round->r = 12;

    ui_Omni_Shoot_Low_Shootend_Round_Long->figure_type = 2;
    ui_Omni_Shoot_Low_Shootend_Round_Long->operate_type = 1;
    ui_Omni_Shoot_Low_Shootend_Round_Long->layer = 0;
    ui_Omni_Shoot_Low_Shootend_Round_Long->color = 0;
    ui_Omni_Shoot_Low_Shootend_Round_Long->start_x = 950;
    ui_Omni_Shoot_Low_Shootend_Round_Long->start_y = 462;
    ui_Omni_Shoot_Low_Shootend_Round_Long->width = 3;
    ui_Omni_Shoot_Low_Shootend_Round_Long->r = 12;

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_Omni_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_Omni_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_Omni_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_Omni_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_Omni_last_figures[i] = ui_Omni_now_figures[i];
#endif
        ui_Omni_dirty_figure[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_Omni_now_figures[i].operate_type = 2;
    }
}

void ui_update_Omni() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_Omni_now_figures[i], &ui_Omni_last_figures[i], sizeof(ui_Omni_now_figures[i])) != 0) {
            ui_Omni_dirty_figure[i] = 1;
            ui_Omni_last_figures[i] = ui_Omni_now_figures[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
