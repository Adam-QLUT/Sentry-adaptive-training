//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_Omni_H
#define UI_Omni_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_Omni_now_figures[8];
extern uint8_t ui_Omni_dirty_figure[8];

#define ui_Omni_Shoot_Low_Auto_rect ((ui_interface_rect_t*)&(ui_Omni_now_figures[0]))
#define ui_Omni_Shoot_Low_Shoot_Round ((ui_interface_round_t*)&(ui_Omni_now_figures[1]))
#define ui_Omni_Chassis_high_Chassis_arc ((ui_interface_arc_t*)&(ui_Omni_now_figures[2]))
#define ui_Omni_Chassis_high_Supercap_rect ((ui_interface_rect_t*)&(ui_Omni_now_figures[3]))
#define ui_Omni_Chassis_high_Supercap_Line ((ui_interface_line_t*)&(ui_Omni_now_figures[4]))
#define ui_Omni_Shoot_static_Shoot_Line ((ui_interface_line_t*)&(ui_Omni_now_figures[5]))
#define ui_Omni_Shoot_Low_Shootend_Round ((ui_interface_round_t*)&(ui_Omni_now_figures[6]))
#define ui_Omni_Shoot_Low_Shootend_Round_Long ((ui_interface_round_t*)&(ui_Omni_now_figures[7]))


#ifdef MANUAL_DIRTY
#define ui_Omni_Shoot_Low_Auto_rect_dirty (ui_Omni_dirty_figure[0])
#define ui_Omni_Shoot_Low_Shoot_Round_dirty (ui_Omni_dirty_figure[1])
#define ui_Omni_Chassis_high_Chassis_arc_dirty (ui_Omni_dirty_figure[2])
#define ui_Omni_Chassis_high_Supercap_rect_dirty (ui_Omni_dirty_figure[3])
#define ui_Omni_Chassis_high_Supercap_Line_dirty (ui_Omni_dirty_figure[4])
#define ui_Omni_Shoot_static_Shoot_Line_dirty (ui_Omni_dirty_figure[5])
#define ui_Omni_Shoot_Low_Shootend_Round_dirty (ui_Omni_dirty_figure[6])
#define ui_Omni_Shoot_Low_Shootend_Round_Long_dirty (ui_Omni_dirty_figure[7])

#endif

void ui_init_Omni();
void ui_update_Omni();

#endif // UI_Omni_H
