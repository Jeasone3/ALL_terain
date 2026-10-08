#ifndef OLED_DEBUG_H
#define OLED_DEBUG_H
#include "Mode_FSM.h"
#include "oled.h"

/* 只在前台调用。日志在 main 统一取出，再分别交给串口与此模块。 */
void OLED_Debug_Init(OLED_Device *device, uint32_t now_ms);
void OLED_Debug_OnFrame(const ModeFSM_DebugFrame *frame);
void OLED_Debug_Tick(uint32_t now_ms);

#endif
