/**
 * @file    ui_form.h
 * @brief   OLED 表单式人机界面（u8g2 + MUI）
 *
 * 取代原来的「9 个键各绑一个功能 + 22 个标定序列靠背」。
 * 导航用中列三个键（物理上呈 上/中/下 排列）：上一个 / 确认 / 下一个。
 */

#ifndef __UI_FORM_H
#define __UI_FORM_H

/** 初始化显示与表单（含 u8g2 上电配置），并画第一屏。 */
void Ui_Init(void);

/** 每周期调用：读键盘 → 喂 MUI → 需要时重画。 */
void Ui_Tick(void);

/** 强制重画一次（例如外部改了要显示的值）。 */
void Ui_Refresh(void);

#endif /* __UI_FORM_H */
