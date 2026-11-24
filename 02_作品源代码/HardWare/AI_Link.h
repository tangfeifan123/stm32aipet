#ifndef __AI_LINK_H
#define __AI_LINK_H

#include <stdint.h>

/* 接收环形缓冲区大小，必须是 2 的幂，便于取模优化 */
#define AI_RX_BUF_SIZE	256

/* 单帧文本最大长度（不含结束符） */
#define AI_TEXT_MAX		60

/* 单帧数据区上限：TALK 帧 = 表情 + 动作 + 速度 + 文本 */
#define AI_FRAME_MAX	(AI_TEXT_MAX + 3)

/* 图片帧数据区上限：表情 + 动作 + 页号 + 128 字节点阵 */
#define AI_IMG_MAX		131

/* 整屏点阵大小：128*64 像素，SSD1306 页式寻址，8 页 × 128 字节 */
#define AI_IMG_BYTES	(128 * 8)

/* 说完话后停留多久再恢复表情（毫秒），由 TIM3 的 20ms 中断计时 */
#define AI_SAY_MS		8000

/* 帧类型 */
#define AI_CMD_TALK		0x01	/* AI 回复：表情 + 动作 + 文本 */
#define AI_CMD_PING		0x02	/* 心跳，用于测试链路 */
#define AI_CMD_TEXT		0x03	/* 只显示文本，不改动作 */
#define AI_CMD_IMG		0x04	/* 图片帧：电脑渲染好中文整屏点阵，分 8 页发过来 */
#define AI_CMD_SPEAK	0x05	/* 语音帧：GBK 文本，转给 TTS 模块(SYN6288)念出来 */

/* ------------------------------------------------------------------
 * 调试屏开关
 *   1 = 屏幕显示链路统计（不再显示表情），用于定位"命令没反应"。
 *       显示内容：
 *         BLE : 蓝牙串口(USART3)收到的总字节数  —— 不动说明字节没进 MCU
 *         VOI : 语音串口(USART1)收到的总字节数  —— 猛涨说明语音模块在刷屏
 *         LST : 蓝牙收到的最后一个字节
 *         MD  : 当前 Action_Mode
 *         FRM : 成功解析的 AI 帧数
 *         TK  : TIM3 计时（秒，滚动）—— 不动说明主循环/中断死了
 *   0 = 正常显示表情。调试完成后把它改回 0 重新烧录即可。
 * ------------------------------------------------------------------ */
#define AI_DEBUG_UI		0

/* 调试统计量，中断里累加，主循环只读 */
extern volatile uint32_t	g_dbgBleRx;		/* USART3 收到的字节数 */
extern volatile uint32_t	g_dbgVoiceRx;	/* USART1 收到的字节数 */
extern volatile uint32_t	g_dbgFrames;	/* 解析成功的帧数 */
extern volatile uint32_t	g_dbgLegacy;	/* Legacy_Cmd 被调用次数 */
extern volatile uint8_t		g_dbgLastBle;	/* 最后一个蓝牙字节 */

void AI_Link_Init(void);
void AI_Link_RxByte(uint8_t Byte);		/* 只能在 USART3 中断里调用 */
void AI_Link_RxVoiceByte(uint8_t Byte);	/* 只能在 USART1 中断里调用 */
void AI_Link_TickIsr(void);				/* 只能在 TIM3 中断里调用，每 20ms 一次 */
void AI_Link_Task(void);				/* 在主循环里反复调用 */
void AI_Link_Send(uint8_t Type, const uint8_t *Data, uint8_t Len);

#endif
