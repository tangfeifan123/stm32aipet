#include "stm32f10x.h"
#include "AI_Link.h"
#include "BlueTooth.h"
#include "Face_Config.h"
#include "OLED.h"

/***************************************************************************************
  * AI 链路模块 —— 让桌面宠物"听懂"电脑/手机上的 DeepSeek 回复
  *
  * 数据流：
  *   PC(调 DeepSeek) --BLE--> ECB01H2S --USART3--> 本模块 --解析--> 表情/动作/文字
  *
  * 帧格式（二进制，抗误判）：
  *   0xAA 0x55 | TYPE | LEN | DATA[LEN] | XOR(TYPE,LEN,DATA...)
  *
  * 两个串口的中断现在都只做一件事：把字节塞进各自的环形缓冲。
  * 所有业务（解析、OLED、动作）统一在主循环的 AI_Link_Task() 里做。
  *
  * 为什么要这样改：
  *   老代码在 USART1（语音模块）中断里直接调用 Face_Config() 刷 OLED。
  *   OLED 是软件 I2C，靠忙等延时产生时钟，一次刷新十几毫秒。
  *   只要 USART1 上有连续字节（语音模块上电噪声、波特率不匹配产生乱码、
  *   或者模块真的在不停发数据），中断就会被永久重入，主循环一次都跑不到。
  *   老固件的蓝牙命令是在 USART3 中断里直接处理的，所以照样能动；
  *   而现在解析放在主循环，一旦主循环被饿死，蓝牙就彻底"没反应"。
  ***************************************************************************************/

/* ---------------- 帧定义 ---------------- */
#define AI_SOF1			0xAA
#define AI_SOF2			0x55

#define AI_ST_SOF1		0
#define AI_ST_SOF2		1
#define AI_ST_TYPE		2
#define AI_ST_LEN		3
#define AI_ST_DATA		4
#define AI_ST_SUM		5

/* ---------------- 蓝牙接收环形缓冲 ---------------- */
static uint8_t				s_rx[AI_RX_BUF_SIZE];
static volatile uint16_t	s_head = 0;		/* 中断写 */
static volatile uint16_t	s_tail = 0;		/* 主循环写 */

/* ---------------- 语音模块接收环形缓冲（USART1） ---------------- */
static uint8_t				s_rxV[64];
static volatile uint16_t	s_headV = 0;
static volatile uint16_t	s_tailV = 0;

/* ---------------- 调试统计 ---------------- */
volatile uint32_t	g_dbgBleRx   = 0;
volatile uint32_t	g_dbgVoiceRx = 0;
volatile uint32_t	g_dbgFrames  = 0;
volatile uint32_t	g_dbgLegacy  = 0;
volatile uint8_t	g_dbgLastBle = 0;

/* 最近收到的字节原文（只在调试屏里用，正常固件不占这份 RAM） */
#if AI_DEBUG_UI
#define HIST_N		32
static uint8_t			s_hist[HIST_N];
static volatile uint8_t	s_histCnt = 0;
#endif

/* ---------------- 解析状态机 ---------------- */
static uint8_t	s_state = AI_ST_SOF1;
static uint8_t	s_type;
static uint8_t	s_len;
static uint8_t	s_idx;
static uint8_t	s_sum;
static uint8_t	s_data[AI_IMG_MAX];

/* ---------------- 整屏图片接收缓冲（中文点阵，8 页 × 128 字节） ---------------- */
static uint8_t	s_img[AI_IMG_BYTES];

/* ---------------- 文本显示 ---------------- */
static char		s_text[AI_TEXT_MAX + 1];
static volatile uint32_t	s_tickMs = 0;		/* 由 TIM3 中断每 20ms 累加 */
static uint32_t	s_sayStart = 0;					/* 开始说话的时刻 */
static uint8_t	s_saying = 0;

/* ===================================================================
 * 串口上报（给 PC 回 ACK）
 * =================================================================== */
static void AI_SendByte(uint8_t Byte)
{
	uint32_t Guard = 0;

	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
	{
		if (++Guard > 200000) return;		/* 防止串口异常时死等 */
	}
	USART_SendData(USART3, Byte);
}

void AI_Link_Send(uint8_t Type, const uint8_t *Data, uint8_t Len)
{
	uint8_t i, Sum;

	Sum = Type ^ Len;
	AI_SendByte(AI_SOF1);
	AI_SendByte(AI_SOF2);
	AI_SendByte(Type);
	AI_SendByte(Len);
	for (i = 0; i < Len; i++)
	{
		AI_SendByte(Data[i]);
		Sum ^= Data[i];
	}
	AI_SendByte(Sum);
}

/* ===================================================================
 * SYN6288 语音合成模块（TTS）
 * 接在 USART1 的 TX（PA9）。把 GBK 编码的中文文本按它的帧格式发过去，
 * 模块就会用喇叭念出来。波特率 9600（默认）。
 * 帧格式：0xFD + 数据长度(2字节,高位在前) + 0x01(合成命令) + GBK文本
 * =================================================================== */
static void UART1_SendByte(uint8_t Byte)
{
	uint32_t Guard = 0;
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
	{
		if (++Guard > 200000) return;
	}
	USART_SendData(USART1, Byte);
}

static void Syn6288_Speak(const uint8_t *Text, uint8_t Len)
{
	uint16_t Total = (uint16_t)Len + 1;		/* 1 字节合成命令 + 文本 */
	uint8_t i;

	if (Len == 0) return;

	UART1_SendByte(0xFD);
	UART1_SendByte((uint8_t)(Total >> 8));
	UART1_SendByte((uint8_t)(Total & 0xFF));
	UART1_SendByte(0x01);
	for (i = 0; i < Len; i++)
		UART1_SendByte(Text[i]);
}

/* ===================================================================
 * 文本显示：128x64 屏，8x16 字体，一行 16 字符，共 4 行
 * =================================================================== */
static void Pet_Say(const char *Text)
{
	char Line[17];
	uint8_t Row, n, p, c, Cut;

	p = 0;
	OLED_Clear();
	for (Row = 0; Row < 4; Row++)
	{
		while (Text[p] == ' ') p++;			/* 行首空格不占位置 */
		n = 0;
		while (n < 16 && Text[p] != '\0')
		{
			c = (uint8_t)Text[p++];
			if (c < 0x20 || c > 0x7E) c = ' ';	/* 非 ASCII 可见字符替换成空格 */
			Line[n++] = (char)c;
		}

		/* 满 16 字符、后面还有内容、而且正好断在单词中间：
		   回退到最近的空格处换行，避免出现 "wit / h" 这种断词 */
		if (n == 16 && Text[p] != '\0' && Line[15] != ' ')
		{
			Cut = 15;
			while (Cut > 0 && Line[Cut] != ' ') Cut--;
			if (Cut > 0)					/* 找到了空格 */
			{
				p -= (16 - Cut);			/* 把多读进去的字符退回 */
				Line[Cut] = '\0';
				n = Cut;
			}
			/* Cut 减到 0 说明 16 个字符里没空格（超长单词），只能硬切 */
		}

		if (n == 0) break;
		Line[n] = '\0';
		OLED_ShowString(0, Row * 16, Line, OLED_8X16);
	}
	OLED_Update();

	s_saying = 1;
	s_sayStart = s_tickMs;
}

/* ===================================================================
 * 老协议：单字节指令（0x29, 0x30~0x49）
 *
 *   KeepMoving = 1：蓝牙遥控，动作持续到收到下一条指令（原 USART3 行为）
 *   KeepMoving = 0：语音模块，走完规定次数就停（原 USART1 行为）
 * =================================================================== */
static void Legacy_Cmd(uint8_t Cmd, uint8_t KeepMoving)
{
#if AI_BAUD_SWEEP
	/* 波特率识别阶段只统计字节，一律不执行动作 */
	(void)Cmd;
	(void)KeepMoving;
	return;
#endif

	Sustainedmove = KeepMoving;
	g_dbgLegacy++;

	switch (Cmd)
	{
		case 0x29: Face_Mode = 0; Face_Config(); Action_Mode = 0; break;	/* 放松趴下 */
		case 0x30: Face_Mode = 1; Face_Config(); Action_Mode = 1; break;	/* 蹲下 */
		case 0x31: Face_Mode = 5; Face_Config(); Action_Mode = 2; break;	/* 直立 */
		case 0x32: Face_Mode = 1; Face_Config(); Action_Mode = 3; break;	/* 趴下 */
		case 0x33: Face_Mode = 2; Face_Config(); Action_Mode = 4; break;	/* 前进 */
		case 0x34: Face_Mode = 2; Face_Config(); Action_Mode = 5; break;	/* 后退 */
		case 0x35: Face_Mode = 2; Face_Config(); Action_Mode = 6; break;	/* 左转 */
		case 0x36: Face_Mode = 2; Face_Config(); Action_Mode = 7; break;	/* 右转 */
		case 0x37: Face_Mode = 4; Face_Config(); Action_Mode = 8; break;	/* 摇摆 */

		case 0x38:															/* 加速 */
			if (SpeedDelay == 120) { Face_Mode = 3; Face_Config(); }
			if (SpeedDelay > 100)
				SpeedDelay -= 20;
			else
			{
				Face_Mode = 2;
				Face_Config();
				SpeedDelay = 200;
			}
			break;

		case 0x39:															/* 摇摆加速 */
			if (SwingDelay == 4) { Face_Mode = 3; Face_Config(); }
			if (SwingDelay > 3)
				SwingDelay--;
			else
			{
				Face_Mode = 4;
				Face_Config();
				SwingDelay = 9;
			}
			break;

		case 0x40:															/* 摇尾巴开关 */
			(WeiBa == 0) ? (WeiBa = 1) : (WeiBa = 0);
			Face_Mode = 1; Face_Config(); Action_Mode = 9;
			break;

		case 0x41: Face_Mode = 2; Face_Config(); Action_Mode = 10; break;	/* 前跳 */
		case 0x42: Face_Mode = 2; Face_Config(); Action_Mode = 11; break;	/* 后跳 */
		case 0x43: Face_Mode = 6; Face_Config(); Action_Mode = 13; break;	/* 打招呼 */
		case 0x44: AllLed = 1; break;										/* 开灯 */
		case 0x45: AllLed = 0; break;										/* 关灯 */
		case 0x46: BreatheLed = 1; break;									/* 开呼吸灯 */
		case 0x47: BreatheLed = 0; break;									/* 关呼吸灯 */
		case 0x48: Face_Mode = 6; Face_Config(); Action_Mode = 14; break;	/* 伸懒腰 */
		case 0x49: Face_Mode = 6; Face_Config(); Action_Mode = 15; break;	/* 后腿拉伸 */
		default: break;
	}
}

/* ===================================================================
 * 语音模块指令映射
 *
 * SU-03T 工程（版本 20261027001"SU-03T小狗"）里每条命令的串口输出是两个
 * 相同的 ASCII 字母，例如"睡觉"发 AA、"前进"发 EE、"叫"发 WW。
 * 注意 GG / HH / WW 这些不是合法十六进制，所以它一定是 ASCII 字符，
 * 不是十六进制字节 —— 语音模块每个命令会在 USART1 上吐 2 个字节。
 *
 * 老固件把 0x41('A') 当成"前跳"，于是喊"睡觉"会跳两下。这里按字母重新映射。
 * =================================================================== */
#define AI_VOICE_MAP	1

#if AI_VOICE_MAP
static uint8_t s_voicePend = 0;		/* 已收到一个字母，等第二个相同字母 */

/* 字母 -> 老指令字节，再交给 Legacy_Cmd 执行 */
static void Voice_Cmd(uint8_t Cmd)
{
	uint8_t Legacy;

	if (Cmd < 'A' || Cmd > 'W')			/* 不是本工程的命令字母 */
	{
		s_voicePend = 0;
		return;
	}

	if (s_voicePend != Cmd)				/* 第一字节，先记住 */
	{
		s_voicePend = Cmd;
		return;
	}
	s_voicePend = 0;					/* 第二字节，凑成一对，执行 */

	switch (Cmd)
	{
		case 'A': Legacy = 0x29; break;	/* 睡觉     -> 放松趴下 */
		case 'B': Legacy = 0x30; break;	/* 蹲下     -> 坐下     */
		case 'C': Legacy = 0x31; break;	/* 站立     -> 直立     */
		case 'D': Legacy = 0x32; break;	/* 趴下     */
		case 'E': Legacy = 0x33; break;	/* 前进     */
		case 'F': Legacy = 0x34; break;	/* 后退     */
		case 'G': Legacy = 0x35; break;	/* 左转     */
		case 'H': Legacy = 0x36; break;	/* 右转     */
		case 'I': Legacy = 0x37; break;	/* 摇摆     */
		case 'J': Legacy = 0x38; break;	/* 加快移动 */
		case 'K': Legacy = 0x39; break;	/* 加快摇摆 */
		case 'L': Legacy = 0x40; break;	/* 摇尾巴   */
		case 'M': Legacy = 0x41; break;	/* 向前跳   */
		case 'N': Legacy = 0x42; break;	/* 向后跳   */
		case 'O': Legacy = 0x43; break;	/* 打招呼   */
		case 'P': Legacy = 0x44; break;	/* 开灯     */
		case 'Q': Legacy = 0x45; break;	/* 关灯     */
		case 'R': Legacy = 0x46; break;	/* 开呼吸灯 */
		case 'S': Legacy = 0x47; break;	/* 关呼吸灯 */
		case 'T': Legacy = 0x48; break;	/* 伸懒腰   */
		case 'U': Legacy = 0x49; break;	/* 拉伸腿   */
		default:  return;				/* 'V' 电量 / 'W' 叫：暂不处理 */
	}

	Legacy_Cmd(Legacy, 0);
}
#endif

/* ===================================================================
 * 处理一条完整的 AI 帧
 * =================================================================== */
static void AI_HandleFrame(void)
{
	uint8_t i, n, Ack = 0x06;

	if (s_type == AI_CMD_PING)
	{
		g_dbgFrames++;
		AI_Link_Send(AI_CMD_PING, &Ack, 1);
		return;
	}

	/* 图片帧：数据 = [表情][动作][页号 0~7] + 128 字节点阵
	   电脑把中文渲染成整屏点阵，分 8 页发过来；收齐第 7 页后一次性上屏，
	   同时设置表情和动作，说完 8 秒后再恢复表情。 */
	if (s_type == AI_CMD_IMG)
	{
		uint8_t page;

		if (s_len < AI_IMG_MAX) return;
		page = s_data[2];
		if (page < 8)
		{
			for (i = 0; i < 128; i++)
				s_img[(uint16_t)page * 128 + i] = s_data[3 + i];

			if (page == 7)
			{
				if ((s_data[0] & 0x0F) <= 6)  Face_Mode   = s_data[0] & 0x0F;
				if ((s_data[1] & 0x0F) <= 15) Action_Mode = s_data[1] & 0x0F;

				OLED_Clear();
				OLED_ShowImage(0, 0, 128, 64, s_img);
				OLED_Update();

				s_saying = 1;
				s_sayStart = s_tickMs;
			}
		}

		if (page == 7)
		{
			g_dbgFrames++;
			AI_Link_Send(AI_CMD_PING, &Ack, 1);	/* 只回一次，避免刷屏 */
		}
		return;
	}

	/* 语音帧：数据是 GBK 编码的中文文本，转给 SYN6288 念出来 */
	if (s_type == AI_CMD_SPEAK)
	{
		if (s_len > 0)
			Syn6288_Speak(s_data, s_len);
		g_dbgFrames++;
		AI_Link_Send(AI_CMD_PING, &Ack, 1);
		return;
	}

	if (s_type == AI_CMD_TALK)
	{
		if (s_len < 3) return;
		if ((s_data[0] & 0x0F) <= 6)  Face_Mode   = s_data[0] & 0x0F;
		if ((s_data[1] & 0x0F) <= 15) Action_Mode = s_data[1] & 0x0F;
		if (s_data[2] >= 100)         SpeedDelay  = s_data[2];

		n = s_len - 3;
		for (i = 0; i < n && i < AI_TEXT_MAX; i++)
			s_text[i] = (char)s_data[3 + i];
		s_text[i] = '\0';
	}
	else if (s_type == AI_CMD_TEXT)
	{
		n = s_len;
		for (i = 0; i < n && i < AI_TEXT_MAX; i++)
			s_text[i] = (char)s_data[i];
		s_text[i] = '\0';
	}
	else
	{
		return;
	}

	g_dbgFrames++;

	if (s_text[0] != '\0')
		Pet_Say(s_text);		/* 有话要说，先把话显示出来 */
	else
		Face_Config();			/* 没话，直接刷表情 */

	AI_Link_Send(AI_CMD_PING, &Ack, 1);		/* 告诉 PC 已收到 */
}

/* ===================================================================
 * 逐字节解析
 * =================================================================== */
static void AI_ParseByte(uint8_t Byte)
{
	switch (s_state)
	{
		case AI_ST_SOF1:
			if (Byte == AI_SOF1) s_state = AI_ST_SOF2;
			else                 Legacy_Cmd(Byte, 1);	/* 不是帧头，按老指令处理 */
			break;

		case AI_ST_SOF2:
			if (Byte == AI_SOF2)      s_state = AI_ST_TYPE;
			else if (Byte == AI_SOF1) s_state = AI_ST_SOF2;
			else { s_state = AI_ST_SOF1; Legacy_Cmd(Byte, 1); }
			break;

		case AI_ST_TYPE:
			s_type  = Byte;
			s_sum   = Byte;
			s_state = AI_ST_LEN;
			break;

		case AI_ST_LEN:
			if (Byte > AI_IMG_MAX) { s_state = AI_ST_SOF1; break; }	/* 超长丢弃 */
			s_len   = Byte;
			s_sum  ^= Byte;
			s_idx   = 0;
			s_state = (s_len == 0) ? AI_ST_SUM : AI_ST_DATA;
			break;

		case AI_ST_DATA:
			s_data[s_idx++] = Byte;
			s_sum ^= Byte;
			if (s_idx >= s_len) s_state = AI_ST_SUM;
			break;

		case AI_ST_SUM:
			if (Byte == s_sum) AI_HandleFrame();
			s_state = AI_ST_SOF1;
			break;

		default:
			s_state = AI_ST_SOF1;
			break;
	}
}

/* ===================================================================
 * 波特率自动识别（调试用）—— 把每一档的"收了多少 / 命中多少"列成表
 *
 * 电脑端循环发 30 31 29 三个字节。
 *   cnt = 这一档收到的字节总数
 *   hit = 其中正好等于 0x30 / 0x31 / 0x29 的个数
 *
 * 判读方法：
 *   正确的那一档 → cnt 和发出去的数量一致，hit ≈ cnt（几乎全中）
 *   波特率偏快   → cnt 是发送量的整数倍，hit 很低（过采样出一堆 00/FF）
 *   波特率偏慢   → cnt 明显少于发送量，hit 很低（漏采）
 *
 * 走完一圈后自动停在得分最高的一档，屏幕保持显示这张表，方便拍照。
 * 扫描和结束后都【不执行动作】，避免舵机乱转把电源拉低拖垮蓝牙模块。
 * =================================================================== */
/* 波特率已实测确定为 115200（见 BlueTooth.c 的说明），扫描功能关闭。
   需要重新排查时把它改回 1 即可。 */
#define AI_BAUD_SWEEP	0

#if AI_BAUD_SWEEP

static const uint32_t s_baudList[] = {
	9600, 19200, 115200, 38400, 57600, 4800, 230400, 2400
};
#define AI_BAUD_N		(sizeof(s_baudList) / sizeof(s_baudList[0]))
#define AI_BAUD_HOLD	5000		/* 每档停留时间 ms */

static uint8_t	s_baudIdx  = 0;
static uint32_t	s_baudTick = 0;
static uint16_t	s_score[AI_BAUD_N];	/* 该档历史最好成绩：命中数 */
static uint16_t	s_cnt[AI_BAUD_N];	/* 该档历史最好成绩：收到的字节数 */
static uint16_t	s_winCnt   = 0;		/* 当前这一档窗口内收到多少 */
static uint16_t	s_winHit   = 0;		/* 当前这一档窗口内命中多少 */

void AI_Link_SetBaud(uint32_t Baud)
{
	USART_InitTypeDef u;

	USART_Cmd(USART3, DISABLE);
	u.USART_BaudRate            = Baud;
	u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	u.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	u.USART_Parity              = USART_Parity_No;
	u.USART_StopBits            = USART_StopBits_1;
	u.USART_WordLength          = USART_WordLength_8b;
	USART_Init(USART3, &u);
	USART_Cmd(USART3, ENABLE);
}

static void AI_BaudClearStat(void)
{
	g_dbgBleRx  = 0;
	g_dbgFrames = 0;
	g_dbgLegacy = 0;
#if AI_DEBUG_UI
	s_histCnt   = 0;
#endif
	s_head      = 0;
	s_tail      = 0;
	s_state     = AI_ST_SOF1;
}

static void AI_BaudSweep(void)
{
	if ((uint32_t)(s_tickMs - s_baudTick) < AI_BAUD_HOLD) return;
	s_baudTick = s_tickMs;

	/* 结算刚过去的这一档：只保留历史最好的一次，
	   这样电脑晚连上、或者某一档恰好没赶上数据，都不会污染结果 */
	if (s_winCnt > s_cnt[s_baudIdx])
	{
		s_cnt[s_baudIdx]   = s_winCnt;
		s_score[s_baudIdx] = s_winHit;
	}
	s_winCnt = 0;
	s_winHit = 0;

	/* 一直循环，不会停 */
	s_baudIdx = (uint8_t)((s_baudIdx + 1) % AI_BAUD_N);
	AI_Link_SetBaud(s_baudList[s_baudIdx]);
	AI_BaudClearStat();
}

#endif

/* ===================================================================
 * 调试屏：把链路状态直接打在屏幕上
 *
 *   B: 蓝牙收到的字节数      V: 语音收到的字节数
 *   H: 环形缓冲写指针        T: 读指针
 *      正常时 H >= T；H 涨而 T 不涨 = 主循环没在解析
 *   S: 解析状态机阶段        0=等帧头 1=等0x55 2=等类型 3=等长度
 *                            4=收数据 5=等校验
 *   LG: Legacy_Cmd 调用次数  涨了就说明字节确实被当成指令执行了
 *   FR: 完整帧数             MD: Action_Mode   TK: TIM3 秒数
 * =================================================================== */
#if AI_DEBUG_UI && !AI_BAUD_SWEEP
static void AI_DebugScreen(void)
{
	uint8_t i, n, idx;
	uint16_t base;

	OLED_Clear();

	/* 第 0 行：两路串口收到的字节数 */
	OLED_ShowString(0, 0, "B:", OLED_6X8);
	OLED_ShowNum(12, 0, g_dbgBleRx, 5, OLED_6X8);
	OLED_ShowString(60, 0, "V:", OLED_6X8);
	OLED_ShowNum(72, 0, g_dbgVoiceRx, 5, OLED_6X8);

	/* 第 1 行：缓冲头尾指针 + 状态机阶段 */
	OLED_ShowString(0, 8, "H:", OLED_6X8);
	OLED_ShowNum(12, 8, s_head, 3, OLED_6X8);
	OLED_ShowString(40, 8, "T:", OLED_6X8);
	OLED_ShowNum(52, 8, s_tail, 3, OLED_6X8);
	OLED_ShowString(84, 8, "S:", OLED_6X8);
	OLED_ShowNum(96, 8, s_state, 1, OLED_6X8);

	/* 第 2 行：老指令调用次数 + 完整帧数 */
	OLED_ShowString(0, 16, "LG:", OLED_6X8);
	OLED_ShowNum(18, 16, g_dbgLegacy, 5, OLED_6X8);
	OLED_ShowString(60, 16, "FR:", OLED_6X8);
	OLED_ShowNum(78, 16, g_dbgFrames, 4, OLED_6X8);

	/* 第 3 行：动作模式 + 计时 */
	OLED_ShowString(0, 24, "MD:", OLED_6X8);
	OLED_ShowNum(18, 24, Action_Mode, 2, OLED_6X8);
	OLED_ShowString(60, 24, "TK:", OLED_6X8);
	OLED_ShowNum(78, 24, (s_tickMs / 1000) % 100, 2, OLED_6X8);

	/* 第 4~6 行：最近收到的字节原文（十六进制），每行 6 个。
	   如果这些值不是 "30 31 29 AA 55 ..."（我们发的内容），
	   而是毫无规律的随机数，就说明是波特率不匹配产生的乱码。 */
	n = s_histCnt;
	if (n > 18) n = 18;
	base = (uint16_t)s_histCnt - n;
	for (i = 0; i < n; i++)
	{
		idx = (uint8_t)((base + i) % HIST_N);
		OLED_ShowHexNum((i % 6) * 21, 32 + (i / 6) * 8, s_hist[idx], 2, OLED_6X8);
	}

	OLED_Update();
}
#endif

#if AI_DEBUG_UI && AI_BAUD_SWEEP
/* 波特率扫描表：每档一行
 *   第 1 列 波特率   第 2 列 收到的字节数   第 3 列 命中 30/31/29 的个数
 *   "<" 标记当前正在试的那一档
 * 正确的那一档：收到的字节数 ≈ 发出去的数量，且命中数 ≈ 收到的数量。
 */
static void AI_DebugScreen(void)
{
	uint8_t i;

	OLED_Clear();
	for (i = 0; i < AI_BAUD_N; i++)
	{
		OLED_ShowNum(0,  i * 8, s_baudList[i], 6, OLED_6X8);
		OLED_ShowNum(42, i * 8, s_cnt[i],     3, OLED_6X8);
		OLED_ShowString(60, i * 8, "/", OLED_6X8);
		OLED_ShowNum(66, i * 8, s_score[i],   3, OLED_6X8);
		if (i == s_baudIdx) OLED_ShowString(84, i * 8, "<", OLED_6X8);
	}
	(void)s_hist[0];			/* 识别阶段用不到字节原文，这里只是保持变量被引用 */
	(void)s_histCnt;
	OLED_Update();
}
#endif

/* ===================================================================
 * 对外接口
 * =================================================================== */
void AI_Link_Init(void)
{
	s_head  = 0;
	s_tail  = 0;
	s_headV = 0;
	s_tailV = 0;
	s_state = AI_ST_SOF1;
	s_saying = 0;
	s_sayStart = 0;
	s_tickMs = 0;
	s_text[0] = '\0';

	g_dbgBleRx   = 0;
	g_dbgVoiceRx = 0;
	g_dbgFrames  = 0;
	g_dbgLegacy  = 0;
	g_dbgLastBle = 0;
#if AI_DEBUG_UI
	s_histCnt    = 0;
#endif

#if AI_BAUD_SWEEP
	/* 开机从扫描表第一档开始，每 5 秒换一档，一直循环 */
	{
		uint8_t i;
		s_baudIdx  = 0;
		s_baudTick = 0;
		s_winCnt   = 0;
		s_winHit   = 0;
		for (i = 0; i < AI_BAUD_N; i++)
		{
			s_score[i] = 0;
			s_cnt[i]   = 0;
		}
		AI_Link_SetBaud(s_baudList[0]);
	}
#endif
}

/* TIM3 每 20ms 进一次中断，这里只做加法，开销可以忽略 */
void AI_Link_TickIsr(void)
{
	s_tickMs += 20;
}

/* USART3（蓝牙）中断里调用：只入队，立刻返回 */
void AI_Link_RxByte(uint8_t Byte)
{
	uint16_t Next = (s_head + 1) % AI_RX_BUF_SIZE;

	g_dbgBleRx++;
	g_dbgLastBle = Byte;

	/* 记下最近 32 个字节的原文，调试屏会按十六进制打出来 */
#if AI_DEBUG_UI
	s_hist[s_histCnt % HIST_N] = Byte;
	s_histCnt++;
#endif

#if AI_BAUD_SWEEP
	/* 波特率识别统计：当前这档窗口内收到多少、其中多少个正好是 0x30 */
	if (s_winCnt < 60000) s_winCnt++;
	if (Byte == 0x30 && s_winHit < 60000) s_winHit++;
#endif

	/* 收到帧头说明 AI 要说话，顺手让阻塞中的运动循环尽快退出，
	   否则 Sustainedmove=1 时主循环会一直卡在 Action_advance 里 */
	if (Byte == AI_SOF1) Sustainedmove = 0;

	if (Next != s_tail)				/* 缓冲满则丢弃最迟到的字节 */
	{
		s_rx[s_head] = Byte;
		s_head = Next;
	}
}

/* USART1（语音模块）中断里调用：只入队，立刻返回。
   绝对不能在这里刷 OLED —— 这是把主循环饿死的老病根。 */
void AI_Link_RxVoiceByte(uint8_t Byte)
{
	uint16_t Next = (s_headV + 1) % sizeof(s_rxV);

	g_dbgVoiceRx++;
	if (Byte == AI_SOF1) Sustainedmove = 0;

	if (Next != s_tailV)
	{
		s_rxV[s_headV] = Byte;
		s_headV = Next;
	}
}

void AI_Link_Task(void)
{
#if AI_BAUD_SWEEP
	AI_BaudSweep();				/* 调试期：每 5 秒换一档波特率，走完一圈停在最好的一档 */
#endif

	/* 1. 把蓝牙缓冲里的字节全部解析掉
	      （识别阶段 Legacy_Cmd 会直接返回，不会真的动舵机） */
	while (s_tail != s_head)
	{
		uint8_t Byte = s_rx[s_tail];
		s_tail = (s_tail + 1) % AI_RX_BUF_SIZE;
		AI_ParseByte(Byte);
	}

	/* 2. 语音模块的字节按老指令处理（走完规定次数就停） */
	while (s_tailV != s_headV)
	{
		uint8_t Byte = s_rxV[s_tailV];
		s_tailV = (s_tailV + 1) % sizeof(s_rxV);
#if AI_VOICE_MAP
		Voice_Cmd(Byte);			/* 按 SU-03T 工程的 A~W 字母表解码 */
#else
		Legacy_Cmd(Byte, 0);
#endif
	}

	/* 3. 说完话停留计时，到点恢复表情
	      用无符号差值比较，计时器溢出也能正确工作 */
	if (s_saying && (uint32_t)(s_tickMs - s_sayStart) >= AI_SAY_MS)
	{
		s_saying = 0;
		Face_Config();
	}

#if AI_DEBUG_UI
	/* 4. 调试屏：每 300ms 刷一次。
	      正在显示文字时让位，否则刚画上去的字 300ms 就被调试屏盖掉了。 */
	{
		static uint32_t LastDraw = 0;
		if (!s_saying && (uint32_t)(s_tickMs - LastDraw) >= 300)
		{
			LastDraw = s_tickMs;
			AI_DebugScreen();
		}
	}
#endif
}
