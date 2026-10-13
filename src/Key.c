#include "main.h"
#include "Key.h"


#define KEY_PRESSED				1
#define KEY_UNPRESSED			0

#define KEY_TIME_DOUBLE			20
#define KEY_TIME_LONG			200
#define KEY_TIME_REPEAT			10

uint8_t Key_Flag[KEY_COUNT];

/*
 * 矩阵行扫描：逐行拉低，读三列，得到 9 个键的按下状态，写入 state[9]。
 * state 下标 = row*3 + col，与 KEY_TO_A/KEY_TO_B/KEY_TO_C 等编号一致。
 * 三行引脚可能跨口（Row_1=PE10 / Row_2=PB2 / Row_3=PE7），逐口独立扫描。
 */
static void Key_ScanMatrix(uint8_t state[KEY_COUNT])
{
	uint8_t r, c;
	GPIO_PinState col;

	/* 行/列引脚（引脚可能跨不同 GPIO 口，行/列各用独立 port 数组对齐 pin） */
	const uint16_t row_pin[KEY_ROWS]  = { Key_Row_1_Pin, Key_Row_2_Pin, Key_Row_3_Pin };
	const uint16_t col_pin[KEY_COLS]  = { Key_Col_1_Pin, Key_Col_2_Pin, Key_Col_3_Pin };
	GPIO_TypeDef *row_port[KEY_ROWS]  = { Key_Row_1_GPIO_Port, Key_Row_2_GPIO_Port, Key_Row_3_GPIO_Port };
	GPIO_TypeDef *col_port[KEY_COLS]  = { Key_Col_1_GPIO_Port, Key_Col_2_GPIO_Port, Key_Col_3_GPIO_Port };

	for (r = 0; r < KEY_ROWS; r++)
	{
		/* 1. 当前行拉低，其余行拉高（三行可能跨口，逐口单独写） */
		for (uint8_t rr = 0; rr < KEY_ROWS; rr++)
		{
			HAL_GPIO_WritePin(row_port[rr], row_pin[rr],
					(rr == r) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		}

		/* 2. 读三列，列被拉低 = 该键按下 */
		for (c = 0; c < KEY_COLS; c++)
		{
			col = HAL_GPIO_ReadPin(col_port[c], col_pin[c]);
			state[r * KEY_COLS + c] = (col == GPIO_PIN_RESET) ? KEY_PRESSED : KEY_UNPRESSED;
		}
	}

	/* 扫描结束，所有行恢复高电平，避免误触 */
	for (uint8_t rr = 0; rr < KEY_ROWS; rr++)
	{
		HAL_GPIO_WritePin(row_port[rr], row_pin[rr], GPIO_PIN_SET);
	}
}

uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
	if (Key_Flag[n] & Flag)
	{
		if (Flag != KEY_HOLD)
		{
			Key_Flag[n] &= ~Flag;
		}
		return 1;
	}
	return 0;
}

void Key_Tick(void)
{
	static uint8_t Count, i;
	static uint8_t CurrState[KEY_COUNT], PrevState[KEY_COUNT];
	static uint8_t S[KEY_COUNT];
	static uint16_t Time[KEY_COUNT];

	for (i = 0; i < KEY_COUNT; i ++)
	{
		if (Time[i] > 0)
		{
			Time[i] --;
		}
	}

	Count ++;
	if (Count >= 2)
	{
		Count = 0;

		/* 一次矩阵扫描，得到 9 键按下状态 */
		uint8_t ScanState[KEY_COUNT];
		Key_ScanMatrix(ScanState);

		for (i = 0; i < KEY_COUNT; i ++)
		{
			PrevState[i] = CurrState[i];
			CurrState[i] = ScanState[i];

			if (CurrState[i] == KEY_PRESSED)
			{
				Key_Flag[i] |= KEY_HOLD;
			}
			else
			{
				Key_Flag[i] &= ~KEY_HOLD;
			}

			if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
			{
				Key_Flag[i] |= KEY_DOWN;
			}

			if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
			{
				Key_Flag[i] |= KEY_UP;
			}

			if (S[i] == 0)
			{
				if (CurrState[i] == KEY_PRESSED)
				{
					Time[i] = KEY_TIME_LONG;
					S[i] = 1;
				}
			}
			else if (S[i] == 1)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					Time[i] = KEY_TIME_DOUBLE;
					S[i] = 2;
				}
				else if (Time[i] == 0)
				{
					Time[i] = KEY_TIME_REPEAT;
					Key_Flag[i] |= KEY_LONG;
					S[i] = 4;
				}
			}
			else if (S[i] == 2)
			{
				if (CurrState[i] == KEY_PRESSED)
				{
					Key_Flag[i] |= KEY_DOUBLE;
					S[i] = 3;
				}
				else if (Time[i] == 0)
				{
					Key_Flag[i] |= KEY_SINGLE;
					S[i] = 0;
				}
			}
			else if (S[i] == 3)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					S[i] = 0;
				}
			}
			else if (S[i] == 4)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					S[i] = 0;
				}
				else if (Time[i] == 0)
				{
					Time[i] = KEY_TIME_REPEAT;
					Key_Flag[i] |= KEY_REPEAT;
					S[i] = 4;
				}
			}
		}
	}
}
