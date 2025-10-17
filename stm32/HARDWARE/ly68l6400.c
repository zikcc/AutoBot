#include "ly68l6400.h"

extern QSPI_HandleTypeDef hqspi;

HAL_StatusTypeDef QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
    QSPI_CommandTypeDef sCommand = {0};

    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction = 0x35; // 写使能命令
    sCommand.AddressMode = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = QSPI_DATA_NONE;
    sCommand.DummyCycles = 0;
    sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    return HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
}


HAL_StatusTypeDef QSPI_ReadID(QSPI_HandleTypeDef *hqspi, uint8_t *id)
{
    QSPI_CommandTypeDef sCommand = {0};

    // 配置 QSPI 命令
    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE; // 使用 1 条线发送指令
    sCommand.Instruction = 0x9F;                        // JEDEC ID 指令
    sCommand.AddressMode = QSPI_ADDRESS_1_LINE;           // 不使用地址
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = QSPI_DATA_1_LINE;               // 使用 1 条线接收数据
    sCommand.NbData = 8;                                // JEDEC ID 通常为 3 字节
    sCommand.DummyCycles = 0;                           // 不需要空周期
    sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;           // 禁用 DDR 模式
    sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;       // 每次指令发送 SIOO 模式

    // 发送命令并接收数据
    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return HAL_ERROR;

    return HAL_QSPI_Receive(hqspi, id, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
}

HAL_StatusTypeDef QSPI_Read(QSPI_HandleTypeDef *hqspi, uint32_t address, uint8_t *buffer, uint16_t length)
{
    QSPI_CommandTypeDef sCommand = {0};

    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction = 0xEB; // 快速读命令 (Quad I/O)
    sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
    sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
    sCommand.Address = address;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = QSPI_DATA_4_LINES;
    sCommand.NbData = length;
    sCommand.DummyCycles = 6; // 典型的 QSPI 需要 8 个虚拟时钟周期
    sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return HAL_ERROR;

    return HAL_QSPI_Receive(hqspi, buffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
}

HAL_StatusTypeDef QSPI_Write(QSPI_HandleTypeDef *hqspi, uint32_t address, uint8_t *buffer, uint16_t length)
{
    QSPI_CommandTypeDef sCommand = {0};

	HAL_QSPI_Abort(hqspi);

    sCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    sCommand.Instruction = 0x38; // Quad Page Program 指令
    sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
    sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
    sCommand.Address = address;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = QSPI_DATA_4_LINES;
    sCommand.NbData = length;
    sCommand.DummyCycles = 0;
    sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return HAL_ERROR;

    return HAL_QSPI_Transmit(hqspi, buffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
}


void LY68_Write_Read_Check()
{
	uint8_t LYbuffer[256];
	uint8_t LYreadBuffer[256];
	// 写入数据
	for (int i = 0; i < 256; i++) {
		LYbuffer[i] = i+7;
	}

	if (QSPI_Write(&hqspi, 0x000000, LYbuffer, 256) != HAL_OK) {
		// 写入失败处理
		LYbuffer[0]=2;
	}

	// 读取数据
	if (QSPI_Read(&hqspi, 0x000000, LYreadBuffer, 256) != HAL_OK) {
		// 读取失败处理
		LYbuffer[0]=2;
	}

	// 验证数据
	for (int i = 0; i < 256; i++) {
		if (LYbuffer[i] != LYreadBuffer[i]) {
			// 数据不匹配，处理错误
	}
	}
}

void LY68L6400_EnableMemoryMappedMode(void)
{
    QSPI_CommandTypeDef sCommand = {0};
    QSPI_MemoryMappedTypeDef sMemMappedCfg = {0};

    // 配置 QSPI 命令
    sCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;  // 使用 1 线发送指令
    sCommand.Instruction = 0xEB;        // Quad IO Fast Read 指令
    sCommand.AddressMode = QSPI_ADDRESS_4_LINES;         // 使用 4 线模式传输地址
    sCommand.AddressSize = QSPI_ADDRESS_24_BITS;         // 地址大小设置为 24 位
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = QSPI_DATA_4_LINES;               // 使用 4 线模式传输数据
    sCommand.DummyCycles = 6;                            // 根据数据手册设置虚拟周期
    sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;            // 关闭 DDR 模式
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_HALF_CLK_DELAY;
    sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;        // 每个指令都发送指令

    // 配置 QSPI 内存映射模式
    sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE; // 关闭超时计数
	sMemMappedCfg.TimeOutPeriod     = 0;
    // 启用内存映射模式
    if (HAL_QSPI_MemoryMapped(&hqspi, &sCommand, &sMemMappedCfg) != HAL_OK)
    {
        // 错误处理
        Error_Handler();
    }
}

///**
// * @brief  从QSPI存储器中读取大量数据.
// * @note   改指令只能使用在50MHz一下，本配置下不好用
// * @param  pData: 指向要读取的数据的指针
// * @param  ReadAddr: 读取起始地址
// * @param  Size: 要读取的数据大小
// * @retval QSPI存储器状态
// */
//uint8_t W25Qx_QSPI_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
//{
//	QSPI_CommandTypeDef s_command;
//	/* 初始化读命令 */
//	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
//	s_command.Instruction = READ_CMD;    //READ_CMD;
//	s_command.AddressMode = QSPI_ADDRESS_1_LINE;
//	s_command.AddressSize = QSPI_ADDRESS_24_BITS;
//	s_command.Address = ReadAddr;
//	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//	s_command.DataMode = QSPI_DATA_1_LINE;
//	s_command.DummyCycles = 0;
//	s_command.NbData = Size;
//	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
//	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
//	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

//	/* 配置命令 */
//	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
//			!= HAL_OK)
//	{
//		return QSPI_ERROR;
//	}

//	/* 接收数据 */
//	if (HAL_QSPI_Receive(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
//			!= HAL_OK)
//	{
//		return QSPI_ERROR;
//	}
//	return QSPI_OK;
//}

/**
 * @brief  将大量数据写入QSPI存储器
 * @param  pData: 指向要写入数据的指针
 * @param  WriteAddr: 写起始地址
 * @param  Size: 要写入的数据大小
 * @retval QSPI存储器状态
 */
uint8_t QSPI_WriteManyData(uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
	QSPI_CommandTypeDef s_command;
	uint32_t end_addr, current_size, current_addr;
	/* 计算写入地址和页面末尾之间的大小 */
	current_addr = 0;
	
	HAL_QSPI_Abort(&hqspi);

	while (current_addr <= WriteAddr)
	{
		current_addr += 256;//一次256 bytes
	}
	current_size = current_addr - WriteAddr;

	/* 检查数据的大小是否小于页面中的剩余位置 */
	if (current_size > Size)
	{
		current_size = Size;
	}

	/* 初始化地址变量 */
	current_addr = WriteAddr;
	end_addr = WriteAddr + Size;

	/* 初始化程序命令 */
	s_command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
	s_command.Instruction = 0x38;
	s_command.AddressMode = QSPI_ADDRESS_4_LINES;
	s_command.AddressSize = QSPI_ADDRESS_24_BITS;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_4_LINES;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
//	s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	/* 逐页执行写入 */
	do
	{
		s_command.Address = current_addr;
		s_command.NbData = current_size;

//		/* 启用写操作 */
//		if (W25Qx_QSPI_WriteEnable() != QSPI_OK)
//		{
//			return QSPI_ERROR;
//		}

		/* 配置命令 */
		if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
				!= HAL_OK)
		{
			return HAL_ERROR;
		}

		/* 传输数据 */
		if (HAL_QSPI_Transmit(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
				!= HAL_OK)
		{
			return HAL_ERROR;
		}

//		/* 配置自动轮询模式等待程序结束 */
//		if (W25Qx_QSPI_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
//		{
//			return HAL_ERROR;
//		}

		/* 更新下一页编程的地址和大小变量 */
		current_addr += current_size;
		pData += current_size;
		current_size =
				((current_addr + 256) > end_addr) ?
						(end_addr - current_addr) : 256;
	} while (current_addr < end_addr);
	return HAL_OK;
}











