/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "MLX90640_I2C_Driver.h"

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
	int ack = HAL_I2C_Mem_Read(&hi2c1, (slaveAddr<<1), startAddress, I2C_MEMADD_SIZE_16BIT, (uint8_t*)data, nMemAddressRead*2, 500);

	if (ack != HAL_OK)
	{
    return -1;
	}
  
  for (uint16_t i = 0; i < nMemAddressRead; i++)
  {
    data[i] = __REV16(data[i]);
  }
  
	return 0;   
} 


int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{

	uint8_t sa = (slaveAddr << 1);
	uint8_t cmd[2];
	uint16_t dataCheck;

	cmd[0] = data >> 8;
	cmd[1] = data & 0x00FF;

	int ack = HAL_I2C_Mem_Write(&hi2c1, sa, writeAddress, I2C_MEMADD_SIZE_16BIT, cmd, sizeof(cmd), 500);

	if (ack != HAL_OK)
	{
			return -1;
	}         
	
	MLX90640_I2CRead(slaveAddr,writeAddress,1, &dataCheck);
	
	if ( dataCheck != data)
	{
			return -2;
	}    
	
	return 0;
}