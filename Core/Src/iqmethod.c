/*
 * iqmethod.c
 *
 *  Created on: Feb 28, 2025
 *      Author: ogahe
 */

#include "iqmethod.h"

void Align_Beginning(uint32_t data[], size_t len)
{
	int32_t buff[len];
	for(size_t i = 0; i < len; i++)
	{
		buff[i] = (int32_t) data[i];
	}

	// Remove Offsets
	for(size_t i = 0; i < len; i++)
	{
		buff[i] -= 2048;
	}
}
