/**
 *  @brief
 *      Interface to the CMSIS-RTOSv2 functions.

 *  @file
 *      rtos.s
 *  @author
 *      Peter Schmid, peter@spyr.ch
 *  @date
 *      2020-03-11
 *  @remark
 *      Language: ARM Assembler, STM32CubeIDE GCC
 *  @copyright
 *      Peter Schmid, Switzerland
 *
 *      This project Mecrsip-Cube is free software: you can redistribute it
 *      and/or modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation, either version 3 of
 *      the License, or (at your option) any later version.
 *
 *      Mecrsip-Cube is distributed in the hope that it will be useful, but
 *      WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *      General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with Mecrsip-Cube. If not, see http://www.gnu.org/licenses/.
 */

@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "osdelay"
rtos_osdelay:
		@ ( u -- n ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	movs	r0, tos
	bl		osDelay
	movs	tos, r0
	pop		{pc}


@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "osthreadnew"
rtos_osthreadnew:
		@ ( addr addr addr -- u ) Create a thread and add it to Active Threads.
// Create a thread and add it to Active Threads.
// \param[in]     func          thread function.
// \param[in]     argument      pointer that is passed to the thread function as start argument.
// \param[in]     attr          thread attributes; NULL: default values.
// \return thread ID for reference by other functions or NULL in case of error.
// osThreadId_t osThreadNew (osThreadFunc_t func, void *argument, const osThreadAttr_t *attr);
@ -----------------------------------------------------------------------------
	push	{lr}
	movs	r2, tos		// set attr
	drop
	movs	r1, tos		// set argument
	drop
	movs	r0, tos		// set func
	bl		osThreadNew
	movs	tos, r0
	pop		{pc}



@ -----------------------------------------------------------------------------
