/**
 *  @brief
 *      Board Support Package.
 *
 *		LEDs and switches.
 *  @file
 *      bsp.s
 *  @author
 *      Peter Schmid, peter@spyr.ch
 *  @date
 *      2020-03-26
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
		Wortbirne Flag_visible, "led1!"
set_led1:
		@ ( u --  ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	movs	r0, tos
	drop
	bl		BSP_setLED1
	pop		{pc}

@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "led1@"
get_led1:
		@ (  -- u ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	pushdatos
	bl		BSP_getLED1
	movs	tos, r0
	pop		{pc}

@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "led2!"
set_led2:
		@ ( u --  ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	movs	r0, tos
	drop
	bl		BSP_setLED2
	pop		{pc}

@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "led2@"
get_led2:
		@ (  -- u ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	pushdatos
	bl		BSP_getLED2
	movs	tos, r0
	pop		{pc}

	@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "led3!"
set_led3:
		@ ( u --  ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	movs	r0, tos
	drop
	bl		BSP_setLED3
	pop		{pc}

@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "led3@"
get_led3:
		@ (  -- u ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	pushdatos
	bl		BSP_getLED3
	movs	tos, r0
	pop		{pc}


@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "switch1?"
get_switch1:
		@ (  -- u ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	pushdatos
	bl		BSP_getSwitch1
	movs	tos, r0
	pop		{pc}


@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "switch2?"
get_switch2:
		@ (  -- u ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	pushdatos
	bl		BSP_getSwitch2
	movs	tos, r0
	pop		{pc}

@ -----------------------------------------------------------------------------
		Wortbirne Flag_visible, "switch3?"
get_switch3:
		@ (  -- u ) waits for a time period specified in kernel ticks
@ -----------------------------------------------------------------------------
	push	{lr}
	pushdatos
	bl		BSP_getSwitch3
	movs	tos, r0
	pop		{pc}


