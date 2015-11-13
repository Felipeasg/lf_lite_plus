/**************************************************************************/
/*!
    @file    cmdline_parser.c
    @author  Netanel (PACABOT)
    @date    02/05/2015
    @version 0.1
 */
/**************************************************************************/

/* General declarations */
#include "basetypes.h"
#include "config.h"
#include "errors.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include "uart1bsp.h"
#include "stm32f4xx_hal_uart.h"

/* Declarations for this module */
#include "cmdline_parser.h"
#include "commads.h"

#define MAX_COMMAND_LEN 100 + 1

/* Context of this module */
CMDLINE_CONTEXT cmdline_ctxt;
// Buffer used for Command line parser
extern char serial_buffer[100];

/* Private functions */
static CMD_HANDLER *cmdline_check_cmd(const char *cmd);


int cmdline_init(CMDLINE_CONTEXT *context)
{
    // Initialize Command line context
    cmdline_ctxt.cmdline = serial_buffer;
    cmdline_ctxt.cmd_received = FALSE;
    cmdline_ctxt.cmd_len = 0;

    if (context != NULL)
    {
        cmdline_ctxt.out = context->out;
    }
    else
    {
        cmdline_ctxt.out = cmd_output;
    }
    cmd_displayPrompt();
    __HAL_LOCK(&huart1);
    // Enable interrupts on UART3
    while (huart1.State != HAL_UART_STATE_READY);
    huart1.ErrorCode = HAL_UART_ERROR_NONE;

    /* Enable the UART Parity Error Interrupt */
//	__HAL_UART_ENABLE_IT(&huart3, UART_IT_PE);

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

    __HAL_UNLOCK(&huart1);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    return CMDLINE_PARSER_E_SUCCESS;
}

int cmdline_parse(void)
{
    const char  *params = NULL;
    CMD_HANDLER *hcmd = NULL;
    char        command[MAX_COMMAND_LEN];
    char        *cmd_end;
    int         cmd_len = 0;
    int         rv;

    if (cmdline_ctxt.cmd_received == FALSE)
    {
        return CMDLINE_PARSER_E_SUCCESS;
    }

    // Reset fields
    cmdline_ctxt.cmd_received = FALSE;

    // Search SPACE character
    cmd_end = strchr(cmdline_ctxt.cmdline, ' ');
    if (cmd_end == NULL)
    {
        // SPACE character not found. Search Carriage Return
        cmd_end = strchr(cmdline_ctxt.cmdline, CMDLINE_CR);
        if (cmd_end == NULL)
        {
            cmdline_ctxt.out("\rBad format");
            rv = CMDLINE_PARSER_E_CMD_NOT_FOUND;
            goto out;
        }
    }
    else
    {
        // Skip space character for parameters
        params = cmd_end + 1;
    }

    // Compute actual command length
    cmd_len = cmd_end - cmdline_ctxt.cmdline;

    // Do not return error if only Carriage Return has been received
    if ((cmd_len == 0) || cmdline_ctxt.cmdline[0] == ' ')
    {
        rv = CMDLINE_PARSER_E_SUCCESS;
        goto out;
    }

    memcpy(command, cmdline_ctxt.cmdline, cmd_len);
    command[cmd_len] = '\0';

    /****************
     * Parse command
     ****************/
    /* Check if command exists */
    hcmd = cmdline_check_cmd(command);
    if ((hcmd == NULL) || (hcmd->pCmdCallback == NULL))
    {
        cmdline_ctxt.out("\rCommand '%s' does not exist", command);
        /* Command does not exist */
        rv = CMDLINE_PARSER_E_UNKNOWN_CMD;
        goto out;
    }

    /******************
     * Execute command
     ******************/
    rv = hcmd->pCmdCallback(params);

out:
    // Reset Command line buffer
    memset(cmdline_ctxt.cmdline, 0x00, cmdline_ctxt.cmd_len);
    // Display back prompt message
    cmd_displayPrompt();
    return rv;
}

void cmdline_setCmdReceived(int status, int cmd_length)
{
    if ((status != TRUE) && (status != FALSE))
    {
        return;
    }
    cmdline_ctxt.cmd_received = status;
}


/**
 * @brief   Checks if command exists
 *
 * @param   cmd The command to search
 *
 * @retval  A pointer to a CMD_HANDLER on success, NULL otherwise
 */
static CMD_HANDLER *cmdline_check_cmd(const char *cmd)
{
    CMD_HANDLER *hcmd = (CMD_HANDLER *)cmd_handlers;

    /* Walk through commands array */
    while(hcmd->command != NULL)
    {
        if (strcmp(hcmd->command, cmd) == 0)
        {
            /* Command found, return a pointer to the current command handler */
            return hcmd;
        }
        hcmd++;
    }
    return NULL;
}
