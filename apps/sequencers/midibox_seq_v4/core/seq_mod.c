// Mapping of HID boards to SRIO lines:
//
// SRIO 0-3:    Board @I2C 0x23 (rightmost)
// SRIO 4-7:    Board @I2C 0x22
// SRIO 8-11:   Board @I2C 0x21
// SRIO 12-15:  Board @I2C 0x20 (leftmost)
// SRIO 16-19:  Board @I2C 0x08 (keypad)
// SRIO 20-23:  Virtual SRIO for GP LEDs (R Color 1, L Color 1, R Color 2, L Color 2)
// SRIO 24-27:  Virtual SRIO for TRACK LEDs (R Color 1, L Color 1, R Color 2, L Color 2)
#include <mios32.h>

// this module can be optionally disabled in a local mios32_config.h file (included from mios32.h)
#if defined(SEQ_USE_MOD)

#include <FreeRTOS.h>
#include <task.h>
#include "app.h"
#include "seq_hwcfg.h"

#define PRIORITY_TASK_SCAN_HID_BOARDS (tskIDLE_PRIORITY + 3)

#include <string.h>


int encoder_value[SEQ_HWCFG_NUM_ENCODERS];
int encoder_reported_value[SEQ_HWCFG_NUM_ENCODERS];


// Because of detented encoders, normally, packs of 4 events are received.
// Convert these "low-level" events to "high-level" delta events: 1 event per pack.
void handle_encoder_event(u8 encoder_index, s8 delta) {
    int new_value = encoder_value[encoder_index] + delta;
    int new_reported_value = new_value / 4;
    int prev_reported_value = encoder_reported_value[encoder_index];
    if (new_reported_value != prev_reported_value) {
        // GP encoders are 1..16
        APP_ENC_NotifyChange(encoder_index, new_reported_value - prev_reported_value);
        encoder_reported_value[encoder_index] = new_reported_value;
    }
    encoder_value[encoder_index] = new_value;
}


s32 i2c_transfer(u8 port, u8 iic_addr, u8 *buffer, size_t size, mios32_iic_transfer_t transfer_type) {
    s32 status;

    if((status=MIOS32_IIC_TransferBegin(port, IIC_Blocking)) < 0)
        MIOS32_MIDI_SendDebugMessage("IIC%d: MIOS32_IIC_TransferBegin failed with %d!\n", port, status);
    else {
        if((status=MIOS32_IIC_Transfer(port, transfer_type, iic_addr, buffer, size)) < 0)
            MIOS32_MIDI_SendDebugMessage("IIC%d: MIOS32_IIC_Transfer failed with %d!\n", port, status);
        else {
            if((status=MIOS32_IIC_TransferWait(port)) < 0)
                MIOS32_MIDI_SendDebugMessage("IIC%d: MIOS32_IIC_TransferWait failed with %d!\n", port, status);
            else
                MIOS32_IIC_TransferFinished(port);
        }
    }

    return status;
}


/**
 * Receive events from the board.
 * @param base_sr base SR for the board; because of strange addressing, corresponds to rightmost column of the board.
 * @return received event (0 = no event)
 */
u8 receive0(u8 iic_addr) {
    uint8_t event;
    s32 status = -1;
    while (status < 0) {
        if ((status = i2c_transfer(
                MIOS32_IIC_IO_PORT,
                iic_addr << 1U,
                (u8 *) &event,
                sizeof(event),
                IIC_Read)) >= 0)
        {
            return event;
        } else {
            MIOS32_MIDI_SendDebugMessage("WARNING: Could not receive via I2C.\n");
        }
    }
    return 0;
}


void receive_and_handle_v1(u8 iic_addr, u8 base_sr, u8 base_encoder) {
    u8 event = receive0(iic_addr);
    // event is 0 when nothing has happened
    if (event != 0) {
        if (event & 0x80U) {
            // button event
            u8 pressed = event & 1U;    // bug in firmware, Pressed was supposed to have lower bit set to 0
            u8 button = (event & 0x7FU) >> 1U;
            u8 column = button >> 2U;
            u8 row = button & 3U;
            u32 sr = base_sr + 3 - column;
            u32 pin = (sr << 3U) | row;
            APP_DIN_NotifyToggle(pin, pressed ? 0 : 1);
        } else {
            // encoder event
            u8 delta_bits = (event & 0x1FU);
            u8 encoder = event >> 5U;
            s8 delta = delta_bits < 16 ? delta_bits : delta_bits - 32;
            handle_encoder_event(base_encoder + encoder, delta);
        }
    }
}

void receive_and_handle_v2(u8 iic_addr, u8 base_sr, u8 base_encoder) {
    u8 event = receive0(iic_addr);
    // event is 0 when nothing has happened
    if (event != 0) {
        if (event & 0x80U) {
            // button event
            u8 depressed = event & 1U;
            u8 button = (event & 0x7FU) >> 1U;
            u8 column = button >> 3U;
            u8 row = button & 0b00000111U;
            u32 sr = base_sr + column;  // Take care of SR reversal!
            u32 pin = (sr << 3U) | row;
            APP_DIN_NotifyToggle(pin, depressed);
        } else {
            // encoder event
            u8 delta_bits = (event & 0x1FU);
            u8 encoder = event >> 5U;   // 2 bits for encoder number, highest bit is 0
            s8 delta = delta_bits < 16 ? delta_bits : delta_bits - 32;
            handle_encoder_event(base_encoder + encoder, delta);
        }
    }
}


void send0(u8 iic_addr, u8 base_sr) {
    volatile u8 *buffer = (volatile u8 *) (&mios32_srio_dout[0][0] + MIOS32_SRIO_NUM_SR - 4 - base_sr);
    i2c_transfer(
            MIOS32_IIC_IO_PORT,
            iic_addr << 1U,
            buffer,
            MIOS32_IIC_IO_ROWS_PER_BOARD,
            IIC_Write);
}


void transpose(u8 src_srio, u8 dst_srio, u8 bit) {
    u8 value = mios32_srio_dout[0][MIOS32_SRIO_NUM_SR - src_srio - 1];
    u8 *dst = &mios32_srio_dout[0][MIOS32_SRIO_NUM_SR - dst_srio - 1];
    u8 set_mask = 1U << bit;
    u8 clear_mask = ~set_mask;

    if (value & 1U) *dst |= set_mask; else *dst &= clear_mask;
    --dst;
    value >>= 1U;

    if (value & 1U) *dst |= set_mask; else *dst &= clear_mask;
    --dst;
    value >>= 1U;

    if (value & 1U) *dst |= set_mask; else *dst &= clear_mask;
    --dst;
    value >>= 1U;

    if (value & 1U) *dst |= set_mask; else *dst &= clear_mask;
    --dst;
    value >>= 1U;

    if (value & 1U) *dst |= set_mask; else *dst &= clear_mask;
    --dst;
    value >>= 1U;

    if (value & 1U) *dst |= set_mask; else *dst &= clear_mask;
    --dst;
    value >>= 1U;

    if (value & 1U) *dst |= set_mask; else *dst &= clear_mask;
    --dst;
    value >>= 1U;

    if (value & 1U) *dst |= set_mask; else *dst &= clear_mask;
}



static void TASK_CommunicateWithHIDBoards(void *pvParameters)  {
    portTickType xLastExecutionTime;

    // Initialise the xLastExecutionTime variable on task entry
    xLastExecutionTime = xTaskGetTickCount();

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while(1)  {
        vTaskDelayUntil(&xLastExecutionTime, 10 / portTICK_RATE_MS);

        receive_and_handle_v1((MIOS32_IIC_IO_BASE_ADDRESS + 0), 12, 1);
        receive_and_handle_v1((MIOS32_IIC_IO_BASE_ADDRESS + 1), 8, 5);
        receive_and_handle_v1((MIOS32_IIC_IO_BASE_ADDRESS + 2), 4, 9);
        receive_and_handle_v1((MIOS32_IIC_IO_BASE_ADDRESS + 3), 0, 13);
        receive_and_handle_v2(MIOS32_IIC_IO_KEYPAD_ADDRESS, 16, 0);

        // Copy bits from "virtual" SRIO for TRACK and GP LEDs into "real" SRIO
        // Bits: 0=round buttons; 2,3,4=Tracks B,G,R; 5,6,7=GP B,G,R
        transpose(20, 0, 6);
        transpose(21, 8, 6);
        transpose(22, 0, 5);
        transpose(23, 8, 5);
        transpose(24, 0, 3);
        transpose(25, 8, 3);
        transpose(26, 0, 2);
        transpose(27, 8, 2);

        send0(MIOS32_IIC_IO_BASE_ADDRESS + 0, 12);
        send0(MIOS32_IIC_IO_BASE_ADDRESS + 1, 8);
        send0(MIOS32_IIC_IO_BASE_ADDRESS + 2, 4);
        send0(MIOS32_IIC_IO_BASE_ADDRESS + 3, 0);
        send0(MIOS32_IIC_IO_KEYPAD_ADDRESS, 16);
    }
#pragma clang diagnostic pop
}


s32 SEQ_Mod_Idle(void)
{
    return 0;
}


/////////////////////////////////////////////////////////////////////////////
// Parser for a complete line
// Returns > 0 if command line matches with one of supported terminal commands
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_Mod_TerminalParseLine(char *input, void *_output_function)
{
    void (*out)(char *format, ...) = _output_function;
    char *separators = " \t";
    char *brkt;
    char *parameter;
/*

    // since strtok_r works destructive (separators in *input replaced by NUL), we have to restore them
    // on an unsuccessful call (whenever this function returns < 1)
    int input_len = strlen(input);

    if ((parameter = strtok_r(input, separators, &brkt))) {
        return 0;
    }

    // restore input line (replace NUL characters by spaces)
    int i;
    char *input_ptr = input;
    for(i=0; i<input_len; ++i, ++input_ptr)
        if( !*input_ptr )
            *input_ptr = ' ';
*/

    return 0; // command not taken
}


/////////////////////////////////////////////////////////////////////////////
// Function to initialize SEQ Modifications
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_Mod_Init(u32 mode)
{
    for (int i = 0; i < SEQ_HWCFG_NUM_ENCODERS; i++) {
        encoder_value[i] = 0;
        encoder_reported_value[i] = 0;
    }
    
    MIOS32_IIC_Init(0);

    xTaskCreate(
            TASK_CommunicateWithHIDBoards,
            (const portCHAR *) "ScanHIDBoards",
            configMINIMAL_STACK_SIZE,
            NULL,
            PRIORITY_TASK_SCAN_HID_BOARDS,
            NULL
    );

    return 0;
}


#endif /* SEQ_USE_MOD */
