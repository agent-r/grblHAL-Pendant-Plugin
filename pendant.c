/*

   TODO:    ->  when STOPPING by "grbl.enqueue_realtime_command(CMD_STOP);",
                iO-Sender does not recognize this. I have to manually click "STOP" in sender as well.

            ->  Btw: Can I stop a HOMING process somehow?


   TODO:    ->  I am using a handwheel with 100 ticks pre revolution.
                When turning the wheel I send 5 to 10 Jog-Commands per Second,
                processed by "grbl.enqueue_gcode("$J=G91X24F1000");".
                This works very good for single ticks and slow moves, but when turning the wheel fast,
                some commands get lost or are not processed. Transfer from Pendant to Plugin seems ok.
                Do I overflow a buffer or something? How can I check this, before sending enquee a command?
 */

#include "grbl/hal.h"
#include "networking/cJSON.h"
#include "grbl/protocol.h"
#include <stdio.h>
#include <string.h>

#define PENDANT_SERIAL_STREAM 0
#define PENDANT_SERIAL_BAUDRATE  115200
static io_stream_t *pendant_serial = NULL;

static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;
static on_state_change_ptr on_state_change;

#define INBUF_SIZE 128               // I do not expect larger commands
#define OUTBUF_SIZE 128             // I will not send larger State-Updates
static bool JSON_received = false;
static char JSON[INBUF_SIZE];

// #define AliveTicks 1000 * 2          // check for "OK" every 2 seconds
#define SendTicks 1000 / 8              // send 8 position updates per second (if position changes)
#define SendAlwaysTicks 1000 / 1        // send 1 update per second if nothing changes (keep alive!)

// static uint32_t CheckAliveMs = 0;
static uint32_t SendMs = 0;
static uint32_t SendAlwaysMs = 0;

// #define pendant_debug_in 1                   // debug inputs and outputs
// #define pendant_debug_in_raw 1
// #define pendant_debug_out 1

///////////////////////////////////////////////////////////////////////////////////////////

static void pendant_parse_and_send_cmd(const char * const cmd_buffer) {

        const cJSON *js_cmd = NULL;
        cJSON *cmd_json = cJSON_Parse(cmd_buffer);

        char SysExecuteCommand[LINE_BUFFER_SIZE];

        if (cJSON_HasObjectItem(cmd_json,"cmd")) {
                js_cmd = cJSON_GetObjectItemCaseSensitive(cmd_json,"cmd");
                if (cJSON_IsString(js_cmd) && (js_cmd->valuestring != NULL))
                {
                        const char * str_cmd = js_cmd->valuestring;
                        #ifdef pendant_debug_in
                                hal.stream.write("CMD:"); hal.stream.write(str_cmd); hal.stream.write(ASCII_EOL);
                        #endif

                        if (strcmp(str_cmd, "START") == 0)
                        {
                                grbl.enqueue_realtime_command(CMD_CYCLE_START);
                        }
                        else if (strcmp(str_cmd, "STOP") == 0)
                        {
                                // grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
                                grbl.enqueue_realtime_command(CMD_STOP);
                                // strncpy(SysExecuteCommand, "$", sizeof(SysExecuteCommand) - 1);
                                // system_execute_line(SysExecuteCommand);     // must be at least "LINE_BUFFER_SIZE" long ???
                        }
                        else if (strcmp(str_cmd, "HOME") == 0)
                        {
                                strncpy(SysExecuteCommand, "$H", sizeof(SysExecuteCommand) - 1);
                                system_execute_line(SysExecuteCommand);     // must be at least "LINE_BUFFER_SIZE" long ???
                                // system_execute_line("$H");
                        }
                        else if (strcmp(str_cmd, "UNLOCK") == 0)
                        {
                                // system_execute_line("$X");
                                strncpy(SysExecuteCommand, "$X", sizeof(SysExecuteCommand) - 1);
                                system_execute_line(SysExecuteCommand);     // must be at least "LINE_BUFFER_SIZE" long ???
                        }
                }
                // KeepAliveMs = hal.get_elapsed_ticks() + AliveTicks;
        }

        else if (cJSON_HasObjectItem(cmd_json,"gcode")) {
                js_cmd = cJSON_GetObjectItemCaseSensitive(cmd_json,"gcode");

                if (cJSON_IsString(js_cmd) && (js_cmd->valuestring != NULL))
                {
                        char * str_gcode = js_cmd->valuestring;
                        #ifdef pendant_debug_in
                                hal.stream.write("GCODE:"); hal.stream.write(str_gcode); hal.stream.write(ASCII_EOL);
                        #endif
                        grbl.enqueue_gcode(str_gcode);
                }
                // KeepAliveMs = hal.get_elapsed_ticks() + AliveTicks;
        }

        // this is for receivong messages (from esp-ble/wifi module debug, for example)
        else if (cJSON_HasObjectItem(cmd_json,"msg")) {
                js_cmd = cJSON_GetObjectItemCaseSensitive(cmd_json,"msg");

                if (cJSON_IsString(js_cmd) && (js_cmd->valuestring != NULL))
                {
                        const char * str_msg = js_cmd->valuestring;
                        hal.stream.write(str_msg); {hal.stream.write(ASCII_EOL);}
                }
                // KeepAliveMs = hal.get_elapsed_ticks() + AliveTicks;
        }
        /*
           else if (cJSON_HasObjectItem(cmd_json,"OK")) {
                KeepAliveMs = hal.get_elapsed_ticks() + AliveTicks;
                #ifdef pendant_debug_in
                        hal.stream.write("[OK]"); hal.stream.write(ASCII_EOL);
                #endif
           }
         */

        // This is a keep-alive transmission from the Pendant. If not received once in 2 seconds, stop sending state updates
        // else if (strcmp(cmd_buffer, "{\"OK\"}") == 0) {
        //         KeepAliveMs = hal.get_elapsed_ticks() + AliveTicks;
        //      #ifdef pendant_debug_in
        //              hal.stream.write("[OK]"); hal.stream.write(ASCII_EOL);
        //      #endif
        // }

        cJSON_Delete(cmd_json);
}


static void pendant_send(sys_state_t state, bool StateChange) {

        static char StateStr[10];

        if (pendant_serial && pendant_serial->type == StreamType_Serial) {

                if (StateChange) {           // state change is recognized!
                        switch (state) {
                        case STATE_IDLE:
                                strncpy(StateStr, "Idle", sizeof(StateStr) - 1); break;
                        case STATE_CYCLE:
                                strncpy(StateStr, "Idle", sizeof(StateStr) - 1); break;
                        case STATE_HOLD:
                                strncpy(StateStr, "Hold", sizeof(StateStr) - 1); break;
                        case STATE_JOG:
                                strncpy(StateStr, "Jogging", sizeof(StateStr) - 1); break;
                        case STATE_HOMING:
                                strncpy(StateStr, "Homing", sizeof(StateStr) - 1); break;
                        case STATE_ESTOP:
                                strncpy(StateStr, "Error", sizeof(StateStr) - 1); break;
                        case STATE_ALARM:
                                strncpy(StateStr, "Alarm", sizeof(StateStr) - 1); break;
                        case STATE_CHECK_MODE:
                                strncpy(StateStr, "Check", sizeof(StateStr) - 1); break;
                        case STATE_SAFETY_DOOR:
                                strncpy(StateStr, "Door", sizeof(StateStr) - 1); break;
                        case STATE_SLEEP:
                                strncpy(StateStr, "Sleep", sizeof(StateStr) - 1); break;
                        case STATE_TOOL_CHANGE:
                                strncpy(StateStr, "Tool", sizeof(StateStr) - 1); break;
                        default:
                                strncpy(StateStr, "N/A", sizeof(StateStr) - 1); break;
                        }
                }

                // get new position and compare to old position
                static int32_t int_pos[N_AXIS];
                static float float_pos[N_AXIS];
                static float float_pos_old[N_AXIS];
                static float wco[N_AXIS];

                memcpy(int_pos, sys.position, sizeof(sys.position));
                system_convert_array_steps_to_mpos(float_pos, int_pos);
                for (int i = 0; i < N_AXIS; i++) {
                        wco[i] = gc_get_offset(i, true);
                        float_pos[i] -= wco[i];
                }

                if (memcmp(float_pos, float_pos_old, sizeof(float_pos)) != 0) {
                        StateChange = true;  // We Send, because position has changed!
                        memcpy(float_pos_old, float_pos, sizeof(float_pos));
                }

                // prepare JSON String for Sending
                if (StateChange) {
                        char wifi_out_buffer[OUTBUF_SIZE];
                        if (N_AXIS == 3) { snprintf(wifi_out_buffer, sizeof(wifi_out_buffer), "{\"state\":\"%s\",\"wx\":%.3f,\"wy\":%.3f,\"wz\":%.3f}", StateStr, float_pos[0], float_pos[1], float_pos[2]); }
                        else if (N_AXIS == 4) { snprintf(wifi_out_buffer, sizeof(wifi_out_buffer), "{\"state\":\"%s\",\"wx\":%.3f,\"wy\":%.3f,\"wz\":%.3f,\"wa\":%.3f}", StateStr, float_pos[0], float_pos[1], float_pos[2], float_pos[3]); }
                        // else if (N_AXIS == 5) { snprintf(wifi_out_buffer, sizeof(wifi_out_buffer), "{\"state\":\"%s\",\"wx\":%.3f,\"wy\":%.3f,\"wz\":%.3f,\"wa\":%.3f,\"wb\":%.3f}"ASCII_EOL, string_state, float_pos[0], float_pos[1], float_pos[2], float_pos[3], float_pos[4]); }
                        
                        pendant_serial->write(wifi_out_buffer);
                        #ifdef pendant_debug_out
                                report_message(wifi_out_buffer, Message_Debug);
                        #endif
                }
        }
        else { report_message("PENDANT -> SEND -> STREAM IS NOT CONNECTED", Message_Error); }

}


static bool pendant_receive_callback(const uint8_t received_char) {

        static int in_buffer_i = 0;
        static char in_buffer[INBUF_SIZE];

        if (received_char == '{') {                                     // EXPECTING START OF NEW JSON STRING
                in_buffer_i = 0;
                in_buffer[in_buffer_i] = received_char;
        }
        else if (received_char == '}') {                                // EXPECTING END OF JSON STRING

                if (in_buffer_i < INBUF_SIZE - 2) {                      // CHECK SPACE FOR LAST CHARACTER + EOL
                        in_buffer_i++;
                        in_buffer[in_buffer_i] = received_char;
                        in_buffer[in_buffer_i+1] = '\0';
                        in_buffer_i = 0;                                 // PREPARE FOR NEXT DATA STRING
                        JSON_received = true;
                        memcpy(JSON, in_buffer, sizeof(in_buffer));
                } 
                else {
                        in_buffer_i = 0;                                 // Puffer voll, JSON verwerfen
                        // BESSER KEIN DEBUG IM CALLBACK! // if (pendant_debug_in) hal.stream.write("[JSON overflow]" ASCII_EOL);
                }
        }
        else {
                if (in_buffer_i < INBUF_SIZE - 2) {
                        in_buffer_i++;
                        in_buffer[in_buffer_i] = received_char;
                } else {
                        in_buffer_i = 0;                                 // Puffer voll, JSON verwerfen
                        // BESSER KEIN DEBUG IM CALLBACK! // // if (pendant_debug_in) hal.stream.write("[JSON overflow]" ASCII_EOL);
                }
        }

        return true;
}

static void pendant_loop (sys_state_t state)
{

        if (JSON_received) {                            // check if a complete json was received:
                #ifdef pendant_debug_in_raw 
                        report_message(JSON, Message_Debug);
                #endif
                JSON_received = false;                  // reset
                pendant_parse_and_send_cmd(JSON);       // send JSON to parser
        }


        uint32_t CurrentMs = hal.get_elapsed_ticks();
        // if (CurrentMs < CheckAliveMs) {          // only send if alive

                if((CurrentMs >= SendMs)) {     // if we have a tick ...
                        SendMs = CurrentMs + SendTicks;

                        if((CurrentMs >= SendAlwaysMs)) {   // ... and its a large tick ...
                                SendAlwaysMs = CurrentMs + SendAlwaysTicks;
                                pendant_send(state, true); // ... we always send!
                        }
                        else {
                                pendant_send(state, false); // else: its a small one: only send if something changed !

                        }
                }
        // }

        on_execute_realtime(state);
}

// report if state has changed (some states show up for only very short time - needed for probing-alarms)
// state has changes, so we update both timers aand send with "always_send=true"
static void state_changed(sys_state_t state) 
{
        pendant_send(state, true);
        if(on_state_change) { on_state_change(state); }
}


// time to set up serial stream
static void report_options (bool newopt)
{
        on_report_options(newopt);
        if(!newopt) { 
                
                report_message("PENDANT -> STARTED", Message_Info);

                if (!pendant_serial) {
                        pendant_serial = (io_stream_t *) stream_open_instance(PENDANT_SERIAL_STREAM, PENDANT_SERIAL_BAUDRATE, pendant_receive_callback, "Pendant");
                }

                char str[32];
                sprintf(str, "PENDANT -> STREAM INSTANCE %u", pendant_serial->instance);
                report_message(str, Message_Info);

                if (pendant_serial->is_connected) { report_message("PENDANT -> STREAM IS CONNECTED", Message_Info); }
                else { report_message("PENDANT -> STREAM IS NOT CONNECTED", Message_Error); }

                if (pendant_serial->type == StreamType_Serial)  { report_message("PENDANT -> STREAM IS SERIAL", Message_Info); }
                else { report_message("PENDANT -> STREAM IS NOT SERIAL", Message_Error); }

        }        
}


// initialize pendant
void pendant_init (void)
{

        // Add pendant_update function to grblHAL foreground process
        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = pendant_loop;

        // Add state change interrupt
        on_state_change = grbl.on_state_change;
        grbl.on_state_change = state_changed;

        // Add report
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

}
