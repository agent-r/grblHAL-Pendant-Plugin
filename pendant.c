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
#include "pendant.h"

static io_stream_t pendant_serial; 
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;
static on_state_change_ptr on_state_change;
// #define OVERRIDE_BUFSIZE 1024        // does it help??


#define INBUF_SIZE 128               // I do not expect larger commands
#define OUTBUF_SIZE 128             // I will not send larger State-Updates
// #define STATEBUF_SIZE 8

#define AliveTicks 1000 * 2         // check for "OK" every 2 seconds
#define SendTicks 1000 / 8           // send 5 position updates per second (if position changes)
#define SendAlwaysTicks 1000 / 1    // send 1 update per second if nothing changes (keep alive!)
#define ReceiveTicks 1000 / 10        // receive commands 10 times per second
static uint32_t AliveMs = 0;
static uint32_t SendMs = 0;
static uint32_t SendAlwaysMs = 0;
static uint32_t ReceiveMs = 0;

#define pendant_debug_in 0                   // debug inputs and outputs
#define pendant_debug_in_raw 0
#define pendant_debug_out 0

/*
   const char StateStrings[12][8] = {
        "Idle",
        "Run",
        "Hold",
        "Jog",
        "Home",
        "EndStop",
        "Alarm",
        "Check",
        "Door",
        "Sleep",
        "Tool",
        "N/A"
   };
 */

///////////////////////////////////////////////////////////////////////////////////////////

static void pendant_parse_and_send_cmd(const char * const cmd_buffer) {

        const cJSON *js_cmd = NULL;
        const cJSON *cmd_json = cJSON_Parse(cmd_buffer);

        char SysExecuteCommand[LINE_BUFFER_SIZE];  // 257

        if (cJSON_HasObjectItem(cmd_json,"cmd")) {
                js_cmd = cJSON_GetObjectItemCaseSensitive(cmd_json,"cmd");
                if (cJSON_IsString(js_cmd) && (js_cmd->valuestring != NULL))
                {
                        const char * str_cmd = js_cmd->valuestring;
                        if (pendant_debug_in) {hal.stream.write("CMD:"); hal.stream.write(str_cmd); hal.stream.write(ASCII_EOL);}

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
                AliveMs = hal.get_elapsed_ticks() + AliveTicks;
        }

        else if (cJSON_HasObjectItem(cmd_json,"gcode")) {
                js_cmd = cJSON_GetObjectItemCaseSensitive(cmd_json,"gcode");

                if (cJSON_IsString(js_cmd) && (js_cmd->valuestring != NULL))
                {
                        char * str_gcode = js_cmd->valuestring;
                        if (pendant_debug_in) { hal.stream.write("GCODE:"); hal.stream.write(str_gcode); hal.stream.write(ASCII_EOL); }
                        grbl.enqueue_gcode(str_gcode);
                }
                AliveMs = hal.get_elapsed_ticks() + AliveTicks;
        }

        // this is for receivong messages (from wifi module debug, for example)
        else if (cJSON_HasObjectItem(cmd_json,"msg")) {
                js_cmd = cJSON_GetObjectItemCaseSensitive(cmd_json,"msg");

                if (cJSON_IsString(js_cmd) && (js_cmd->valuestring != NULL))
                {
                        const char * str_msg = js_cmd->valuestring;
                        hal.stream.write(str_msg); {hal.stream.write(ASCII_EOL);}
                }
                AliveMs = hal.get_elapsed_ticks() + AliveTicks;
        }
        /*
           else if (cJSON_HasObjectItem(cmd_json,"OK")) {
                AliveMs = hal.get_elapsed_ticks() + AliveTicks;
                if (pendant_debug_in) { hal.stream.write("[OK]"); hal.stream.write(ASCII_EOL); }
           }
         */

        // This is a keep-alive transmission from the Pendant. If not received once in 2 seconds, stop sending state updates
        else if (strcmp(cmd_buffer, "{\"OK\"}") == 0) {
                AliveMs = hal.get_elapsed_ticks() + AliveTicks;
                if (pendant_debug_in) { hal.stream.write("[OK]"); hal.stream.write(ASCII_EOL); }
        }

        cJSON_Delete(cmd_json); // GPT IMPROVED ?
}


static void pendant_send(sys_state_t state, bool AlwaysSend) {

        // static byte state_number = 11;
        static char StateStr[10];
        static sys_state_t stateOld;
   
        /* 
        if (memcmp(state, stateOld, sizeof(state)) != 0) {
                AlwaysSend = true;  // We Send, because state has changed!
                memcpy(stateOld, state, sizeof(state));
        }
        */

        // GPT IMPROVED ?
        if (state != stateOld) {
                AlwaysSend = true;  // state has changed
                stateOld = state;
        }

        if (AlwaysSend) {               // state change is already checked!
                switch (state) {
                case STATE_IDLE:
                        strncpy(StateStr, "Idle", sizeof(StateStr) - 1); break;
                // state_number = 0; break;
                case STATE_CYCLE:
                        strncpy(StateStr, "Idle", sizeof(StateStr) - 1); break;
                // state_number = 1; break;
                case STATE_HOLD:
                        strncpy(StateStr, "Hold", sizeof(StateStr) - 1); break;
                // state_number = 2; break;
                case STATE_JOG:
                        strncpy(StateStr, "Jogging", sizeof(StateStr) - 1); break;
                // state_number = 3; break;
                case STATE_HOMING:
                        strncpy(StateStr, "Homing", sizeof(StateStr) - 1); break;
                // state_number = 4; break;
                case STATE_ESTOP:
                        strncpy(StateStr, "Error", sizeof(StateStr) - 1); break;
                // state_number = 5; break;
                case STATE_ALARM:
                        strncpy(StateStr, "Alarm", sizeof(StateStr) - 1); break;
                // state_number = 6; break;
                case STATE_CHECK_MODE:
                        strncpy(StateStr, "Check", sizeof(StateStr) - 1); break;
                // state_number = 7; break;
                case STATE_SAFETY_DOOR:
                        strncpy(StateStr, "Door", sizeof(StateStr) - 1); break;
                // state_number = 8; break;
                case STATE_SLEEP:
                        strncpy(StateStr, "Sleep", sizeof(StateStr) - 1); break;
                // state_number = 9; break;
                case STATE_TOOL_CHANGE:
                        strncpy(StateStr, "Tool", sizeof(StateStr) - 1); break;
                // state_number = 10; break;
                default:
                        strncpy(StateStr, "N/A", sizeof(StateStr) - 1); break;
                        // state_number = 11; break;
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
                wco[i] = gc_get_offset(i);
                float_pos[i] -= wco[i];
        }

        if (memcmp(float_pos, float_pos_old, sizeof(float_pos)) != 0) {
                AlwaysSend = true;  // We Send, because position has changed!
                memcpy(float_pos_old, float_pos, sizeof(float_pos));
        }

        // prepare JSON String for Sending
        if (AlwaysSend) {
                char wifi_out_buffer[OUTBUF_SIZE];
                if (N_AXIS == 3) { snprintf(wifi_out_buffer, sizeof(wifi_out_buffer), "{\"state\":\"%s\",\"wx\":%.3f,\"wy\":%.3f,\"wz\":%.3f}", StateStr, float_pos[0], float_pos[1], float_pos[2]); }
                else if (N_AXIS == 4) { snprintf(wifi_out_buffer, sizeof(wifi_out_buffer), "{\"state\":\"%s\",\"wx\":%.3f,\"wy\":%.3f,\"wz\":%.3f,\"wa\":%.3f}", StateStr, float_pos[0], float_pos[1], float_pos[2], float_pos[3]); }
                // else if (N_AXIS == 5) { snprintf(wifi_out_buffer, sizeof(wifi_out_buffer), "{\"state\":\"%s\",\"wx\":%.3f,\"wy\":%.3f,\"wz\":%.3f,\"wa\":%.3f,\"wb\":%.3f}"ASCII_EOL, string_state, float_pos[0], float_pos[1], float_pos[2], float_pos[3], float_pos[4]); }
                pendant_serial.write(wifi_out_buffer);
                if (pendant_debug_out) {hal.stream.write(wifi_out_buffer); hal.stream.write(ASCII_EOL);}
        }
}


/////////////   RECEIVE COMMANDS   //////////////
static void pendant_receive() {

        static int i = 0;
        static char in_buffer[INBUF_SIZE];

        while(pendant_serial.get_rx_buffer_count() > 0)
        {
                char in = pendant_serial.read();

                if (pendant_debug_in_raw) {
                        char * str_in = "-";
                        str_in[0] = in;
                        hal.stream.write(str_in);
                }

                if (in == '{') {
                        i = 0;
                        in_buffer[i] = in;
                }
                else if (in == '}') {
                        /* GPT IMPROVED ?
                        i++;
                        in_buffer[i] = in;
                        in_buffer[i+1] = '\0';
                        i = 0;
                        if (in_buffer[0] == '{') {
                                pendant_parse_and_send_cmd(in_buffer);
                        }
                                */
                        if (i < INBUF_SIZE - 2) {  // Platz für } und \0 prüfen
                                i++;
                                in_buffer[i] = in;
                                in_buffer[i+1] = '\0';
                                i = 0; // bereit für nächsten JSON
                                pendant_parse_and_send_cmd(in_buffer);
                        } 
                        else {
                        // Puffer voll, JSON verwerfen
                        i = 0;
                        // Optional: Debugmeldung
                        if (pendant_debug_in) hal.stream.write("[JSON overflow]" ASCII_EOL);
                        }
                }
                else {

                        if (i < INBUF_SIZE - 2) {
                                i++;
                                in_buffer[i] = in;
                            } else {
                                // Puffer voll, JSON verwerfen
                                i = 0;
                                if (pendant_debug_in) hal.stream.write("[JSON overflow]" ASCII_EOL);
                            }
                        /* GPT IMPROVED ?
                        if (i < (sizeof(in_buffer)-3)) {
                                i++;
                                in_buffer[i] = in;
                        }
                        */
                }
        }
}


static void pendant_loop (sys_state_t state)
{
        uint32_t CurrentMs = hal.get_elapsed_ticks();

        // check, if it is time to receive
        if((CurrentMs >= ReceiveMs)) {
                ReceiveMs = CurrentMs + ReceiveTicks;
                pendant_receive();
        }


        if (CurrentMs < AliveMs) {          // only send if alive

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
        }
        on_execute_realtime(state);
}

// report if state has changed (some states show up for only very short time - needed for probing-alarms)
static void state_changed(sys_state_t state) {

        uint32_t CurrentMs = hal.get_elapsed_ticks();

        if (CurrentMs < AliveMs) {
                SendMs = CurrentMs + SendTicks;             // we just had a big update -> reset both timers
                SendAlwaysMs = CurrentMs + SendAlwaysTicks;
                pendant_send(state, true);
        }

        if(on_state_change) { on_state_change(state); }
}

// Say hello, pendant!
static void report_options (bool newopt)
{
        on_report_options(newopt);
        if(!newopt) { hal.stream.write("[PLUGIN:PENDANT]" ASCII_EOL); }
}

// initialize pendant
bool pendant_init (const io_stream_t *stream)
{
        // initialize serial stream
        memcpy(&pendant_serial, serialInit(115200), sizeof(io_stream_t));

        // Add pendant_update function to grblHAL foreground process
        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = pendant_loop;

        // Add state change interrupt
        on_state_change = grbl.on_state_change;
        grbl.on_state_change = state_changed;

        // Add report
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        return true;
}
