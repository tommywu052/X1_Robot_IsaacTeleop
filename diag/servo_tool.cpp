// servo_tool.cpp -- Feetech STS bus diagnostic for the X1 grippers (and any servo).
//
// Talks DIRECTLY to /dev/ttyACM0 @ 1e6 using the same scservo_sdk the robot's
// XiaobeiHardwareInterface uses. Purpose: figure out why LEFT gripper (servo 7)
// reports position but won't move, while RIGHT (servo 57) works.
//
// !!! EXCLUSIVE PORT !!!  ros2_control_node also owns /dev/ttyACM0. Running this
// at the same time GARBLES the bus. STOP the real-robot stack first:
//     pkill -9 -f ros2_control_node   (this makes arms go LIMP -> support them!)
// or just Ctrl-C run_real_robot.sh. Then run this tool, then relaunch.
//
// Build (on cam):
//   g++ -std=c++17 -I ~/xiaobei_X1_ws/src/xiaobei_hardware/include servo_tool.cpp \
//     ~/xiaobei_X1_ws/src/xiaobei_hardware/src/scservo_sdk/{SCS,SCSerial,SMS_STS}.cpp \
//     -o ~/servo_tool
//
// Usage:
//   ~/servo_tool                      # status of servo 7 and 57 (default)
//   ~/servo_tool status 7 57 1 51     # status of listed ids
//   ~/servo_tool enable  <id>         # force torque ON, hold current position
//   ~/servo_tool disable <id>         # torque OFF (free)
//   ~/servo_tool recover <id>         # off->on + restore torque limit + hold pos (clears overload trip)
//   ~/servo_tool movetest <id> <delta> [torqueLimit]
//                                     # SAFE nudge: from current pos, +delta then back,
//                                     # then -delta then back, logging pos/load/current.
//                                     # Reveals which direction is FREE vs which STALLS
//                                     # (load pegged, pos stuck) => confirms a reversed
//                                     # gripper direction driving into a mechanical stop.
//                                     # delta ~150-300 pulses, torqueLimit 0..1000 (default 300, gentle).

#include "scservo_sdk/SMS_STS.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

static const char *PORT = "/dev/ttyACM0";
static const int BAUD = 1000000;

SMS_STS st;

static const char *mode_str(int m) {
    if (m == 0) return "POSITION(servo)";
    if (m == 1) return "WHEEL/const-speed  <-- position cmds IGNORED!";
    if (m == 2) return "PWM";
    if (m == 3) return "STEP";
    return "?";
}

static void status(int id) {
    printf("========== servo id %d ==========\n", id);
    if (st.Ping(id) == -1) {
        printf("  PING FAIL: no response on the bus (unpowered / wrong id / cable).\n");
        return;
    }
    int mode = st.readByte(id, SMS_STS_MODE);
    int torq = st.readByte(id, SMS_STS_TORQUE_ENABLE);
    int amin = st.readWord(id, SMS_STS_MIN_ANGLE_LIMIT_L);
    int amax = st.readWord(id, SMS_STS_MAX_ANGLE_LIMIT_L);
    int ofs  = st.readWord(id, SMS_STS_OFS_L);
    int tlim = st.readWord(id, SMS_STS_TORQUE_LIMIT_L);
    int pos  = st.ReadPos(id);
    int spd  = st.ReadSpeed(id);
    int load = st.ReadLoad(id);
    int volt = st.ReadVoltage(id);
    int temp = st.ReadTemper(id);
    int cur  = st.ReadCurrent(id);
    int mov  = st.ReadMove(id);
    printf("  ping=OK   MODE=%d %s\n", mode, mode_str(mode));
    printf("  TORQUE_ENABLE=%d  (%s)\n", torq, torq == 1 ? "ON" : "OFF/limp -> won't move");
    printf("  angle_limit=[%d .. %d]   OFS=%d   torque_limit=%d/1000\n", amin, amax, ofs, tlim);
    printf("  pos=%d   speed=%d   moving=%d\n", pos, spd, mov);
    printf("  load=%d/1000   current=%d   voltage=%d(0.1V)   temp=%dC%s\n",
           load, cur, volt, temp, temp >= 65 ? "  <-- HOT (overheat protection?)" : "");
}

static void ref_note() {
    // gripper_bridge commands joint 0.0(open)->1.0(close) for BOTH j_7 and j_57.
    // pulse = rad*direction*(2048/pi)+2048 with offset 2048.
    //   j_7  dir=+1: close(1.0) -> ~2700 (pulse UP from 2048)
    //   j_57 dir=-1: close(1.0) -> ~1396 (pulse DOWN from 2048)
    printf("\n[ref] 'close' command sends: servo7 -> ~2700 (dir+1),  servo57 -> ~1396 (dir-1).\n");
    printf("      Both start ~2048 (open). If servo7's real 'close' is pulse-DOWN like 57,\n");
    printf("      then dir should be -1; +1 drives it the WRONG way into a hard stop (stall).\n");
}

static void enable(int id, int on) {
    int r = st.EnableTorque(id, (u8)on);
    printf("EnableTorque(%d, %d) -> %d\n", id, on, r);
}

static void recover(int id) {
    printf("recover servo %d: torque OFF -> restore torque_limit=1000 -> hold current pos -> torque ON\n", id);
    st.EnableTorque(id, 0);
    usleep(300000);
    st.writeWord(id, SMS_STS_TORQUE_LIMIT_L, 1000);
    int pos = st.ReadPos(id);
    if (pos == -1) { printf("  no response; abort.\n"); return; }
    st.WritePosEx(id, (s16)pos, 0, 0);
    st.EnableTorque(id, 1);
    usleep(200000);
    printf("  done. TORQUE_ENABLE now=%d, pos=%d\n", st.readByte(id, SMS_STS_TORQUE_ENABLE), st.ReadPos(id));
}

static void watch(int id, double secs) {
    int steps = (int)(secs / 0.1);
    for (int k = 0; k < steps; k++) {
        usleep(100000);
        printf("    t=%.1fs pos=%d load=%d cur=%d moving=%d\n",
               k * 0.1, st.ReadPos(id), st.ReadLoad(id), st.ReadCurrent(id), st.ReadMove(id));
    }
}

static void movetest(int id, int delta, int tlim) {
    int start = st.ReadPos(id);
    if (start == -1) { printf("servo %d no response; abort.\n", id); return; }
    printf("movetest servo %d: start_pos=%d  delta=%d  torque_limit=%d/1000\n", id, start, delta, tlim);
    printf("(safe: low torque limit so a stall cannot crush; watch load/current)\n");
    st.writeWord(id, SMS_STS_TORQUE_LIMIT_L, (u16)tlim);
    st.EnableTorque(id, 1);

    int slow = 400;  // speed steps/s (gentle)
    printf("\n-- nudge +%d (toward pulse %d) --\n", delta, start + delta);
    st.WritePosEx(id, (s16)(start + delta), (u16)slow, 20);
    watch(id, 1.5);
    printf("-- back to start %d --\n", start);
    st.WritePosEx(id, (s16)start, (u16)slow, 20);
    watch(id, 1.5);

    printf("\n-- nudge -%d (toward pulse %d) --\n", delta, start - delta);
    st.WritePosEx(id, (s16)(start - delta), (u16)slow, 20);
    watch(id, 1.5);
    printf("-- back to start %d --\n", start);
    st.WritePosEx(id, (s16)start, (u16)slow, 20);
    watch(id, 1.5);

    printf("\nINTERPRET: the direction where pos DID NOT change while load/current stayed HIGH\n");
    printf("           is the mechanical hard stop (wrong way). The free direction is correct.\n");
    // restore a normal torque limit
    st.writeWord(id, SMS_STS_TORQUE_LIMIT_L, 1000);
}

int main(int argc, char **argv) {
    printf("opening %s @ %d ...\n", PORT, BAUD);
    if (!st.begin(BAUD, PORT)) {
        printf("FAILED to open %s. Is the robot USB attached, and is ros2_control_node STOPPED?\n", PORT);
        return 1;
    }
    std::string cmd = (argc >= 2) ? argv[1] : "status";

    if (cmd == "status") {
        if (argc >= 3) {
            for (int i = 2; i < argc; i++) status(atoi(argv[i]));
        } else {
            status(7);
            status(57);
        }
        ref_note();
    } else if (cmd == "enable" && argc >= 3) {
        enable(atoi(argv[2]), 1);
        status(atoi(argv[2]));
    } else if (cmd == "disable" && argc >= 3) {
        enable(atoi(argv[2]), 0);
    } else if (cmd == "recover" && argc >= 3) {
        recover(atoi(argv[2]));
    } else if (cmd == "movetest" && argc >= 4) {
        int id = atoi(argv[2]);
        int delta = atoi(argv[3]);
        int tlim = (argc >= 5) ? atoi(argv[4]) : 300;
        movetest(id, delta, tlim);
    } else {
        printf("usage: servo_tool [status [ids...] | enable <id> | disable <id> | recover <id> | movetest <id> <delta> [torqueLimit]]\n");
    }

    st.end();
    return 0;
}
