#include "ring_runtime.h"
#include "ring_link.h"
#include "ring_control.h"
#include "ring_force.h"
#include "ring_maneuver.h"
#include "ring_entry_speed.h"
#include "ring.h"
#include <string.h>

static volatile uint32_t runtime_ms, last_command_ms;
static volatile float heading_deg;
static volatile float heading_rate;
static ring_force_t force_control;
static ring_maneuver_t maneuver;
static ring_entry_speed_t entry_speed;
static float preceding_target;
static uint32_t preceding_target_ms,motion_sample_ms;
static uint8_t preceding_target_valid,motion_sample_valid;
static uint32_t maneuver_event_seen;
static int force_requested,force_side;
static volatile int speed_cap;
static volatile uint8_t runtime_ready;
static volatile uint32_t beep_requested;
static uint32_t beep_started, beep_deadline, next_beep_ms;
static uint8_t beep_on;
static float gyro_scale, gyro_bias;
static int32_t bias_sum;
static unsigned bias_samples;
static uint32_t epoch, feedback_sequence, seen_sequence;
static uint32_t detected_seen, exited_seen;
static int ring_fault;
static volatile uint8_t ring_active;
static volatile uint8_t abort_pending;
static volatile float fallback_forward,fallback_turn;
static int requested_boost;
static float applied_boost;
static ring_runtime_diagnostics_t diagnostics;
static uint32_t diagnostic_sequence, last_diagnostic_ms;
static uint32_t entry_diagnostic_sequence, last_entry_diagnostic_ms;
static uint32_t geometry_diagnostic_sequence, last_geometry_diagnostic_ms;
static uint32_t topology_sequence, last_topology_ms;
static uint32_t mouth_sequence,last_mouth_ms;
static uint32_t pair_last_ms;
static uint8_t pair_packet_valid;
static volatile uint32_t operator_flags; /* main writes; 1ms buzzer tick reads */

const ring_runtime_diagnostics_t *ring_runtime_diagnostics(void)
{
    return &diagnostics;
}

void ring_runtime_init(float gyro_dps_per_count)
{
    ring_packet_t previous;
    epoch = 1;
    if (ring_link_read(ring_motion_mailbox, &previous)) {
        epoch = (uint32_t)previous.word[RL_EPOCH] + 1u;
        if (!epoch) epoch = 1;
    }
    runtime_ms = last_command_ms = pair_last_ms = 0;
    pair_packet_valid=0;
    heading_deg = gyro_bias = 0.0f;
    heading_rate=0;force_requested=force_side=0;
    memset(&force_control,0,sizeof(force_control));memset(&maneuver,0,sizeof(maneuver));
    maneuver_event_seen=0;
    memset(&entry_speed,0,sizeof(entry_speed));
    preceding_target=0;preceding_target_ms=motion_sample_ms=0;
    preceding_target_valid=motion_sample_valid=0;
    gyro_scale = gyro_dps_per_count;
    speed_cap = -1;
    runtime_ready = 0;
    operator_flags = RING_OPERATOR_VALID;
    bias_sum = 0;
    bias_samples = 0;
    feedback_sequence = seen_sequence = 0;
    detected_seen = exited_seen = 0;
    beep_requested = beep_started = beep_deadline = next_beep_ms = 0;
    beep_on = 0;
    ring_fault = 0;
    ring_active = 0;
    abort_pending=0;fallback_forward=fallback_turn=0;
    requested_boost = 0;
    applied_boost = 0.0f;
    memset(&diagnostics, 0, sizeof(diagnostics));
    diagnostics.cap = -1;
    diagnostics.motion_age_ms=UINT32_MAX;
    diagnostics.command_epoch = epoch;
    diagnostics.control_version=RING_PAIR_VERSION;
    diagnostics.island_near = -1;
    diagnostics.near_x = diagnostics.look_x = -1;
    diagnostics.visual_dt_ms = 65535u;
    diagnostic_sequence = last_diagnostic_ms = 0;
    entry_diagnostic_sequence = last_entry_diagnostic_ms = 0;
    geometry_diagnostic_sequence = last_geometry_diagnostic_ms = 0;
    diagnostics.guide_x = -1;
    diagnostics.guide_y = 255u;
    topology_sequence = last_topology_ms = mouth_sequence = last_mouth_ms = 0;
    diagnostics.mouth_near_x=diagnostics.mouth_near_y=-1;
    diagnostics.mouth_far_x=diagnostics.mouth_far_y=diagnostics.mouth_first_y=-1;
    diagnostics.near_row = diagnostics.far_row = -1;
}

void ring_runtime_set_operator(uint8_t run_requested, uint8_t binary_display)
{
    operator_flags = RING_OPERATOR_VALID |
        (run_requested ? RING_OPERATOR_RUN : 0u) |
        (binary_display ? RING_OPERATOR_BINARY : 0u);
}

void ring_runtime_observe_motion(float measured_rpm,float pitch_deg,
    float speed_tilt_deg,int left_pwm,int right_pwm)
{
    diagnostics.measured_forward10=(int16_t)fmaxf(-32768.f,fminf(32767.f,measured_rpm*10.f));
    diagnostics.pitch100=(int16_t)fmaxf(-32768.f,fminf(32767.f,pitch_deg*100.f));
    diagnostics.speed_tilt100=(int16_t)fmaxf(-32768.f,fminf(32767.f,speed_tilt_deg*100.f));
    diagnostics.left_pwm=left_pwm;diagnostics.right_pwm=right_pwm;
    motion_sample_ms=runtime_ms;motion_sample_valid=1;
}

int8_t ring_runtime_tick(int16_t gyro_z_raw, uint8_t standing)
{
    uint32_t now = ++runtime_ms;
    if (!RING_ENABLE) return -1;
    runtime_ready = standing;
    /* Use the existing stationary startup wait. This does not modify any
     * gyro_offset_* or quaternion/balance state and never delays a servo. */
    if (!standing && now >= 1000u && now < 2000u) {
        bias_sum += gyro_z_raw;
        ++bias_samples;
    }
    if (now == 2000u && bias_samples)
        gyro_bias = (float)bias_sum / bias_samples;
    if (standing && now >= 2000u && bias_samples) {
        float rate=((float)gyro_z_raw-gyro_bias)*gyro_scale;
        heading_rate+=.02f*(rate-heading_rate);
        heading_deg += rate * 0.001f;
    } else heading_rate=0;

    if (!standing && (operator_flags & RING_OPERATOR_RUN)) {
        /* Starting has priority over preview beeps. Drop pending preview
         * events and release an active preview tone ONCE; subsequent ticks
         * leave the original startup buzzer entirely alone. */
        beep_started = beep_requested;
        next_beep_ms = now;
        if (beep_on) {
            beep_on = 0;
            return 0;
        }
        return -1;
    }
    if (beep_on && (int32_t)(now - beep_deadline) >= 0) {
        beep_on = 0;
        next_beep_ms = now + RING_EVENT_BEEP_MS;
        return 0;
    }
    if (!beep_on && beep_started != beep_requested &&
        (int32_t)(now - next_beep_ms) >= 0) {
        ++beep_started;
        beep_on = 1;
        beep_deadline = now + RING_EVENT_BEEP_MS;
        return 1;
    }
    return -1;
}

uint8_t ring_runtime_watchdog_stop(void)
{
    return (uint8_t)(!RING_TRACK_MODE && RING_ENABLE && runtime_ready && ring_active && speed_cap >= 0 &&
                     runtime_ms - last_command_ms > RING_LINK_TIMEOUT_MS);
}

/* Existing 1ms call site: restore the last output calculated BEFORE ring
 * overrides once if its producer stops. No waiting, stop command, forced
 * steering or stale cap is kept alive by a stalled main loop. A later
 * normal vision frame supplies the current ordinary error as usual. */
void ring_runtime_poll_timeout(float *forward,float *turn)
{
    if(!RING_ENABLE)return;
    if(!RING_TRACK_MODE) {
        if(ring_runtime_watchdog_stop()){*forward=0;*turn=0;}
        return;
    }
    if(runtime_ready && ring_active && runtime_ms-last_command_ms>RING_LINK_TIMEOUT_MS) {
        abort_pending=4;ring_active=0;speed_cap=-1;
        *forward=(operator_flags&RING_OPERATOR_RUN)?fallback_forward:0;
        *turn=(operator_flags&RING_OPERATOR_RUN)?fallback_turn:0;
    }
}

/* Retry separately: the producer publishes command before diagnostics.
 * Telemetry validity cannot change control validity, freshness or outputs. */
static void read_diagnostics(uint32_t now)
{
    ring_packet_t packet;
    if (ring_link_read_diagnostic(&packet) &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RD_COMMAND_SEQUENCE] == diagnostics.command_sequence &&
        diagnostics.command_sequence != 0u &&
        (uint32_t)packet.word[RL_SEQUENCE] != diagnostic_sequence) {
        uint32_t flags = (uint32_t)packet.word[RD_FLAGS];
        int source = ring_diagnostic_source(packet.word[RD_FLAGS]);
        int near_x = ring_diagnostic_near_x(packet.word[RD_POINTS]);
        int look_x = ring_diagnostic_look_x(packet.word[RD_POINTS]);
        int valid = (flags & 0xfc000000u) == 0u &&
            ((flags >> 8) & 3u) != 3u &&
            (source == 0 || source == 1 || source == 2 || source == 4 || source == 5 || source == 6) &&
            near_x >= -1 && near_x < RING_W && look_x >= -1 && look_x < RING_W;
        if (valid) {
            diagnostic_sequence = (uint32_t)packet.word[RL_SEQUENCE];
            last_diagnostic_ms = now;
            diagnostics.diagnostic_epoch = epoch;
            diagnostics.diagnostic_command_sequence = (uint32_t)packet.word[RD_COMMAND_SEQUENCE];
            diagnostics.frame_sequence = (uint32_t)packet.word[RD_FRAME];
            diagnostics.visual_dt_ms = (uint16_t)(uint32_t)packet.word[RD_TIMING_REJECT];
            diagnostics.reject = (uint16_t)((uint32_t)packet.word[RD_TIMING_REJECT] >> 16);
            diagnostics.candidate_detected = (uint8_t)ring_diagnostic_detected(packet.word[RD_FLAGS]);
            diagnostics.candidate_allowed = (uint8_t)ring_diagnostic_allowed(packet.word[RD_FLAGS]);
            diagnostics.score = (uint8_t)ring_diagnostic_score(packet.word[RD_FLAGS]);
            diagnostics.confirm_frames = (uint8_t)ring_diagnostic_confirm_frames(packet.word[RD_FLAGS]);
            diagnostics.entry_side = (int8_t)ring_diagnostic_side(packet.word[RD_FLAGS]);
            diagnostics.branch_ok = (uint8_t)ring_diagnostic_branch(packet.word[RD_FLAGS]);
            diagnostics.path_source = (uint8_t)source;
            diagnostics.classification = (uint8_t)ring_diagnostic_classification(packet.word[RD_FLAGS]);
            diagnostics.near_x = (int16_t)near_x;
            diagnostics.look_x = (int16_t)look_x;
        }
    }
    diagnostics.diagnostic_age_ms = now - last_diagnostic_ms;
    diagnostics.diagnostic_valid = (uint8_t)(diagnostic_sequence != 0u &&
        diagnostics.diagnostic_epoch == epoch &&
        diagnostics.diagnostic_command_sequence == diagnostics.command_sequence &&
        diagnostics.diagnostic_age_ms <= RING_LINK_TIMEOUT_MS);
}

static void read_entry_diagnostics(uint32_t now)
{
    ring_packet_t packet;
    if (ring_link_read_entry_diagnostic(&packet) &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RE_COMMAND_SEQUENCE] == diagnostics.command_sequence &&
        diagnostics.command_sequence != 0u &&
        (uint32_t)packet.word[RL_SEQUENCE] != entry_diagnostic_sequence) {
        uint32_t flags = (uint32_t)packet.word[RE_FLAGS];
        uint32_t age_lost = (uint32_t)packet.word[RE_AGE_LOST];
        unsigned entry_flags = flags & 65535u;
        unsigned reject = (flags >> 16) & 255u;
        unsigned decision = flags >> 24;
        /* Unknown bits/decisions are rejected independently of commands/D. */
        if ((entry_flags & ~8191u) == 0u && (reject & ~63u) == 0u &&
            decision <= (RING_TRACK_MODE?15u:14u) && (age_lost & 0xff000000u) == 0u) {
            entry_diagnostic_sequence = (uint32_t)packet.word[RL_SEQUENCE];
            last_entry_diagnostic_ms = now;
            diagnostics.entry_diagnostic_epoch = epoch;
            diagnostics.entry_diagnostic_command_sequence = (uint32_t)packet.word[RE_COMMAND_SEQUENCE];
            diagnostics.entry_frame_sequence = (uint32_t)packet.word[RE_FRAME];
            diagnostics.entry_flags = (uint16_t)entry_flags;
            diagnostics.hold_reject = (uint16_t)reject;
            diagnostics.decision = (uint8_t)decision;
            diagnostics.path_age_ms = (uint16_t)age_lost;
            diagnostics.lost_frames = (uint8_t)(age_lost >> 16);
            diagnostics.hold_turn10 = (int16_t)ring_entry_diagnostic_hold_turn(packet.word[RE_ANGLES]);
            diagnostics.entry_turn10 = (int16_t)ring_entry_diagnostic_entry_turn(packet.word[RE_ANGLES]);
        }
    }
    diagnostics.entry_diagnostic_age_ms = now - last_entry_diagnostic_ms;
    diagnostics.entry_diagnostic_valid = (uint8_t)(entry_diagnostic_sequence != 0u &&
        diagnostics.entry_diagnostic_epoch == epoch &&
        diagnostics.entry_diagnostic_command_sequence == diagnostics.command_sequence &&
        diagnostics.entry_diagnostic_age_ms <= RING_LINK_TIMEOUT_MS);
}

static void read_geometry_diagnostics(uint32_t now)
{
    ring_packet_t packet;
    if (ring_link_read_geometry_diagnostic(&packet) &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RG_COMMAND_SEQUENCE] == diagnostics.command_sequence &&
        diagnostics.command_sequence != 0u &&
        (uint32_t)packet.word[RL_SEQUENCE] != geometry_diagnostic_sequence) {
        uint32_t geometry = (uint32_t)packet.word[RG_GEOMETRY];
        unsigned mode = geometry & 255u;
        unsigned rows = (geometry >> 8) & 255u;
        unsigned associated = (geometry >> 16) & 255u;
        unsigned support = geometry >> 24;
        int x = ring_geometry_diagnostic_x(packet.word[RG_GUIDE]);
        unsigned y = ring_geometry_diagnostic_y(packet.word[RG_GUIDE]);
        int valid_point = (x == -1 && y == 255u) ||
            (x >= 0 && x < RING_W && y < RING_H);
        if (mode <= 6u && rows <= RING_H && associated <= RING_H &&
            (support & ~7u) == 0u &&
            ((uint32_t)packet.word[RG_STRONG_AGE] & 0xffff0000u) == 0u &&
            valid_point) {
            geometry_diagnostic_sequence = (uint32_t)packet.word[RL_SEQUENCE];
            last_geometry_diagnostic_ms = now;
            diagnostics.geometry_epoch = epoch;
            diagnostics.geometry_command_sequence = (uint32_t)packet.word[RG_COMMAND_SEQUENCE];
            diagnostics.geometry_frame_sequence = (uint32_t)packet.word[RG_FRAME];
            diagnostics.branch_mode = (uint8_t)mode;
            diagnostics.inner_rows = (uint8_t)rows;
            diagnostics.associated_rows = (uint8_t)associated;
            diagnostics.target_support = (uint8_t)support;
            diagnostics.strong_age_ms = (uint16_t)(uint32_t)packet.word[RG_STRONG_AGE];
            diagnostics.guide_x = (int16_t)x;
            diagnostics.guide_y = (uint8_t)y;
        }
    }
    diagnostics.geometry_age_ms = now - last_geometry_diagnostic_ms;
    diagnostics.geometry_valid = (uint8_t)(geometry_diagnostic_sequence != 0u &&
        diagnostics.geometry_epoch == epoch &&
        diagnostics.geometry_command_sequence == diagnostics.command_sequence &&
        diagnostics.geometry_age_ms <= RING_LINK_TIMEOUT_MS);
}

/* H is optional evidence. Missing/corrupt/stale H cannot acquire control,
 * refresh last_command_ms, stop motion, or modify ordinary outputs. */
static void read_topology_diagnostics(uint32_t now)
{
    ring_packet_t packet;
    if (ring_link_read_topology_diagnostic(&packet) &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RH_COMMAND_SEQUENCE] == diagnostics.command_sequence &&
        diagnostics.command_sequence &&
        (uint32_t)packet.word[RL_SEQUENCE] != topology_sequence) {
        uint32_t flags=(uint32_t)packet.word[RH_FLAGS];
        unsigned stage=(flags>>2)&7u;
        int near_row=ring_topology_near_row(packet.word[RH_FLAGS]);
        int far_row=ring_topology_far_row(packet.word[RH_FLAGS]);
        if ((flags&0xfe000000u)==0u && ((flags>>23)&3u)<=2u && (flags&3u)!=3u &&
            stage<=RING_TANGENT_EXIT && near_row<RING_H && far_row<RING_H) {
            topology_sequence=(uint32_t)packet.word[RL_SEQUENCE];
            last_topology_ms=now;
            diagnostics.topology_epoch=epoch;
            diagnostics.topology_command_sequence=(uint32_t)packet.word[RH_COMMAND_SEQUENCE];
            diagnostics.topology_frame=(uint32_t)packet.word[RH_FRAME];
            diagnostics.ring_side=(int8_t)ring_topology_side(packet.word[RH_FLAGS]);
            diagnostics.approach_stage=(uint8_t)stage;
            diagnostics.left_entry=(uint8_t)((flags>>5)&1u);
            diagnostics.lap_valid=(uint8_t)((flags>>6)&1u);
            diagnostics.mouth_source=(uint8_t)((flags>>23)&3u);
            diagnostics.near_row=(int16_t)near_row;
            diagnostics.far_row=(int16_t)far_row;
            diagnostics.lap_origin_mdeg=packet.word[RH_ORIGIN_MDEG];
            diagnostics.lap_mdeg=packet.word[RH_LAP_MDEG];
        }
    }
    diagnostics.topology_age_ms=now-last_topology_ms;
    diagnostics.topology_valid=(uint8_t)(topology_sequence &&
        diagnostics.topology_epoch==epoch &&
        diagnostics.topology_command_sequence==diagnostics.command_sequence &&
        diagnostics.topology_age_ms<=RING_LINK_TIMEOUT_MS);
}

static void read_mouth_diagnostics(uint32_t now)
{
    ring_packet_t p;
    if(ring_link_read_mouth_diagnostic(&p) && (uint32_t)p.word[RL_EPOCH]==epoch &&
        (uint32_t)p.word[RJ_COMMAND_SEQUENCE]==diagnostics.command_sequence &&
        diagnostics.command_sequence && (uint32_t)p.word[RL_SEQUENCE]!=mouth_sequence) {
        uint32_t gates=(uint32_t)p.word[RJ_GATES],pair=(uint32_t)p.word[RJ_PROGRESS_PAIR];
        int nx=ring_mouth_coordinate(p.word[RJ_POINTS],0);
        int ny=ring_mouth_coordinate(p.word[RJ_POINTS],8);
        int fx=ring_mouth_coordinate(p.word[RJ_POINTS],16);
        int fy=ring_mouth_coordinate(p.word[RJ_POINTS],24);
        int first=ring_mouth_coordinate(p.word[RJ_PROGRESS_PAIR],0);
        int nvalid=(nx== -1 && ny== -1)||(nx>=0 && nx<RING_W && ny>=0 && ny<RING_H);
        int fvalid=(fx== -1 && fy== -1)||(fx>=0 && fx<RING_W && fy>=0 && fy<RING_H);
        if((gates&0x80000000u)==0u && nvalid && fvalid && first<RING_H &&
            ((pair>>16)&255u)==RING_PAIR_VERSION && (pair>>24)==RING_PAIR_VERSION) {
            mouth_sequence=(uint32_t)p.word[RL_SEQUENCE];last_mouth_ms=now;
            diagnostics.mouth_epoch=epoch;diagnostics.mouth_command_sequence=(uint32_t)p.word[RJ_COMMAND_SEQUENCE];
            diagnostics.mouth_frame=(uint32_t)p.word[RJ_FRAME];
            diagnostics.mouth_near_x=(int16_t)nx;diagnostics.mouth_near_y=(int16_t)ny;
            diagnostics.mouth_far_x=(int16_t)fx;diagnostics.mouth_far_y=(int16_t)fy;
            diagnostics.mouth_first_y=(int16_t)first;
            diagnostics.mouth_progress=(int16_t)ring_mouth_progress(p.word[RJ_PROGRESS_PAIR]);
            diagnostics.mouth_gate=(uint16_t)ring_mouth_gate_value((int32_t)gates);diagnostics.mouth_ref=(uint8_t)((gates>>13)&1u);
            diagnostics.mouth_near_count=(uint8_t)((gates>>14)&15u);
            diagnostics.mouth_far_count=(uint8_t)((gates>>18)&15u);
            diagnostics.mouth_gap=(uint8_t)((gates>>22)&3u);
            diagnostics.mouth_near_missed=(uint8_t)((gates>>26)&7u);
            diagnostics.mouth_pass_reason=(uint8_t)((gates>>29)&3u);
        }
    }
    diagnostics.mouth_age_ms=now-last_mouth_ms;
    diagnostics.mouth_valid=(uint8_t)(mouth_sequence && diagnostics.mouth_epoch==epoch &&
        diagnostics.mouth_command_sequence==diagnostics.command_sequence &&
        diagnostics.mouth_age_ms<=RING_LINK_TIMEOUT_MS);
}

void ring_runtime_apply(float *forward_target, float *turn_out, float wheel_revs,
                        int vision_state)
{
    ring_packet_t packet;
    uint32_t now = runtime_ms;
    float original_forward = *forward_target;
    float original_turn = *turn_out;
    if (!RING_ENABLE) return;
    fallback_forward=original_forward;fallback_turn=original_turn;
    if(abort_pending) {
        requested_boost=0;applied_boost=0;ring_fault=0;
        memset(&force_control,0,sizeof(force_control));
        diagnostics.release_reason=abort_pending;
    }
    if (ring_link_read(ring_command_mailbox, &packet) &&
        (uint32_t)packet.word[RL_EPOCH] == epoch &&
        (uint32_t)packet.word[RL_SEQUENCE] != seen_sequence) {
        uint32_t detected = (uint32_t)packet.word[RC_DETECTED];
        uint32_t exited = (uint32_t)packet.word[RC_EXITED];
        int cap = ring_command_cap_value(packet.word[RC_CAP]);
        int turn_deg = ring_command_turn_deg(packet.word[RC_CAP]);
        int phase = (int)((uint32_t)packet.word[RC_PHASE] & 255u);
        int boost = ring_command_boost(packet.word[RC_PHASE]);
        unsigned version=ring_command_version(packet.word[RC_PHASE]);
        uint32_t diag_word = (uint32_t)packet.word[RC_FAULT];
        int fault = ring_command_diag_fault(packet.word[RC_FAULT]);
        int raw_state = ring_command_diag_raw_state(packet.word[RC_FAULT]);
        int near = ring_command_diag_near(packet.word[RC_FAULT]);
        int route_code = (int)((diag_word >> 25) & 3u);
        int steer_error = ring_command_diag_error(packet.word[RC_FAULT]);
        int timed=(diag_word&RING_COMMAND_TIMED_ENTRY)!=0u;
        int owned = (timed && phase==RING_APPROACH) || (phase >= (RING_TRACK_MODE ? RING_ENTER : RING_APPROACH) &&
            phase <= RING_RECOVER);
        int was_active = ring_active;
        int force=(diag_word&RING_COMMAND_FORCE_ENTRY)!=0u;
        int route=ring_command_diag_route(packet.word[RC_FAULT]);
        unsigned released=(diag_word>>RING_COMMAND_RELEASE_SHIFT)&7u;
        int valid = phase <= RING_FAULT &&
                    (version==RING_PAIR_VERSION || (!RING_TRACK_MODE && version==0u)) &&
                    (!timed || (RING_TRACK_MODE && RING_TIMED_ENTRY &&
                        version==RING_PAIR_VERSION && phase>=RING_APPROACH && phase<=RING_RECOVER &&
                        route && !force && !fault && cap>0 && !boost &&
                        !ring_command_diag_stop(packet.word[RC_FAULT]))) && fault <= 4 &&
                    (!released || (RING_TRACK_MODE && released<=4u &&
                        phase==RING_COOLDOWN && cap==-1 && !boost && !force && !timed && !fault &&
                        !ring_command_diag_stop(packet.word[RC_FAULT]))) &&
                    (!force || (RING_TRACK_MODE && version==RING_PAIR_VERSION &&
                        (phase==RING_ENTER || phase==RING_CIRCLE) && !fault && cap>0 && route &&
                        !ring_command_diag_stop(packet.word[RC_FAULT]) &&
                        !ring_command_diag_cross_veto(packet.word[RC_FAULT]))) &&
                    raw_state <= 2 && route_code <= 2 && near < RING_H &&
                    steer_error >= -90 && steer_error <= 90 &&
                    boost >= -RING_ENTRY_BOOST_RPM && boost <= RING_ENTRY_BOOST_RPM &&
                    cap >= -1 && cap <= RING_APPROACH_RPM &&
                    (!owned || cap >= 0) &&
                    (phase != RING_FAULT || cap == 0) &&
                    (!RING_TRACK_MODE || phase != RING_APPROACH || ((timed?cap>0:cap==-1) && boost == 0)) &&
                    ((phase != RING_NORMAL && phase != RING_COOLDOWN) ||
                     (cap == -1 && boost == 0));
        /* A successful handoff must also pass the CONTROL core's own
         * measured net-angle gate. An explicit abort remains an abort;
         * it never counts as a successful lap or exit beep. */
        if(valid && RING_TRACK_MODE && RING_TIMED_ENTRY && was_active &&
           maneuver.stage && maneuver.stage<4u) {
            if(((owned && !timed) && maneuver.stage<3u) ||
               ((phase==RING_EXIT || phase==RING_RECOVER || exited>exited_seen) &&
                (maneuver.stage!=3u || !maneuver.lap_reached ||
                fabsf(wheel_revs-maneuver.turn_revs)<RING_EXIT_MIN_REVS ||
                fabsf(maneuver.side*(heading_deg-maneuver.turn_yaw)-RING_EXIT_MIN_DEG)>RING_LAP_EXIT_HEADING_TOL))) valid=0;
            /* Cancellation with no new exit event must always release.
             * v27 rejected it and left the previous ring owner latched. */
        }
        seen_sequence = (uint32_t)packet.word[RL_SEQUENCE];
        diagnostics.vision_version=(uint8_t)version;
        pair_packet_valid=(uint8_t)(valid && version==RING_PAIR_VERSION);
        if(pair_packet_valid)pair_last_ms=now;
        /* Ring caps only belong to an actual ring steering handover. A
         * candidate packet, NORMAL with a bogus cap, or unacquired FAULT
         * must not limit/stop the original controller. */
        if (valid && (phase == RING_NORMAL || phase == RING_COOLDOWN ||
            (RING_TRACK_MODE && phase == RING_APPROACH && !timed))) {
            ring_active = 0;
            speed_cap = -1;
            ring_fault = 0;
            requested_boost = 0;
            applied_boost = 0.0f;
            force_requested=0;memset(&force_control,0,sizeof(force_control));
            memset(&maneuver,0,sizeof(maneuver));
            abort_pending=0;diagnostics.release_reason=(uint8_t)released;
            if(detected<maneuver_event_seen)maneuver_event_seen=detected;
        } else if (valid && ((owned && detected && (vision_state == 3 || timed)) ||
                            (ring_active && (owned || phase == RING_FAULT))) && !abort_pending &&
                            (!timed || (runtime_ready && bias_samples && (operator_flags&RING_OPERATOR_RUN)))) {
            ring_active = 1;
            speed_cap = cap;
            ring_fault = fault;
            requested_boost = owned ? boost : 0;
            if(timed && !was_active && phase==RING_APPROACH && detected>maneuver_event_seen &&
               runtime_ready && bias_samples && (operator_flags&RING_OPERATOR_RUN) &&
               !ring_command_diag_cross_veto(packet.word[RC_FAULT])) {
                ring_maneuver_begin(&maneuver,now,-route,heading_deg,wheel_revs);
                ring_entry_speed_begin(&entry_speed,now,
                    preceding_target_valid && now-preceding_target_ms<=RING_LINK_TIMEOUT_MS?
                    preceding_target:original_forward);
                maneuver_event_seen=detected;
            }
            if(timed && maneuver.stage && maneuver.stage<4u && maneuver.side!=-route){
                maneuver.stage=4;maneuver.reason=7;
            }
            if(!timed && phase==RING_ENTER && !was_active)
                ring_force_begin(&force_control,now,-route,heading_deg,wheel_revs);
            force_requested=force;force_side=-route;
        }
        /* A restarted vision core can reset counters. Count each observed
         * forward event once; repeated snapshots never extend the beep. */
        if (valid) {
            int standby_event = !runtime_ready &&
                !(operator_flags & RING_OPERATOR_RUN) &&
                (phase == RING_NORMAL || phase == RING_COOLDOWN);
            int running_detection = runtime_ready &&
                (RING_TRACK_MODE ? ((operator_flags & RING_OPERATOR_RUN) != 0u) : ring_active);
            last_command_ms = now;
            diagnostics.command_sequence = (uint32_t)packet.word[RL_SEQUENCE];
            diagnostics.phase = (int16_t)phase;
            diagnostics.turn_deg = (int16_t)turn_deg;
            diagnostics.cap = (int16_t)cap;
            diagnostics.requested_boost = (int16_t)boost;
            diagnostics.fault = (uint8_t)fault;
            diagnostics.candidate = (uint8_t)ring_command_diag_candidate(packet.word[RC_FAULT]);
            diagnostics.cross_veto = (uint8_t)ring_command_diag_cross_veto(packet.word[RC_FAULT]);
            diagnostics.committed = (uint8_t)ring_command_diag_committed(packet.word[RC_FAULT]);
            diagnostics.path_valid = (uint8_t)ring_command_diag_path_valid(packet.word[RC_FAULT]);
            diagnostics.stop = (uint8_t)ring_command_diag_stop(packet.word[RC_FAULT]);
            diagnostics.raw_state = (int16_t)raw_state;
            diagnostics.island_near = (int16_t)near;
            diagnostics.steer_error = (int16_t)steer_error;
            diagnostics.route = (int16_t)ring_command_diag_route(packet.word[RC_FAULT]);
            diagnostics.detected = detected;
            diagnostics.exited = exited;
            if (detected > detected_seen &&
                (running_detection || standby_event)) ++beep_requested;
            if (runtime_ready && exited > exited_seen && was_active && phase == RING_COOLDOWN)
                ++beep_requested;
            /* A normal preview event can be consumed while parked. During
             * running, wait for the actual steering handover so its first
             * confirmation snapshot cannot swallow the detection beep. */
            if (ring_active || running_detection || standby_event || !detected) detected_seen = detected;
            exited_seen = exited;
        }
    }
    if(RING_TRACK_MODE && ring_active &&
       (runtime_ms-last_command_ms>RING_LINK_TIMEOUT_MS || speed_cap==0 || ring_fault || diagnostics.stop)) {
        /* Production ring failures relinquish ownership. They never
         * change the caller's original speed/turn to zero. */
        diagnostics.release_reason=(uint8_t)(runtime_ms-last_command_ms>RING_LINK_TIMEOUT_MS?4:1);
        abort_pending=diagnostics.release_reason;ring_active=0;speed_cap=-1;ring_fault=0;
        requested_boost=0;applied_boost=0;force_requested=0;
        memset(&force_control,0,sizeof(force_control));
    }
    if (runtime_ready && ring_active && speed_cap >= 0) {
        int fresh = now-last_command_ms<=RING_LINK_TIMEOUT_MS;
        if (ring_control_stop(speed_cap, 3, ring_fault, fresh)) {
            *turn_out = 0.0f;
            applied_boost = 0.0f;
        } else if (vision_state == 3 && original_forward > 0.0f) {
            float step = requested_boost - applied_boost;
            if (step > RING_BOOST_STEP_RPM) step = RING_BOOST_STEP_RPM;
            if (step < -RING_BOOST_STEP_RPM) step = -RING_BOOST_STEP_RPM;
            applied_boost += step;
            *turn_out = ring_control_turn(*turn_out, applied_boost, 1);
        } else applied_boost = 0.0f;
        /* Optional D/E/G/H/J packets never arm or renew this control.
         * Only the matching fresh command can request the locked F turn. */
        if(maneuver.stage) {
            int eligible=fresh && runtime_ready && original_forward>0 && bias_samples &&
                (operator_flags&RING_OPERATOR_RUN);
            *turn_out=ring_maneuver_apply(&maneuver,*turn_out,now,heading_deg,wheel_revs,heading_rate,
                eligible,diagnostics.path_valid && !diagnostics.stop &&
                diagnostics.phase>=RING_ENTER && diagnostics.phase<=RING_RECOVER);
        } else *turn_out=ring_force_apply(&force_control,*turn_out,now,heading_deg,wheel_revs,
            heading_rate,force_side,force_requested,
            fresh && !ring_control_stop(speed_cap,3,ring_fault,fresh) &&
            vision_state==3 && original_forward>0 && bias_samples &&
            (operator_flags&RING_OPERATOR_RUN),diagnostics.path_valid &&
            (diagnostics.phase==RING_ENTER || diagnostics.phase==RING_CIRCLE));
        /* Evaluate the maneuver BEFORE its speed: the exact 1000ms
         * transition starts a ramp, never a one-frame 100/90 RPM step. */
        if(RING_TRACK_MODE && RING_TIMED_ENTRY && maneuver.stage)
            *forward_target=ring_entry_speed_apply(&entry_speed,original_forward,
                speed_cap,maneuver.stage,now,fresh && !ring_fault && !diagnostics.stop &&
                runtime_ready && (operator_flags&RING_OPERATOR_RUN));
        else *forward_target=ring_control_speed(original_forward,speed_cap,3,ring_fault,fresh);
    }
    if(maneuver.stage && maneuver.stage<4u && (abort_pending || !runtime_ready ||
       !(operator_flags&RING_OPERATOR_RUN) || original_forward<=0.f)) {
        maneuver.stage=4;maneuver.reason=6;maneuver.target_rate=0;
    }
    if(!ring_active || !runtime_ready || !(operator_flags&RING_OPERATOR_RUN) || original_forward<=0.f)
        memset(&entry_speed,0,sizeof(entry_speed));
    if(runtime_ready && (operator_flags&RING_OPERATOR_RUN) && !ring_active && original_forward>0.f){
        preceding_target=original_forward;preceding_target_ms=now;preceding_target_valid=1;
    }else if(!runtime_ready || !(operator_flags&RING_OPERATOR_RUN) || original_forward<=0.f)
        preceding_target_valid=0;
    diagnostics.held_target10=entry_speed.valid?(int16_t)(entry_speed.held*10.f):0;
    diagnostics.speed_goal10=entry_speed.valid?(int16_t)(entry_speed.goal*10.f):0;
    diagnostics.motion_age_ms=motion_sample_valid?now-motion_sample_ms:UINT32_MAX;
    diagnostics.maneuver_stage=(uint8_t)maneuver.stage;
    diagnostics.maneuver_turn10=(int16_t)fmaxf(-32768.f,fminf(32767.f,maneuver.progress*10.f));
    diagnostics.maneuver_remaining10=(int16_t)fmaxf(-32768.f,fminf(32767.f,
        maneuver.stage?(RING_EXIT_MIN_DEG-maneuver.progress)*10.f:0.f));
    diagnostics.maneuver_elapsed_ms=(uint16_t)fminf(65535.f,
        maneuver.stage?(float)(now-(maneuver.stage==1 || !maneuver.turn_ms?maneuver.started_ms:maneuver.turn_ms)):0.f);
    diagnostics.force_active=(uint8_t)force_control.active;
    diagnostics.force_reason=(uint8_t)force_control.reason;
    diagnostics.force_side=(int8_t)(force_control.begun?force_control.side:0);
    diagnostics.force_target10=force_control.active?(int16_t)(force_control.side*RING_FORCE_RATE_DPS*10):0;
    diagnostics.force_rate10=(int16_t)fmaxf(-32768.f,fminf(32767.f,heading_rate*10));
    if(maneuver.stage){
        diagnostics.force_active=(uint8_t)(ring_active && maneuver.stage==2u);
        diagnostics.force_reason=(uint8_t)maneuver.reason;diagnostics.force_side=(int8_t)maneuver.side;
        diagnostics.force_target10=(int16_t)(maneuver.target_rate*10.f);
    }
    diagnostics.pair_ok=(uint8_t)(pair_packet_valid && now-pair_last_ms<=RING_LINK_TIMEOUT_MS);
    read_diagnostics(now);
    read_entry_diagnostics(now);
    read_geometry_diagnostics(now);
    read_topology_diagnostics(now);
    read_mouth_diagnostics(now);
    diagnostics.control_dt_ms = now - diagnostics.now_ms;
    diagnostics.now_ms = now;
    diagnostics.command_age_ms = now - last_command_ms;
    diagnostics.wheel_mrev = (int32_t)(wheel_revs * 1000.0f);
    diagnostics.active = ring_active;
    diagnostics.run_requested=(uint8_t)((operator_flags&RING_OPERATOR_RUN)!=0u);
    diagnostics.standing=runtime_ready;
    diagnostics.output_state = (uint8_t)vision_state;
    diagnostics.applied_boost = (int16_t)applied_boost;
    diagnostics.original_target10 = (int16_t)(original_forward * 10.0f);
    diagnostics.final_target10 = (int16_t)(*forward_target * 10.0f);
    diagnostics.original_turn10 = (int16_t)(original_turn * 10.0f);
    diagnostics.final_turn10 = (int16_t)(*turn_out * 10.0f);

    packet = (ring_packet_t){{0}};
    packet.word[RL_MAGIC] = RING_LINK_MAGIC;
    packet.word[RL_EPOCH] = (int32_t)epoch;
    packet.word[RM_MS] = (int32_t)now;
    packet.word[RM_YAW_MDEG] = (int32_t)(heading_deg * 1000.0f);
    packet.word[RM_WHEEL_MREV] = (int32_t)(wheel_revs * 1000.0f);
    packet.word[RM_READY] = ring_motion_status(runtime_ready | ((now >= 2000u && bias_samples) ? 2u : 0u),
        maneuver.stage,diagnostics.maneuver_turn10);
    packet.word[RM_RESERVED] = ring_motion_operator_version(operator_flags|
        (abort_pending?RING_OPERATOR_ABORT_RING:0u),RING_PAIR_VERSION);
    ring_link_write(ring_motion_mailbox, &packet, &feedback_sequence);
}
