/* Final runtime outputs using real command mailboxes, not yaw-only claims
 * of vehicle motion. Tests speed commands; physical response needs COM9. */
#include "ring_entry_speed.h"
#include "ring_runtime.h"
#include "ring_link.h"
#include "ring_uart_diag.h"
#include <stdlib.h>
stc_SCB_t uart_port;char uart_capture[32768];unsigned uart_capture_count;
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL entry speed:%d %s\n",__LINE__,#c);exit(1);}}while(0)
static uint32_t seq,epoch;
static float forward,turn;
static void ticks(int raw,unsigned n,int ready)
{while(n--)(void)ring_runtime_tick((int16_t)raw,(uint8_t)ready);}
static void apply(float normal,int state)
{
 forward=normal;turn=17;
 ring_runtime_observe_motion(-36.5f,2.25f,-3.5f,-1234,2345);
 ring_runtime_apply(&forward,&turn,0,state);
}
static void boot(void)
{
 memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);seq=0;
 ring_runtime_init(.1f);ring_runtime_set_operator(1,0);
 ticks(0,2000,0);ticks(0,1,1);apply(150,1);
 epoch=ring_runtime_diagnostics()->command_epoch;
}
static void command(int side,int phase,int cap,int timed,float normal,int state)
{
 ring_packet_t p={{0}};p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
 p.word[RC_CAP]=ring_command_cap_turn(cap,0);
 p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
 p.word[RC_DETECTED]=1;
 p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,0,1,0,1,0,80,-side);
 if(phase==RING_COOLDOWN)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|(1u<<RING_COMMAND_RELEASE_SHIFT));
 if(timed)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_TIMED_ENTRY);
 ring_link_write(ring_command_mailbox,&p,&seq);apply(normal,state);
}
static void profile(int side)
{
 unsigned i;float previous;const ring_runtime_diagnostics_t *d;
 boot();ticks(0,46,1);
 /* Recognition-frame ordinary steering already changed; latch the last
  * delivered150, not this frame's125 and not the approach100 cap. */
 command(side,RING_APPROACH,100,1,125,1);
 CHECK(forward==150 && turn==0);
 for(i=50;i<RING_MANEUVER_DELAY_MS;i+=50){ticks(0,50,1);command(side,RING_APPROACH,100,1,i%2?125:150,1);CHECK(forward==150);}
 ticks(0,49,1);command(side,RING_APPROACH,100,1,125,1);
 d=ring_runtime_diagnostics();CHECK(d->maneuver_stage==1 && forward==150 && d->held_target10==1500);
 ticks(0,1,1);command(side,RING_APPROACH,100,1,150,1);
 CHECK(ring_runtime_diagnostics()->maneuver_stage==2 && forward>149.9f && forward<150);
 previous=forward;
 for(i=0;i<10;++i){
  ticks(side*1000,50,1);command(side,RING_ENTER,90,1,150,3);
  CHECK(forward<previous && previous-forward<=2.001f && forward>90);previous=forward;
 }
 CHECK(forward>129.9f && forward<130.1f);
 for(i=0;i<6;++i){ticks(side*1000,50,1);command(side,RING_ENTER,90,1,150,3);}
 CHECK(ring_runtime_diagnostics()->maneuver_stage==3 && forward>117.9f && forward<118.1f);
 /* Actual visual CIRCLE keeps the same gyro lap and speed ramp; it must not
  * suddenly snap to90. Repeated commands cannot re-latch150. */
 for(i=0;i<20;++i){
  previous=forward;ticks(0,50,1);command(side,RING_CIRCLE,90,0,150,3);
  CHECK(forward<=previous && previous-forward<=2.001f && forward>=90);
 }
 CHECK(forward==90);
 d=ring_runtime_diagnostics();
 CHECK(d->measured_forward10==-365 && d->pitch100==225 && d->speed_tilt100==-350);
 CHECK(d->left_pwm==-1234 && d->right_pwm==2345 && d->motion_age_ms==0);
 command(side,RING_COOLDOWN,-1,0,137,1);CHECK(forward==137 && turn==17);
 CHECK(!ring_runtime_diagnostics()->held_target10);
 ticks(0,46,1);apply(125,1);CHECK(forward==125 && turn==17);
}
static void release_cases(void)
{
 unsigned k;
 for(k=0;k<5;++k){
  boot();command(1,RING_APPROACH,100,1,150,1);ticks(0,46,1);
  if(k==0){ring_runtime_set_operator(0,0);command(1,RING_APPROACH,100,1,0,1);CHECK(forward==0 && turn==17);}
  if(k==1){ticks(0,1,0);command(1,RING_APPROACH,100,1,0,1);CHECK(forward==0 && turn==17);}
  if(k==2){ticks(0,301,1);apply(139,1);CHECK(forward==139 && turn==17 && !ring_runtime_diagnostics()->active);}
  if(k==3){command(1,RING_COOLDOWN,-1,0,129,1);CHECK(forward==129 && turn==17);}
  if(k==4){apply(0,1);CHECK(forward==0 && turn==17);apply(150,1);CHECK(forward==150);}
 }
 /* A normal100/125/150 target bypasses every new profile parameter. */
 boot();for(k=1;k<=150;++k){apply((float)k,1);CHECK(forward==(float)k && turn==17);}
}
static void bounds(void)
{
 ring_entry_speed_t s;float out;
 ring_entry_speed_begin(&s,0xffffff00u,150);
 out=ring_entry_speed_apply(&s,150,90,2,744u,1);
 CHECK(out==146); /* uint32 wrap; delayed poll cannot jump by40 RPM */
 out=ring_entry_speed_apply(&s,0,90,2,790u,1);CHECK(out==0 && !s.valid);
 ring_entry_speed_begin(&s,0,125);
 CHECK(ring_entry_speed_apply(&s,150,100,1,999u,1)==125);
 CHECK(ring_entry_speed_apply(&s,150,90,4,1000u,1)==150 && !s.valid);
}
static void wire(void)
{
 unsigned i;char *p,*end;unsigned commas=0;
 boot();command(1,RING_APPROACH,100,1,125,1);
 for(i=0;i<180;++i){uart_port.used=0;ring_uart_diag_poll(0,0,0);}
 uart_capture[uart_capture_count]=0;
 CHECK(strstr(uart_capture,"#RING,28,") && strstr(uart_capture,"meas10,pitch100,tilt100,pwmL,pwmR,m_age"));
 p=strstr(uart_capture,"\r\nK,");CHECK(p);p+=2;end=strstr(p,"\r\n");CHECK(end);
 while(p<end)if(*p++==',')++commas;
 CHECK(commas==29 && strstr(uart_capture,",1500,1500,-365,225,-350,-1234,2345,0\r\n"));
}
int main(void)
{
 profile(-1);profile(1);release_cases();bounds();wire();
 puts("PASS: pre-recognition150 held for configured delay,100/90 caps cannot brake hold,40RPM/s turn/CIRCLE ramp, both sides, original outputs/zero/operator/stale/stand loss/cooldown, clock wrap and delayed-poll bound, read-only motion observations and K29 wire fields");
 return 0;
}
