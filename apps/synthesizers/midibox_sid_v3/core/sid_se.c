// $Id$
/*
 * MBSID Sound Engine Routines
 *
 * ==========================================================================
 *
 *  Copyright (C) 2009 Thorsten Klose (tk@midibox.org)
 *  Licensed for personal non-commercial use only.
 *  All other rights reserved.
 * 
 * ==========================================================================
 */

/////////////////////////////////////////////////////////////////////////////
// Include files
/////////////////////////////////////////////////////////////////////////////

#include <mios32.h>
#include <string.h>

#include <sid.h>
#include <notestack.h>

#include "sid_se.h"
#include "sid_se_l.h"
#include "sid_patch.h"
#include "sid_random.h"
#include "sid_asid.h"


/////////////////////////////////////////////////////////////////////////////
// for optional debugging messages via DEBUG_MSG (defined in mios32_config.h)
// should be at least 1 for sending error messages
/////////////////////////////////////////////////////////////////////////////
#define DEBUG_VERBOSE_LEVEL 1


// measure performance with the stopwatch
#define STOPWATCH_PERFORMANCE_MEASURING 1


/////////////////////////////////////////////////////////////////////////////
// Global Variables
/////////////////////////////////////////////////////////////////////////////

u8 sid_se_speed_factor;

sid_se_vars_t sid_se_vars[SID_NUM];
sid_se_clk_t sid_se_clk;
sid_se_voice_t sid_se_voice[SID_NUM][SID_SE_NUM_VOICES];
sid_se_filter_t sid_se_filter[SID_NUM][SID_SE_NUM_FILTERS];
sid_se_lfo_t sid_se_lfo[SID_NUM][SID_SE_NUM_LFO];
sid_se_env_t sid_se_env[SID_NUM][SID_SE_NUM_ENV];
sid_se_wt_t sid_se_wt[SID_NUM][SID_SE_NUM_WT];
sid_se_seq_t sid_se_seq[SID_NUM];


#include "sid_se_frq_table.inc"
#include "sid_se_env_table.inc"
#include "sid_se_lfo_table.inc"
#include "sid_se_sin_table.inc"


/////////////////////////////////////////////////////////////////////////////
// Initialisation
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_Init(u32 mode)
{
  int sid;

  // currently the sound engine runs x times faster than MBSID V2 (will be user-selectable in future)
  sid_se_speed_factor = 2;

  // ensure that clock generator structure is cleared
  memset(&sid_se_clk, 0, sizeof(sid_se_clk_t));

  // initialize structures of each SID engine
  for(sid=0; sid<SID_NUM; ++sid)
    SID_SE_InitStructs(sid);

  // start timer
  // TODO: increase sid_se_speed_factor once performance has been evaluated
  MIOS32_TIMER_Init(2, 2000 / sid_se_speed_factor, SID_SE_Update, MIOS32_IRQ_PRIO_MID);

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Initialises the structures of a SID sound engine
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_InitStructs(u8 sid)
{
  int i;

  sid_se_vars_t *vars = (sid_se_vars_t *)&sid_se_vars[sid];
  memset(vars, 0, sizeof(sid_se_vars_t));
  vars->sid = sid;

  for(i=0; i<SID_SE_NUM_VOICES; ++i) {
    u8 voice = i;
    sid_se_voice_t *v = (sid_se_voice_t *)&sid_se_voice[sid][voice];
    memset(v, 0, sizeof(sid_se_voice_t));

    v->sid = sid; // cross-reference to assigned SID
    v->phys_sid = 2*sid + ((voice>=3) ? 1 : 0); // reference to physical SID (chip)
    v->voice = voice; // cross-reference to assigned voice
    v->phys_voice = voice % 3; // reference to physical voice
    v->phys_sid_voice = (sid_voice_t *)&sid_regs[v->phys_sid].v[v->phys_voice];
    v->voice_patch = (sid_se_voice_patch_t *)&sid_patch[sid].L.voice[voice];
    v->clk = (sid_se_clk_t *)&sid_se_clk;
    v->mod_dst_pitch = &sid_se_vars[sid].mod_dst[SID_SE_MOD_DST_PITCH1 + voice];
    v->mod_dst_pw = &sid_se_vars[sid].mod_dst[SID_SE_MOD_DST_PW1 + voice];
    v->trg_mask_note_on = (u8 *)&sid_patch[sid].L.trg_matrix[SID_SE_TRG_NOn][0];
    v->trg_mask_note_off = (u8 *)&sid_patch[sid].L.trg_matrix[SID_SE_TRG_NOff][0];
    v->trg_dst = (u8 *)&sid_se_vars[sid].triggers;

    NOTESTACK_Init(&v->notestack,
		   NOTESTACK_MODE_PUSH_TOP,
		   &v->notestack_items[0],
		   SID_SE_NOTESTACK_SIZE);
    v->split_upper = 0x7f;
    v->transpose = 0x40;
    v->pitchbender = 0x80;
  }

  for(i=0; i<SID_SE_NUM_FILTERS; ++i) {
    u8 filter = i;
    sid_se_filter_t *f = (sid_se_filter_t *)&sid_se_filter[sid][filter];
    memset(f, 0, sizeof(sid_se_filter_t));

    f->sid = sid; // cross-reference to assigned SID
    f->phys_sid = 2*sid + filter; // reference to physical SID (chip)
    f->filter = filter; // cross-reference to assigned filter
    f->phys_sid_regs = (sid_regs_t *)&sid_regs[f->phys_sid];
    f->filter_patch = (sid_se_filter_patch_t *)&sid_patch[sid].L.filter[filter];
    f->mod_dst_filter = &sid_se_vars[sid].mod_dst[SID_SE_MOD_DST_FIL1 + filter];
    f->mod_dst_volume = &sid_se_vars[sid].mod_dst[SID_SE_MOD_DST_VOL1 + filter];
  }

  for(i=0; i<SID_SE_NUM_LFO; ++i) {
    u8 lfo = i;
    sid_se_lfo_t *l = (sid_se_lfo_t *)&sid_se_lfo[sid][lfo]; 
    memset(l, 0, sizeof(sid_se_lfo_t));

    l->sid = sid; // cross-reference to assigned SID
    l->lfo = lfo; // cross-reference to LFO number
    l->lfo_patch = (sid_se_lfo_patch_t *)&sid_patch[sid].L.lfo[lfo];
    l->mod_src_lfo = &sid_se_vars[sid].mod_src[SID_SE_MOD_SRC_LFO1 + lfo];
    l->mod_dst_lfo_depth = &sid_se_vars[sid].mod_dst[SID_SE_MOD_DST_LD1 + lfo];
    l->mod_dst_lfo_rate = &sid_se_vars[sid].mod_dst[SID_SE_MOD_DST_LR1 + lfo];
    l->trg_mask_lfo_period = (u8 *)&sid_patch[sid].L.trg_matrix[SID_SE_TRG_L1P + lfo][0];
    l->trg_dst = (u8 *)&sid_se_vars[sid].triggers;
  }

  for(i=0; i<SID_SE_NUM_ENV; ++i) {
    u8 env = i;
    sid_se_env_t *e = (sid_se_env_t *)&sid_se_env[sid][env];
    memset(e, 0, sizeof(sid_se_env_t));

    e->sid = sid; // cross-reference to assigned SID
    e->env = env; // cross-reference to ENV number
    e->env_patch = (sid_se_env_patch_t *)&sid_patch[sid].L.env[env];
    e->mod_src_env = &sid_se_vars[sid].mod_src[SID_SE_MOD_SRC_ENV1 + env];
    e->trg_mask_env_sustain = (u8 *)&sid_patch[sid].L.trg_matrix[SID_SE_TRG_E1S + env][0];
    e->trg_dst = (u8 *)&sid_se_vars[sid].triggers;
  }

  for(i=0; i<SID_SE_NUM_WT; ++i) {
    u8 wt = i;
    sid_se_wt_t *w = (sid_se_wt_t *)&sid_se_wt[sid][wt];
    memset(w, 0, sizeof(sid_se_wt_t));

    w->sid = sid; // cross-reference to assigned SID
    w->wt = wt; // cross-reference to WT number
    w->wt_patch = (sid_se_wt_patch_t *)&sid_patch[sid].L.wt[wt];
    w->mod_src_wt = &sid_se_vars[sid].mod_src[SID_SE_MOD_SRC_WT1 + wt];
    w->mod_dst_wt = &sid_se_vars[sid].mod_dst[SID_SE_MOD_DST_WT1 + wt];
    w->trg_src = (u8 *)&sid_se_vars[sid].triggers;
  }

  sid_se_seq_t *s = (sid_se_seq_t *)&sid_se_seq[sid];
  memset(s, 0, sizeof(sid_se_seq_t));

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Sound Engine Update Cycle
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_Update(void)
{
  // exit in ASID mode
  if( SID_ASID_ModeGet() != SID_ASID_MODE_OFF )
    return 0;


  ///////////////////////////////////////////////////////////////////////////
  // Clock
  ///////////////////////////////////////////////////////////////////////////

  SID_SE_Clk((sid_se_clk_t *)&sid_se_clk);


  ///////////////////////////////////////////////////////////////////////////
  // Engine
  ///////////////////////////////////////////////////////////////////////////

  // TODO: support for multiple SIDs
  u8 sid = 0;
  sid_se_engine_t engine = sid_patch[sid].engine;

#if STOPWATCH_PERFORMANCE_MEASURING >= 1
  APP_StopwatchReset();
#endif

  switch( engine ) {
    case SID_SE_LEAD: SID_SE_L_Update(sid); break;
  }

#if STOPWATCH_PERFORMANCE_MEASURING == 1
  APP_StopwatchCapture();
#endif

  // update SID registers
  SID_Update(0);

#if STOPWATCH_PERFORMANCE_MEASURING == 2
  APP_StopwatchCapture();
#endif

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Clock
/////////////////////////////////////////////////////////////////////////////

// called from NOTIFY_MIDI_Rx() in app.c
s32 SID_SE_IncomingRealTimeEvent(u8 event)
{
  // ensure that following operations are atomic
  MIOS32_IRQ_Disable();

  switch( event ) {
    case 0xf8: // MIDI Clock
      // we've measure a new delay between two F8 events
      sid_se_clk.incoming_clk_delay = sid_se_clk.incoming_clk_ctr;
      sid_se_clk.incoming_clk_ctr = 0;

      // increment clock counter by 4
      sid_se_clk.clk_req_ctr += 4 - sid_se_clk.sent_clk_ctr;
      sid_se_clk.sent_clk_ctr = 0;

      // determine new clock delay
      sid_se_clk.sent_clk_delay = sid_se_clk.incoming_clk_delay >> 2;
      break;

    case 0xfa: // MIDI Clock Start
      sid_se_clk.event.MIDI_START_REQ = 1;

      // Auto Mode: immediately switch to slave mode
      // TODO

      // ensure that incoming clock counter < 0xfff (means: no clock received for long time)
      if( sid_se_clk.incoming_clk_ctr > 0xfff )
	sid_se_clk.incoming_clk_ctr = 0;

      // cancel all requested clocks
      sid_se_clk.clk_req_ctr = 0;
      sid_se_clk.sent_clk_ctr = 3;
      break;

    case 0xfb: // MIDI Clock Continue
      sid_se_clk.event.MIDI_CONTINUE_REQ = 1;
      break;

    case 0xfc: // MIDI Clock Stop
      sid_se_clk.event.MIDI_STOP_REQ = 1;
      break;
  }

  MIOS32_IRQ_Enable();

  return 0; // no error
}

s32 SID_SE_Clk(sid_se_clk_t *clk)
{
  // clear previous clock events
  clk->event.ALL_SE = 0;

  // increment the clock counter, used to measure the delay between two F8 events
  // see also SID_SE_IncomingClk()
  if( clk->incoming_clk_ctr != 0xffff )
    ++clk->incoming_clk_ctr;

  // now determine master/slave flag depending on ensemble setup
  u8 clk_mode = 2; // TODO: selection via ensemble
  u8 slave_mode = 0;
  switch( clk_mode ) {
    case 0: // master
      slave_mode = 0;
      break;

    case 1: // slave
      slave_mode = 1;
      break;

    default: // auto
      // slave mode so long incoming clock counter < 0xfff
      slave_mode = clk->incoming_clk_ctr < 0xfff;
      break;
  }

  // only used in slave mode:
  // decrement sent clock delay, send interpolated clock events 3 times
  if( !--clk->sent_clk_delay && clk->sent_clk_ctr < 3 ) {
    ++clk->sent_clk_ctr;
    ++clk->clk_req_ctr;
    clk->sent_clk_delay = clk->incoming_clk_delay >> 2; // to generate clock 4 times
  }

  // handle MIDI Start Event
  if( clk->event.MIDI_START_REQ ) {
    // take over
    clk->event.MIDI_START_REQ = 0;
    clk->event.MIDI_START = 1;

    // reset counters
    clk->global_clk_ctr6 = 0;
    clk->global_clk_ctr24 = 0;
  }


  // handle MIDI Clock Event
  if( clk->clk_req_ctr ) {
    // decrement counter by one (if there are more requests, they will be handled on next handler invocation)
    --clk->clk_req_ctr;

    if( slave_mode )
      clk->event.CLK = 1;
  } else {
    if( !slave_mode ) {
      // TODO: check timer overrun flag
      // temporary solution: use BPM counter
      if( ++clk->tmp_bpm_ctr > 5 ) { // ca. 120 BPM * 4
	clk->tmp_bpm_ctr = 0;
	clk->event.CLK = 1;
      }
    }
  }
  if( clk->event.CLK ) {
    // increment global clock counter (for Clk/6 and Clk/24 divider)
    if( ++clk->global_clk_ctr6 >= 6 ) {
      clk->global_clk_ctr6 = 0;
      if( ++clk->global_clk_ctr24 >= 4 )
	clk->global_clk_ctr24 = 0;
    }
  }

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Arpeggiator
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_Arp(sid_se_voice_t *v)
{
  sid_se_voice_arp_mode_t arp_mode = (sid_se_voice_arp_mode_t)v->voice_patch->arp_mode;
  sid_se_voice_arp_speed_div_t arp_speed_div = (sid_se_voice_arp_speed_div_t)v->voice_patch->arp_speed_div;
  sid_se_voice_arp_gl_rng_t arp_gl_rng = (sid_se_voice_arp_gl_rng_t)v->voice_patch->arp_gl_rng;

  u8 new_note_req = 0;
  u8 first_note_req = 0;
  u8 gate_clr_req = 0;

  // check if arp sync requested
  if( v->clk->event.MIDI_START || v->arp_state.SYNC_ARP ) {
    // set arp counters to max values (forces proper reset)
    v->arp_note_ctr = 0xff;
    v->arp_oct_ctr = 0xff;
    // reset ARP Up flag (will be set to 1 with first note)
    v->arp_state.ARP_UP = 0;
    // request first note (for oneshot function)
    first_note_req = 1;
    // reset divider if not disabled or if arp synch on MIDI clock start event
    if( v->clk->event.MIDI_START || !arp_mode.SYNC ) {
      v->arp_div_ctr = 0xff;
      v->arp_gl_ctr = 0xff;
      // request new note
      new_note_req = 1;
    }
  }

  // if clock sync event:
  if( v->clk->event.CLK ) {
    // increment clock divider
    // reset divider if it already has reached the target value
    int inc = 1;
    if( arp_mode.CAC ) {
      // in CAC mode: increment depending on number of pressed keys
      inc = v->notestack.len;
      if( !inc ) // at least one increment
	inc = 1;
    }
    // handle divider
    // TODO: improve this!
    u8 div_ctr = v->arp_div_ctr + inc;
    u8 speed_div = arp_speed_div.DIV + 1;
    while( div_ctr >= speed_div ) {
      div_ctr -= speed_div;
      // request new note
      new_note_req = 1;
      // request gate clear if voice not active anymore (required for Gln=>Speed)
      if( !v->state.VOICE_ACTIVE )
	gate_clr_req = 1;
    }
    v->arp_div_ctr = div_ctr;

    // increment gatelength counter
    // reset counter if it already has reached the target value
    if( ++v->arp_gl_ctr > arp_gl_rng.GATELENGTH ) {
      // reset counter
      v->arp_gl_ctr = 0;
      // request gate clear
      gate_clr_req = 1;
    }
  }

  // check if HOLD mode has been deactivated - disable notes in this case
  u8 disable_notes = !arp_mode.HOLD && v->arp_state.HOLD_SAVED;
  // store HOLD flag in MIDI voice record
  v->arp_state.HOLD_SAVED = arp_mode.HOLD;

  // skip the rest if arp is disabled
  if( disable_notes || !arp_mode.ENABLE ) {
    // check if arp was active before (for proper 1->0 transition when ARP is disabled)
    if( v->arp_state.ARP_ACTIVE ) {
      // notify that arp is not active anymore
      v->arp_state.ARP_ACTIVE = 0;
      // clear note stack (especially important in HOLD mode!)
      NOTESTACK_Clear(&v->notestack);
      // propagate Note Off through trigger matrix
      v->trg_dst[0] |= v->trg_mask_note_off[0] & 0xc0; // gates handled separately
      v->trg_dst[1] |= v->trg_mask_note_off[1];
      v->trg_dst[2] |= v->trg_mask_note_off[2];
      // request gate clear
      v->state.GATE_SET_REQ = 0;
      v->state.GATE_CLR_REQ = 1;
    }
  } else {
    // notify that arp is active (for proper 1->0 transition when ARP is disabled)
    v->arp_state.ARP_ACTIVE = 1;

    // check if voice not active anymore (not valid in HOLD mode) or gate clear has been requested
    // skip voice active check in hold mode and voice sync mode
    if( gate_clr_req || (!arp_mode.HOLD && !arp_mode.SYNC && !v->state.VOICE_ACTIVE) ) {
      // forward this to note handler if gate is not already deactivated
      if( v->state.GATE_ACTIVE ) {
	// propagate Note Off through trigger matrix
	v->trg_dst[0] |= v->trg_mask_note_off[0] & 0xc0; // gates handled separately
	v->trg_dst[1] |= v->trg_mask_note_off[1];
	v->trg_dst[2] |= v->trg_mask_note_off[2];
	// request gate clear
	v->state.GATE_SET_REQ = 0;
	v->state.GATE_CLR_REQ = 1;
      }
    }

    // check if a new arp note has been requested
    // skip if note counter is 0xaa (oneshot mode)
    if( new_note_req && v->arp_note_ctr != 0xaa ) {
      // reset gatelength counter
      v->arp_gl_ctr = 0;
      // increment note counter
      // if max value of arp note counter reached, reset it
      if( ++v->arp_note_ctr >= v->notestack.len )
	v->arp_note_ctr = 0;
      // if note is zero, reset arp note counter
      if( !v->notestack_items[v->arp_note_ctr].note )
	v->arp_note_ctr = 0;


      // dir modes
      u8 note_number = 0;
      if( arp_mode.DIR >= 6 ) { // Random
	note_number = SID_RANDOM_Gen_Range(0, v->notestack.len-1);
      } else {
	u8 new_note_up;
	if( arp_mode.DIR >= 2 && arp_mode.DIR <= 5 ) { // Alt Mode 1 and 2
	  // toggle ARP_UP flag each time the arp note counter is zero
	  if( !v->arp_note_ctr ) {
	    v->arp_state.ARP_UP ^= 1;
	    // increment note counter to prevent double played notes
	    if( arp_mode.DIR >= 2 && arp_mode.DIR <= 3 ) // only in Alt Mode 1
	      if( ++v->arp_note_ctr >= v->notestack.len )
		v->arp_note_ctr = 0;
	  }

	  // direction depending on Arp Up/Down and Alt Up/Down flag
	  if( (arp_mode.DIR & 1) == 0 ) // UP modes
	    new_note_up = v->arp_state.ARP_UP;
	  else // DOWN modes
	    new_note_up = !v->arp_state.ARP_UP;
	} else {
	  // direction depending on arp mode 0 or 1
	  new_note_up = (arp_mode.DIR & 1) == 0;
	}

	if( new_note_up )
	  note_number = v->arp_note_ctr;
	else
	  if( v->notestack.len )
	    note_number = v->notestack.len - v->arp_note_ctr - 1;
	  else
	    note_number = 0;
      }

      int new_note = v->notestack_items[note_number].note;

      // now check for oneshot mode: if note is 0, or if note counter and oct counter is 0, stop here
      if( !first_note_req && arp_speed_div.ONESHOT &&
	  (!new_note || (note_number == 0 && v->arp_oct_ctr == 0)) ) {
	// set note counter to 0xaa to stop ARP until next reset
	v->arp_note_ctr = 0xaa;
	// don't play new note
	new_note = 0;
      }

      // only play if note is not zero
      if( new_note ) {
	// if first note has been selected, increase octave until max value is reached
	if( first_note_req )
	  v->arp_oct_ctr = 0;
	else if( note_number == 0 ) {
	  ++v->arp_oct_ctr;
	}
	if( v->arp_oct_ctr > arp_gl_rng.OCTAVE_RANGE )
	  v->arp_oct_ctr = 0;

	// transpose note
	new_note += 12*v->arp_oct_ctr;

	// saturate octave-wise
	while( new_note >= 0x6c ) // use 0x6c instead of 128, since range 0x6c..0x7f sets frequency to 0xffff...
	  new_note -= 12;

	// store new arp note
	v->arp_note = new_note;

	// forward gate set request if voice is active and gate not active
	if( v->state.VOICE_ACTIVE ) {
	  v->state.GATE_CLR_REQ = 0; // ensure that gate won't be cleared by previous CLR_REQ
	  if( !v->state.GATE_ACTIVE ) {
	    // set gate
	    v->state.GATE_SET_REQ = 1;

	    // propagate Note On through trigger matrix
	    v->trg_dst[0] |= v->trg_mask_note_on[0] & 0xc0; // gates handled separately
	    v->trg_dst[1] |= v->trg_mask_note_on[1];
	    v->trg_dst[2] |= v->trg_mask_note_on[2];
	  }
	}
      }
    }
  }

  // clear arp sync flag
  v->arp_state.SYNC_ARP = 0;

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Voice Gate
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_Gate(sid_se_voice_t *v)
{
  s32 change_pitch = 1;

  // voice disable handling (allows to turn on/off voice via waveform parameter)
  sid_se_voice_waveform_t waveform = (sid_se_voice_waveform_t)v->voice_patch->waveform;
  if( v->state.VOICE_DISABLED ) {
    if( !waveform.VOICE_OFF ) {
      v->state.VOICE_DISABLED = 0;
      if( v->state.VOICE_ACTIVE )
	v->state.GATE_SET_REQ = 1;
    }
  } else {
    if( waveform.VOICE_OFF ) {
      v->state.VOICE_DISABLED = 1;
      v->state.GATE_CLR_REQ = 1;
    }
  }

  // if gate not active: ignore clear request
  if( !v->state.GATE_ACTIVE )
    v->state.GATE_CLR_REQ = 0;

  // gate set/clear request?
  if( v->state.GATE_CLR_REQ ) {
    v->state.GATE_CLR_REQ = 0;

    // clear SID gate flag if GSA (gate stays active) function not enabled
    sid_se_voice_flags_t voice_flags = (sid_se_voice_flags_t)v->voice_patch->flags;
    if( !voice_flags.GSA )
      v->phys_sid_voice->gate = 0;

    // gate not active anymore
    v->state.GATE_ACTIVE = 0;
  } else if( v->state.GATE_SET_REQ || v->state.OSC_SYNC_IN_PROGRESS ) {
    // don't set gate if oscillator disabled
    if( !waveform.VOICE_OFF ) {
      sid_se_opt_flags_t opt_flags = (sid_se_opt_flags_t)sid_patch[v->sid].opt_flags;

      // delay note so long 16bit delay counter != 0
      if( v->set_delay_ctr ) {
	int delay_inc = v->voice_patch->delay;
	// if ABW (ADSR bug workaround) active: use at least 30 mS delay
	if( opt_flags.ABW ) {
	  delay_inc += 25;
	  if( delay_inc > 255 )
	    delay_inc = 255;
	}

	int set_delay_ctr = v->set_delay_ctr + sid_se_env_table[delay_inc] / sid_se_speed_factor;
	if( set_delay_ctr < 0xffff ) {
	  // no overrun
	  v->set_delay_ctr = set_delay_ctr;
	  // don't change pitch so long delay is active
	  change_pitch = 0;
	} else {
	  // overrun: clear counter to disable delay
	  v->set_delay_ctr = 0x0000;
	}
      } else {
	// acknowledge the set request
	v->state.GATE_SET_REQ = 0;

	// if ABW (ADSR bug workaround) function active: update ADSR registers now!
	if( opt_flags.ABW ) {
	  v->phys_sid_voice->ad = v->voice_patch->ad;
	  v->phys_sid_voice->sr = v->voice_patch->sr;
	}

	// optional OSC synchronisation
	// TODO: will be different for drum and multi engine
	u8 osc_phase;
	if( !v->state.OSC_SYNC_IN_PROGRESS && (osc_phase=sid_patch[v->sid].L.osc_phase) ) {
	  // notify that OSC synchronisation has been started
	  v->state.OSC_SYNC_IN_PROGRESS = 1;
	  // set test flag for one update cycle
	  v->phys_sid_voice->test = 1;
	  // set pitch depending on selected oscillator phase to achieve an offset between the waveforms
	  // This approach has been invented by Wilba! :-)
	  u32 reference_frq = 16779; // 1kHz
	  u32 osc_sync_frq;
	  switch( v->voice % 3 ) {
	    case 1: osc_sync_frq = (reference_frq * (1000 + 4*(u32)osc_phase)) / 1000; break;
	    case 2: osc_sync_frq = (reference_frq * (1000 + 8*(u32)osc_phase)) / 1000; break;
	    default: osc_sync_frq = reference_frq; // (Voice 0 and 3)
	  }
	  v->phys_sid_voice->frq_l = osc_sync_frq & 0xff;
	  v->phys_sid_voice->frq_h = osc_sync_frq >> 8;
	  // don't change pitch for this update cycle!
	  change_pitch = 0;
	} else if( v->phys_sid_voice->test ) {
	  // clear test flag
	  v->phys_sid_voice->test = 0;
	  // don't change pitch for this update cycle!
	  change_pitch = 0;
	  // ensure that pitch handler will re-calculate pitch frequency on next update cycle
	  v->linear_frq = 0;
	} else {
	  // this code is also executed if OSC synchronisation disabled
	  // set the gate flag
	  v->phys_sid_voice->gate = 1;
	  // OSC sync finished
	  v->state.OSC_SYNC_IN_PROGRESS = 0;
	}
      }
    }
    v->state.GATE_ACTIVE = 1;
  }

  return change_pitch;
}


s32 SID_SE_NoteRestart(sid_se_voice_t *v)
{
  // request gate if voice is active (and request clear for proper ADSR handling)
  v->state.GATE_CLR_REQ = 1;
  if( v->state.VOICE_ACTIVE )
    v->state.GATE_SET_REQ = 1;

  // check if voice should be delayed - set delay counter to 0x0001 in this case, else to 0x0000
  v->set_delay_ctr = v->voice_patch->delay ? 0x0001 : 0x0000;

  // delay also if ABW (ADSR bug workaround) option is active
  sid_se_opt_flags_t opt_flags = (sid_se_opt_flags_t)sid_patch[v->sid].opt_flags;
  if( opt_flags.ABW ) {
    v->set_delay_ctr = 0x0001;
    // clear ADSR registers, so that the envelope gets completely released
    v->phys_sid_voice->ad = 0x00;
    v->phys_sid_voice->sr = 0x00;
  }

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Voice Pitch
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_Pitch(sid_se_voice_t *v)
{
  // transpose MIDI note
  sid_se_voice_arp_mode_t arp_mode = (sid_se_voice_arp_mode_t)v->voice_patch->arp_mode;
  int transposed_note = arp_mode.ENABLE ? v->arp_note : v->note;

  transposed_note += (int)v->voice_patch->transpose - 64;
  transposed_note += (int)v->transpose - 64;

  // octave-wise saturation
  while( transposed_note < 0 )
    transposed_note += 12;
  while( transposed_note >= 128 )
    transposed_note -= 12;

  // Glissando handling
  sid_se_voice_flags_t voice_flags = (sid_se_voice_flags_t)v->voice_patch->flags;
  if( transposed_note != v->prev_transposed_note ) {
    v->prev_transposed_note = transposed_note;
    if( voice_flags.PORTA_MODE >= 2 ) // Glissando active?
      v->portamento_ctr = 0xffff; // force overrun
  }

  u8 portamento = v->voice_patch->portamento;
  if( v->state.PORTA_ACTIVE && portamento && voice_flags.PORTA_MODE >= 2 ) {
    // increment portamento counter, amount derived from envelope table  .. make it faster
    int portamento_ctr = v->portamento_ctr + sid_se_env_table[portamento >> 1] / sid_se_speed_factor;
    // next note step?
    if( portamento_ctr >= 0xffff ) {
      // reset counter
      v->portamento_ctr = 0;

      // increment/decrement note
      if( transposed_note > v->glissando_note )
	++v->glissando_note;
      else if( transposed_note < v->glissando_note )
	--v->glissando_note;

      // target reached?
      if( transposed_note == v->glissando_note )
	v->state.PORTA_ACTIVE = 0;
    } else {
      // take over new counter value
      v->portamento_ctr = portamento_ctr;
    }
    // switch to current glissando note
    transposed_note = v->glissando_note;
  } else {
    // store current transposed note (optionally varried by glissando effect)
    v->glissando_note = transposed_note;
  }

  // transfer note -> linear frequency
  int target_frq = transposed_note << 9;

  // increase/decrease target frequency by pitchrange
  // depending on pitchbender and finetune value
  if( v->voice_patch->pitchrange ) {
    u16 pitchbender = v->pitchbender; // TODO: multi/bassline/drum engine take pitchbender from MIDI Voice
    int delta = (int)pitchbender - 0x80;
    delta += (int)v->voice_patch->finetune-0x80;

    // detuning
    u8 detune = sid_patch[v->sid].L.osc_detune;
    if( detune ) {
      // additional detuning depending on SID channel and oscillator
      // Left OSC1: +detune/4   (lead only, 0 in bassline mode)
      // Right OSC1: -detune/4  (lead only, 0 in bassline mode)
      // Left OSC2: +detune
      // Right OSC2: -detune
      // Left OSC3: -detune
      // Right OSC3: +detune
      switch( v->voice ) {
        case 0: delta += detune/4; break;
        case 3: delta -= detune/4; break;

        case 1:
        case 5: delta += detune; break;

        case 2:
        case 4: delta -= detune; break;
      }
    }

    if( delta ) {
      int scaled = (delta * 4 * (int)v->voice_patch->pitchrange);
      target_frq += scaled;
    }
  }

  // saturate target frequency to 16bit
  if( target_frq < 0 ) target_frq = 0; else if( target_frq > 0xffff ) target_frq = 0xffff;

  // portamento
  // whenever target frequency has been changed, update portamento frequency
  if( v->portamento_end_frq != target_frq ) {
    v->portamento_end_frq = target_frq;
    v->portamento_begin_frq = v->linear_frq;
    // reset portamento counter if not in glissando mode
    if( voice_flags.PORTA_MODE < 2 )
      v->portamento_ctr = 0;

    if( target_frq == v->linear_frq )
      v->state.PORTA_ACTIVE = 0; // nothing to do...
  }

  int linear_frq = target_frq;
  if( v->state.PORTA_ACTIVE && portamento && voice_flags.PORTA_MODE < 2 ) {
    linear_frq = v->linear_frq;

    // get portamento multiplier from envelope table
    // this one is used for "constant time glide" and "normal portamento"
    int porta_multiplier = sid_se_env_table[portamento] / sid_se_speed_factor;

    if( voice_flags.PORTA_MODE >= 1 ) { // constant glide time and glissando
      // increment counter
      int portamento_ctr = v->portamento_ctr + porta_multiplier;

      // target reached on overrun
      if( portamento_ctr >= 0xffff ) {
	linear_frq = target_frq;
	v->state.PORTA_ACTIVE = 0;
      } else {
	v->portamento_ctr = portamento_ctr;

	// scale between new and old frequency
	int delta = v->portamento_end_frq - v->portamento_begin_frq;
	linear_frq = v->portamento_begin_frq + ((delta * portamento_ctr) >> 16);
	if( delta > 0 ) {
	  if( linear_frq >= target_frq ) {
	    linear_frq = target_frq;
	    v->state.PORTA_ACTIVE = 0;
	  }
	} else {
	  if( linear_frq <= target_frq ) {
	    linear_frq = target_frq;
	    v->state.PORTA_ACTIVE = 0;
	  }
	}
      }
    } else { // normal portamento

      // increment/decrement frequency
      int inc = (int)(((u32)linear_frq * (u32)porta_multiplier) >> 16);
      if( !inc )
	inc = 1;

      if( target_frq > linear_frq ) {
	linear_frq += inc;
	if( linear_frq >= target_frq ) {
	  linear_frq = target_frq;
	  v->state.PORTA_ACTIVE = 0;
	}
      } else {
	linear_frq -= inc;
	if( linear_frq <= target_frq ) {
	  linear_frq = target_frq;
	  v->state.PORTA_ACTIVE = 0;
	}
      }
    }
  }

  // pitch modulation
  linear_frq += *v->mod_dst_pitch;
  if( linear_frq < 0 ) linear_frq = 0; else if( linear_frq > 0xffff ) linear_frq = 0xffff;

  // if frequency has been changed
  if( v->linear_frq != linear_frq ) {
    v->linear_frq = linear_frq;

    // linear frequency -> SID frequency conversion
    // convert bitfield [15:9] of 16bit linear frequency to SID frequency value
    u8 frq_ix = (linear_frq >> 9) + 21;
    if( frq_ix > 126 )
      frq_ix = 126;
    // interpolate between two frequency steps (frq_a and frq_b)
    int frq_a = sid_se_frq_table[frq_ix];
    int frq_b = sid_se_frq_table[frq_ix+1];

    // use bitfield [8:0] of 16bit linear frequency for scaling between two semitones
    int frq_semi = (frq_b-frq_a) * (linear_frq & 0x1ff);
    int frq = frq_a + (frq_semi >> 9);

    // write result into SID frequency register
    v->phys_sid_voice->frq_l = frq & 0xff;
    v->phys_sid_voice->frq_h = frq >> 8;
  }
  
  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Voice Pulsewidth
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_PW(sid_se_voice_t *v)
{
  // convert pulsewidth to 12bit signed value
  int pulsewidth = ((v->voice_patch->pulsewidth_h & 0x0f) << 8) | v->voice_patch->pulsewidth_l;

  // PW modulation
  pulsewidth += *v->mod_dst_pw / 16;
  if( pulsewidth > 0xfff ) pulsewidth = 0xfff; else if( pulsewidth < 0 ) pulsewidth = 0;

  // transfer to SID registers
  v->phys_sid_voice->pw_l = pulsewidth & 0xff;
  v->phys_sid_voice->pw_h = (pulsewidth >> 8) & 0x0f;

  return 0; // no error
}



/////////////////////////////////////////////////////////////////////////////
// SID Filter and Volume
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_FilterAndVolume(sid_se_filter_t *f)
{
  int cutoff = ((f->filter_patch->cutoff_h & 0x0f) << 8) | f->filter_patch->cutoff_l;

  // cutoff modulation (/8 instead of /16 to simplify extreme modulation results)
  cutoff += *f->mod_dst_filter / 8;
  if( cutoff > 0xfff ) cutoff = 0xfff; else if( cutoff < 0 ) cutoff = 0;

  // calibration
  // TODO: take calibration values from ensemble
  u16 cali_min = 0;
  u16 cali_max = 1536;
  cutoff = cali_min + ((cutoff * (cali_max-cali_min)) / 4096);

  // map 12bit value to 11 value of SID register
  f->phys_sid_regs->filter_l = (cutoff >> 1) & 0x7;
  f->phys_sid_regs->filter_h = (cutoff >> 4);

  // resonance (4bit only)
  int resonance = f->filter_patch->resonance;
  f->phys_sid_regs->resonance = resonance >> 4;

  // filter channel/mode selection
  u8 chn_mode = f->filter_patch->chn_mode;
  f->phys_sid_regs->filter_select = chn_mode & 0xf;
  f->phys_sid_regs->filter_mode = chn_mode >> 4;

  // volume
  int volume = sid_patch[f->sid].L.volume << 9;

  // volume modulation
  volume += *f->mod_dst_volume;
  if( volume > 0xffff ) volume = 0xffff; else if( volume < 0 ) volume = 0;

  f->phys_sid_regs->volume = volume >> 12;

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// LFO
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_LFO(sid_se_lfo_t *l)
{
  sid_se_lfo_mode_t lfo_mode = (sid_se_lfo_mode_t)l->lfo_patch->mode;

  // set wave register to initial value and skip LFO if not enabled
  if( !lfo_mode.ENABLE ) {
    *l->mod_src_lfo = 0;
  } else {
    if( l->delay_ctr ) {
      int new_delay_ctr = l->delay_ctr + (sid_se_env_table[l->lfo_patch->delay] / sid_se_speed_factor);
      if( new_delay_ctr > 0xffff )
	l->delay_ctr = 0; // delay passed
      else
	l->delay_ctr = new_delay_ctr; // delay not passed
    }

    if( !l->delay_ctr ) { // delay passed?
      u8 lfo_overrun = 0;
      u8 lfo_stalled = 0;

      // in oneshot mode: check if overrun already occured
      if( lfo_mode.ONESHOT ) {
	if( sid_se_vars[l->sid].lfo_overrun & (1 << l->lfo) ) {
	  // set counter to maximum value and skip sweep
	  l->ctr = 0xffff;
	  lfo_stalled = 1;
	}
      }

      // if clock sync enabled: only increment on each 16th clock event
      if( lfo_mode.CLKSYNC && (!sid_se_clk.event.CLK || sid_se_clk.global_clk_ctr6 != 0) )
	lfo_stalled = 1;

      if( !lfo_stalled ) {
	// increment 16bit counter by given rate
	// the rate can be modulated
	s32 lfo_rate = l->lfo_patch->rate + (*l->mod_dst_lfo_rate / 256);
	if( lfo_rate > 255 ) lfo_rate = 255; else if( lfo_rate < 0 ) lfo_rate = 0;

	// if LFO synched via clock, replace 245-255 by MIDI clock optimized incrementers
	u16 inc;
	if( lfo_mode.CLKSYNC && lfo_rate >= 245 )
	  inc = sid_se_lfo_table_mclk[lfo_rate-245]; // / sid_se_speed_factor;
	else
	  inc = sid_se_lfo_table[lfo_rate] / sid_se_speed_factor;

	// add to counter and check for overrun
	s32 new_ctr = l->ctr + inc;
	if( new_ctr > 0xffff ) {
	  lfo_overrun = 1;

	  if( lfo_mode.ONESHOT )
	    new_ctr = 0xffff; // stop at end position

	  // propagate overrun to trigger matrix
	  l->trg_dst[0] |= l->trg_mask_lfo_period[0];
	  l->trg_dst[1] |= l->trg_mask_lfo_period[1];
	  l->trg_dst[2] |= l->trg_mask_lfo_period[2];

	  // required for oneshot mode
	  sid_se_vars[l->sid].lfo_overrun |= (1 << l->lfo);
	}
	l->ctr = (u16)new_ctr;
      }

      SID_SE_LFO_GenWave(l, lfo_overrun);
    }
  }

  return 0; // no error
}


s32 SID_SE_LFO_GenWave(sid_se_lfo_t *l, u8 lfo_overrun)
{
  sid_se_lfo_mode_t lfo_mode = (sid_se_lfo_mode_t)l->lfo_patch->mode;

  // map counter to waveform
  u8 lfo_waveform_skipped = 0;
  s16 wave;
  switch( lfo_mode.WAVEFORM ) {
    case 0: { // Sine
      // sine table contains a quarter of a sine
      // we have to negate/mirror it depending on the mapped counter value
      u8 ptr = l->ctr >> 7;
      if( l->ctr & (1 << 14) )
	ptr ^= 0x7f;
      ptr &= 0x7f;
      wave = sid_se_sin_table[ptr];
      if( l->ctr & (1 << 15) )
	wave = -wave;
    } break;  

    case 1: { // Triangle
      // similar to sine, but linear waveform
      wave = (l->ctr & 0x3fff) << 1;
      if( l->ctr & (1 << 14) )
	wave = 0x7fff - wave;
      if( l->ctr & (1 << 15) )
	wave = -wave;
    } break;  

    case 2: { // Saw
      wave = l->ctr - 0x8000;
    } break;  

    case 3: { // Pulse
      wave = (l->ctr < 0x8000) ? -0x8000 : 0x7fff; // due to historical reasons it's inverted
    } break;  

    case 4: { // Random
      // only on LFO overrun
      if( lfo_overrun )
	wave = SID_RANDOM_Gen_Range(0x0000, 0xffff);
      else
	lfo_waveform_skipped = 1;
    } break;  

    case 5: { // Positive Sine
      // sine table contains a quarter of a sine
      // we have to negate/mirror it depending on the mapped counter value
      u8 ptr = l->ctr >> 8;
      if( l->ctr & (1 << 15) )
	ptr ^= 0x7f;
      ptr &= 0x7f;
      wave = sid_se_sin_table[ptr];
    } break;  

    case 6: { // Positive Triangle
      // similar to sine, but linear waveform
      wave = (l->ctr & 0x7fff);
      if( l->ctr & (1 << 15) )
	wave = 0x7fff - wave;
    } break;  

    case 7: { // Positive Saw
      wave = l->ctr >> 1;
    } break;  

    case 8: { // Positive Pulse
      wave = (l->ctr < 0x8000) ? 0 : 0x7fff; // due to historical reasons it's inverted
    } break;  

    default: // take saw as default
      wave = l->ctr - 0x8000;
  }

  if( !lfo_waveform_skipped ) {
    // scale to LFO depth
    // the depth can be modulated
    s32 lfo_depth = ((s32)l->lfo_patch->depth - 0x80) + (*l->mod_dst_lfo_depth / 256);
    if( lfo_depth > 127 ) lfo_depth = 127; else if( lfo_depth < -128 ) lfo_depth = -128;
    
    // final LFO value
    *l->mod_src_lfo = (wave * lfo_depth) / 128;
  }

  return 0; // no error
}


s32 SID_SE_LFO_Restart(sid_se_lfo_t *l)
{
  // reset counter (take phase into account)
  l->ctr = l->lfo_patch->phase << 8;

  // clear overrun flag (for oneshot mode)
  sid_se_vars[l->sid].lfo_overrun &= ~(1 << l->lfo);

  // check if LFO should be delayed - set delay counter to 0x0001 in this case
  l->delay_ctr = l->lfo_patch->delay ? 1 : 0;

  // re-calculate waveform
  return SID_SE_LFO_GenWave(l, 0);
}


/////////////////////////////////////////////////////////////////////////////
// ENV
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_ENV(sid_se_env_t *e)
{
  // if clock sync enabled: only increment on each 16th clock event
  if( (e->env_patch->mode & (1 << 7)) &&
      (!sid_se_clk.event.CLK || sid_se_clk.global_clk_ctr6 != 0) ) {
    if( e->state == SID_SE_ENV_STATE_IDLE )
      return 0; // nothing to do
  } else {
    switch( e->state ) {
      case SID_SE_ENV_STATE_ATTACK1:
        if( e->delay_ctr ) {
  	int new_delay_ctr = e->delay_ctr + (sid_se_env_table[e->env_patch->delay] / sid_se_speed_factor);
  	if( new_delay_ctr > 0xffff )
  	  e->delay_ctr = 0; // delay passed
  	else {
  	  e->delay_ctr = new_delay_ctr; // delay not passed
  	  return 0; // no error
  	}
        }
  
        if( SID_SE_ENV_Step(&e->ctr, e->env_patch->attlvl << 8, e->env_patch->attack1, e->env_patch->att_curve) )
  	e->state = SID_SE_ENV_STATE_ATTACK2; // TODO: Set Phase depending on e->mode
        break;
  
      case SID_SE_ENV_STATE_ATTACK2:
        if( SID_SE_ENV_Step(&e->ctr, 0xffff, e->env_patch->attack2, e->env_patch->att_curve) )
  	e->state = SID_SE_ENV_STATE_DECAY1; // TODO: Set Phase depending on e->mode
        break;
  
      case SID_SE_ENV_STATE_DECAY1:
        if( SID_SE_ENV_Step(&e->ctr, e->env_patch->declvl << 8, e->env_patch->decay1, e->env_patch->dec_curve) )
  	e->state = SID_SE_ENV_STATE_DECAY2; // TODO: Set Phase depending on e->mode
        break;
  
      case SID_SE_ENV_STATE_DECAY2:
        if( SID_SE_ENV_Step(&e->ctr, e->env_patch->sustain << 8, e->env_patch->decay2, e->env_patch->dec_curve) ) {
  	e->state = SID_SE_ENV_STATE_SUSTAIN; // TODO: Set Phase depending on e->mode
  
  	// propagate sustain phase to trigger matrix
  	e->trg_dst[0] |= e->trg_mask_env_sustain[0];
  	e->trg_dst[1] |= e->trg_mask_env_sustain[1];
  	e->trg_dst[2] |= e->trg_mask_env_sustain[2];
        }
        break;
  
      case SID_SE_ENV_STATE_SUSTAIN:
        // always update sustain level
        e->ctr = e->env_patch->sustain << 8;
        break;
  
      case SID_SE_ENV_STATE_RELEASE1:
        if( SID_SE_ENV_Step(&e->ctr, e->env_patch->rellvl << 8, e->env_patch->release1, e->env_patch->rel_curve) )
  	e->state = SID_SE_ENV_STATE_RELEASE2; // TODO: Set Phase depending on e->mode
        break;
  
      case SID_SE_ENV_STATE_RELEASE2:
        if( e->ctr )
  	SID_SE_ENV_Step(&e->ctr, 0x0000, e->env_patch->release2, e->env_patch->rel_curve);
        break;
  
      default: // like SID_SE_ENV_STATE_IDLE
        return 0; // nothing to do...
    }
  }

  // scale to ENV depth
  s32 env_depth = (s32)e->env_patch->depth - 0x80;
    
  // final ENV value (range +/- 0x7fff)
  *e->mod_src_env = ((e->ctr/2) * env_depth) / 128;

  return 0; // no error
}


s32 SID_SE_ENV_Step(u16 *ctr, u16 target, u8 rate, u8 curve)
{
  if( target == *ctr )
    return 1; // next state

  // modify rate if curve != 0x80
  u16 inc_rate;
  if( curve != 0x80 ) {
    // this nice trick has been proposed by Razmo
    int abs_curve = curve - 0x80;
    if( abs_curve < 0 )
      abs_curve = -abs_curve;
    else
      abs_curve ^= 0x7f; // invert if positive range for more logical behaviour of positive/negative curve

    int rate_msbs = (rate >> 1); // TODO: we could increase resolution by using an enhanced frq_table
    int feedback = (abs_curve * (*ctr>>8)) >> 8; 
    int ix;
    if( curve > 0x80 ) { // bend up
      ix = (rate_msbs ^ 0x7f) - feedback;
      if( ix < 0 )
	ix = 0;
    } else { // bend down
      ix = (rate_msbs ^ 0x7f) + feedback;
      if( ix >= 127 )
	ix = 127;
    }
    inc_rate = sid_se_frq_table[ix];
  } else {
    inc_rate = sid_se_env_table[rate];
  }

  // positive or negative direction?
  if( target > *ctr ) {
    s32 new_ctr = (s32)*ctr + (inc_rate / sid_se_speed_factor);
    if( new_ctr >= target ) {
      *ctr = target;
      return 1; // next state
    }
    *ctr = new_ctr;
    return 0; // stay in state
  }

  s32 new_ctr = (s32)*ctr - (inc_rate / sid_se_speed_factor);
  if( new_ctr <= target ) {
    *ctr = target;
    return 1; // next state
  }
  *ctr = new_ctr;

  return 0; // stay in state
}


s32 SID_SE_ENV_Restart(sid_se_env_t *e)
{
  // start at attack phase
  e->state = SID_SE_ENV_STATE_ATTACK1;

  // check if ENV should be delayed - set delay counter to 0x0001 in this case
  e->delay_ctr = e->env_patch->delay ? 1 : 0;

  return 0; // no error
}


s32 SID_SE_ENV_Release(sid_se_env_t *e)
{
  // set release phase
  e->state = SID_SE_ENV_STATE_RELEASE1;

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Wavetable
/////////////////////////////////////////////////////////////////////////////
s32 SID_SE_WT(sid_se_wt_t *w)
{
  s32 step = -1;

  // if key control flag (END[7]) set, control position from current played key
  if( w->wt_patch->end & (1 << 7) ) {
    // copy currently played note to step position
    step = sid_se_voice[w->sid][0].played_note;

  // if MOD control flag (BEGIN[7]) set, control step position from modulation matrix
  } else if( w->wt_patch->begin & (1 << 7) ) {
    step = ((s32)*w->mod_dst_wt + 0x8000) >> 9; // 16bit signed -> 7bit unsigned
  }

  if( step >= 0 ) {
    // use modulated step position
    // scale between begin/end range
    if( w->wt_patch->end > w->wt_patch->begin ) {
      s32 range = (w->wt_patch->end - w->wt_patch->begin) + 1;
      step = w->wt_patch->begin + ((step * range) / 128);
    } else {
      // should we invert the waveform?
      s32 range = (w->wt_patch->begin - w->wt_patch->end) + 1;
      step = w->wt_patch->end + ((step * range) / 128);
    }
  } else {
    // don't use modulated position - normal mode
    u8 next_step_req = 0;

    // check if WT reset requested
    if( w->trg_src[2] & (1 << w->wt) ) {
      // next clock will increment div to 0
      w->div_ctr = 0xff;
      // next step will increment to start position
      w->pos = (w->wt_patch->begin & 0x7f) - 1;
    }

    // check for WT step event
    if( w->trg_src[2] & (1 << (4+w->wt)) ) {
      // increment clock divider
      // reset divider if it already has reached the target value
      if( ++w->div_ctr == 0 || (w->div_ctr > (w->wt_patch->speed & 0x3f)) ) {
	w->div_ctr = 0;
	next_step_req = 1;
      }
    }

    // check for next step request
    // skip if position is 0xaa (notifies oneshot -> WT stopped)
    if( next_step_req && w->pos != 0xaa ) {
      // increment position counter, reset at end position
      if( ++w->pos > w->wt_patch->end ) {
	// if oneshot mode: set position to 0xaa, WT is stopped now!
	if( w->wt_patch->loop & (1 << 7) )
	  w->pos = 0xaa;
	else
	  w->pos = w->wt_patch->loop & 0x7f;
      }
      step = w->pos; // step is positive now -> will be played
    }
  }

  // check if step should be played
  if( step >= 0 ) {
    u8 wt_value = sid_patch[w->sid].L.wt_memory[step & 0x7f];

    // forward to mod matrix
    if( wt_value < 0x80 ) {
      // relative value -> convert to -0x8000..+0x7fff
      *w->mod_src_wt = ((s32)wt_value - 0x40) * 512;
    } else {
      // absolute value -> convert to 0x0000..+0x7f00
      *w->mod_src_wt = ((s32)wt_value & 0x7f) * 256;
    }
    
    // determine SID channels which should be modified
    u8 sidlr = (w->wt_patch->speed >> 6); // SID selection
    u8 ins = w->wt; // preparation for multi engine

    // call parameter handler
    if( w->wt_patch->assign ) {
#if DEBUG_VERBOSE_LEVEL >= 2
      DEBUG_MSG("WT %d: 0x%02x 0x%02x\n", w->wt, step, wt_value);
#endif
      SID_PAR_SetWT(w->sid, w->wt_patch->assign, wt_value, sidlr, ins);
    }
  }

  return 0; // no error
}
