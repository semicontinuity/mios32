// $Id$
/*
 * Utility page
 *
 * ==========================================================================
 *
 *  Copyright (C) 2008 Thorsten Klose (tk@midibox.org)
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
#include "seq_lcd.h"
#include "seq_ui.h"

#include "seq_core.h"
#include "seq_layer.h"
#include "seq_par.h"
#include "seq_trg.h"
#include "seq_cc.h"
#include "seq_live.h"


/////////////////////////////////////////////////////////////////////////////
// Local definitions
/////////////////////////////////////////////////////////////////////////////

// used "In-Menu" messages
#define MSG_DEFAULT 0x00
#define MSG_COPY    0x81
#define MSG_PASTE   0x82
#define MSG_CLEAR   0x83
#define MSG_MOVE    0x84
#define MSG_SCROLL  0x85
#define MSG_UNDO    0x86


// name the two buffers of the move function
#define MOVE_BUFFER_NEW 0
#define MOVE_BUFFER_OLD 1


// saves some memory for LPC17 (tmp. check)
#if defined(MIOS32_FAMILY_LPC17xx)
# define UNDO_ENABLED 0
#else
# define UNDO_ENABLED 1
#endif


typedef enum {
  PASTE_CLEAR_MODE_SELECT = 0,
  PASTE_CLEAR_MODE_TRACK,
  PASTE_CLEAR_MODE_PAR_LAYER,
  PASTE_CLEAR_MODE_TRG_LAYER,
  PASTE_CLEAR_MODE_INS_LAYER,
} paste_clear_mode_t;

/////////////////////////////////////////////////////////////////////////////
// Local variables
/////////////////////////////////////////////////////////////////////////////

static u8 in_menu_msg;

static const char in_menu_msg_str[6][9] = {
  ">COPIED<",	// #1
  ">PASTED<",	// #2
  "CLEARED!",	// #3
  ">>MOVE<<",	// #4
  ">SCROLL<",	// #5
  ">>UNDO<<"	// #6
};

static u8 copypaste_begin;
static u8 copypaste_end;

static u8 copypaste_buffer_filled = 0;
static u8 copypaste_track = 0;
static u8 copypaste_par_layer[SEQ_PAR_MAX_BYTES];
static u8 copypaste_trg_layer[SEQ_TRG_MAX_BYTES];
static u8 copypaste_cc[128];
static u8 copypaste_trk_name[81];
static u8 copypaste_par_layers;
static u16 copypaste_par_steps;
static u8 copypaste_trg_layers;
static u16 copypaste_trg_steps;
static u8 copypaste_num_instruments;
static u8 copypaste_selected_par_layer;
static u8 copypaste_selected_trg_layer;
static u8 copypaste_selected_instrument;

#if UNDO_ENABLED
static u8 undo_buffer_filled = 0;
static u8 undo_track = 0;
static u8 undo_par_layer[SEQ_PAR_MAX_BYTES];
static u8 undo_trg_layer[SEQ_TRG_MAX_BYTES];
static u8 undo_cc[128];
static u8 undo_trk_name[81];
static u8 undo_par_layers;
static u16 undo_par_steps;
static u8 undo_trg_layers;
static u16 undo_trg_steps;
static u8 undo_num_instruments;
#endif

static s8 move_enc;
static u8 move_par_layer[2][16];
static u16 move_trg_layer[2];


/////////////////////////////////////////////////////////////////////////////
// Local Prototypes
/////////////////////////////////////////////////////////////////////////////
static s32 COPY_Track(u8 track);
static s32 PASTE_Track(u8 track, paste_clear_mode_t paste_clear_mode);
static s32 CLEAR_Track(u8 track, paste_clear_mode_t paste_clear_mode);
static s32 UNDO_Track(void);

static s32 MOVE_StoreStep(u8 track, u16 step, u8 buffer, u8 clr_triggers);
static s32 MOVE_RestoreStep(u8 track, u16 step, u8 buffer);

static s32 SCROLL_Track(u8 track, u16 first_step, s32 incrementer);

static s32 SEQ_UI_UTIL_MuteAllTracks(void);
static s32 SEQ_UI_UTIL_UnMuteAllTracks(void);


/////////////////////////////////////////////////////////////////////////////
// Local LED handler function
/////////////////////////////////////////////////////////////////////////////
static s32 LED_Handler(u16 *gp_leds)
{
  // branch to edit page if copy/paste/move/scroll button is pressed
  switch( in_menu_msg ) {
    case MSG_COPY:
    case MSG_PASTE:
    case MSG_MOVE:
    case MSG_SCROLL:
      return SEQ_UI_EDIT_LED_Handler(gp_leds);
  }

  if( ui_cursor_flash ) // if flashing flag active: no LED flag set
    return 0;

 *gp_leds = 0x0001;

  return 0; // no error
}



/////////////////////////////////////////////////////////////////////////////
// Local encoder callback function
// Should return:
//   1 if value has been changed
//   0 if value hasn't been changed
//  -1 if invalid or unsupported encoder
/////////////////////////////////////////////////////////////////////////////
static s32 Encoder_Handler(seq_ui_encoder_t encoder, s32 incrementer)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();
  u8 encoder_step = (u8)encoder + 16*ui_selected_step_view;

  // branch depending on in-menu message
  switch( in_menu_msg ) {
    case MSG_COPY: {
      // change copy offset and copy marker to begin/end range depending on encoder which has been moved
      if( encoder == SEQ_UI_ENCODER_GP1 || encoder_step <= copypaste_begin ) {
	if( SEQ_UI_Var8_Inc(&copypaste_begin, 0, copypaste_end, incrementer) ) {
	  SEQ_UI_SelectedStepSet(copypaste_begin); // set new visible step/view
	  return 1; // value changed
	}
      } else {
	int num_steps = SEQ_TRG_NumStepsGet(visible_track);
	if( SEQ_UI_Var8_Inc(&copypaste_end, copypaste_begin, num_steps-1, incrementer) ) {
	  SEQ_UI_SelectedStepSet(copypaste_end); // set new visible step/view
	  return 1; // value changed
	}
      }
      return 0; // no value changed
    } break;

    case MSG_PASTE: {
      // change paste offset
      int num_steps = SEQ_TRG_NumStepsGet(visible_track);
      if( SEQ_UI_Var8_Inc(&ui_selected_step, 0, num_steps-1, incrementer) ) {
	SEQ_UI_SelectedStepSet(ui_selected_step); // set new visible step/view
	return 1; // value changed
      }
      return 0; // no value changed
    } break;

    case MSG_MOVE: {
      // if encoder number is different from move_enc, get step, otherwise move step

      if( move_enc != encoder ) { // first selection or new encoder is moved

	// select step
	SEQ_UI_SelectedStepSet(encoder_step); // this will change ui_selected_step

	// select new encoder
	move_enc = encoder;
	// store current step value in buffer
	MOVE_StoreStep(visible_track, ui_selected_step, MOVE_BUFFER_NEW, 0);
	// store it also in "old" record and disable current value (clear all triggers)
	MOVE_StoreStep(visible_track, ui_selected_step, MOVE_BUFFER_OLD, 1);

      } else {

	// increment step -> this will move it
	u16 new_step = ui_selected_step;
	int num_steps = SEQ_TRG_NumStepsGet(visible_track);
	if( SEQ_UI_Var16_Inc(&new_step, 0, num_steps-1, incrementer) ) {
	  // restore old value
	  MOVE_RestoreStep(visible_track, ui_selected_step, MOVE_BUFFER_OLD);
	  // set new visible step/view
	  SEQ_UI_SelectedStepSet(new_step); // this will change ui_selected_step
	  // store "new" old value w/o disabling triggers
	  MOVE_StoreStep(visible_track, new_step, MOVE_BUFFER_OLD, 0);
	  // restore moved value in new step step
	  MOVE_RestoreStep(visible_track, new_step, MOVE_BUFFER_NEW);
	}
      }
      return 1; // value changed
    } break;

    case MSG_SCROLL: {
      // select step
      SEQ_UI_SelectedStepSet(encoder_step); // this will change ui_selected_step
      // call scroll handler
      SCROLL_Track(visible_track, ui_selected_step, incrementer);
      return 1; // value changed
    } break;

    default:
      if( encoder == SEQ_UI_ENCODER_GP1 )
	return SEQ_UI_GxTyInc(incrementer);
  }

  return -1; // invalid or unsupported encoder
}


/////////////////////////////////////////////////////////////////////////////
// Local button callback function
// Should return:
//   1 if value has been changed
//   0 if value hasn't been changed
//  -1 if invalid or unsupported button
/////////////////////////////////////////////////////////////////////////////
static s32 Button_Handler(seq_ui_button_t button, s32 depressed)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();

  switch( button ) {
    case SEQ_UI_BUTTON_GP1: // GxTy
      if( depressed ) return 0; // ignore when button depressed
      return 1; // nothing to do for button

    case SEQ_UI_BUTTON_GP2: // Copy
      if( depressed ) {
	if( in_menu_msg != MSG_COPY )
	  return 0; // ignore if no copy message
	// turn message inactive and hold it for 1 second
	in_menu_msg &= 0x7f;
	ui_hold_msg_ctr = 1000;
	// copy steps
	COPY_Track(visible_track);
      } else {
	if( in_menu_msg & 0x80 )
	  return 0; // ignore as long as other message is displayed

	// print message
	in_menu_msg = MSG_COPY;
	// select first step in section
	u8 track_length = SEQ_CC_Get(visible_track, SEQ_CC_LENGTH);
	u8 first_step = ui_selected_step - (ui_selected_step % ((int)track_length + 1));
	SEQ_UI_SelectedStepSet(first_step);
	// select full range
	copypaste_begin = first_step;
	copypaste_end = first_step + track_length;
      }
      return 1;

    case SEQ_UI_BUTTON_GP3: // Paste
      // duplicate steps if COPY button pressed as well
      if( in_menu_msg == MSG_COPY ) {
	if( !depressed ) {
	  u8 visible_track = SEQ_UI_VisibleTrackGet();
	  if( SEQ_UI_UTIL_PasteDuplicateSteps(visible_track) >= 1 ) {
	    SEQ_UI_Msg_Track("steps duplicated");
	  } else {
	    SEQ_UI_Msg_Track("full - no duplication!");
	  }
	}
	return 1;
      }

      if( depressed ) {
	if( in_menu_msg != MSG_PASTE )
	  return 0; // ignore if no paste message
	// turn message inactive and hold it for 1 second
	in_menu_msg &= 0x7f;
	ui_hold_msg_ctr = 1000;
	// paste steps
	PASTE_Track(visible_track, PASTE_CLEAR_MODE_SELECT);
      } else {
	if( in_menu_msg & 0x80 )
	  return 0; // ignore as long as other message is displayed

	// update undo buffer
	SEQ_UI_UTIL_UndoUpdate(visible_track);
	// print message
	in_menu_msg = MSG_PASTE;
	// select first step
	SEQ_UI_SelectedStepSet(16 * ui_selected_step_view);
      }
      return 1;

    case SEQ_UI_BUTTON_GP4: // Clear
      if( depressed ) {
	if( in_menu_msg != MSG_CLEAR )
	  return 0; // ignore if no clear message
	// turn message inactive and hold it for 1 second
	in_menu_msg &= 0x7f;
	ui_hold_msg_ctr = 1000;
      } else {
	if( in_menu_msg & 0x80 )
	  return 0; // ignore as long as other message is displayed

	// update undo buffer
	SEQ_UI_UTIL_UndoUpdate(visible_track);

	// clear steps of all selected tracks
	u8 track;
	for(track=0; track<SEQ_CORE_NUM_TRACKS; ++track) {
	  if( ui_selected_tracks & (1 << track) )
	    CLEAR_Track(track, PASTE_CLEAR_MODE_SELECT);
	}

	// print message
	in_menu_msg = MSG_CLEAR;
      }
      return 1;

    case SEQ_UI_BUTTON_GP5: // Move
      move_enc = -1; // disable move encoder
      if( depressed ) {
	if( in_menu_msg != MSG_MOVE )
	  return 0; // ignore if no move message
	// turn message inactive and hold it for 0.5 second
	in_menu_msg &= 0x7f;
	ui_hold_msg_ctr = 500;
      } else {
	if( in_menu_msg & 0x80 )
	  return 0; // ignore as long as other message is displayed

	// update undo buffer
	SEQ_UI_UTIL_UndoUpdate(visible_track);
	// print message
	in_menu_msg = MSG_MOVE;
      }
      return 1;

    case SEQ_UI_BUTTON_GP6: // Scroll
      if( depressed ) {
	if( in_menu_msg != MSG_SCROLL )
	  return 0; // ignore if no scroll message
	// turn message inactive and hold it for 0.5 second
	in_menu_msg &= 0x7f;
	ui_hold_msg_ctr = 500;
      } else {
	if( in_menu_msg & 0x80 )
	  return 0; // ignore as long as other message is displayed

	// update undo buffer
	SEQ_UI_UTIL_UndoUpdate(visible_track);
	// print message
	in_menu_msg = MSG_SCROLL;
	// select first step
	SEQ_UI_SelectedStepSet(16 * ui_selected_step_view);
      }
      return 1;

    case SEQ_UI_BUTTON_GP7: // select Random Page
      if( depressed ) return -1;

      if( in_menu_msg & 0x80 )
	return 0; // ignore as long as other message is displayed

      SEQ_UI_PageSet(SEQ_UI_PAGE_TRKRND);
      return 0;
      
    case SEQ_UI_BUTTON_GP8: // Undo
      if( depressed ) {
	// turn message inactive and hold it for 1 second
	if( in_menu_msg != MSG_UNDO )
	  return 0; // ignore if no undo message
	in_menu_msg &= 0x7f;
	ui_hold_msg_ctr = 1000;
      } else {
	if( in_menu_msg & 0x80 )
	  return 0; // ignore as long as other message is displayed

	// undo last change
	UNDO_Track();
	// print message
	in_menu_msg = MSG_UNDO;
      }
      return 1;

    case SEQ_UI_BUTTON_GP9: // select Save Page
      if( depressed ) return -1;
      if( in_menu_msg & 0x80 )
	return 0; // ignore as long as message is displayed
      return SEQ_UI_PageSet(SEQ_UI_PAGE_SAVE);

    case SEQ_UI_BUTTON_GP10: // select Jam Page
      if( depressed ) return -1;
      if( in_menu_msg & 0x80 )
	return 0; // ignore as long as message is displayed
      return SEQ_UI_PageSet(SEQ_UI_PAGE_TRKJAM);

    case SEQ_UI_BUTTON_GP11: // select CV Page
      if( depressed ) return -1;
      if( in_menu_msg & 0x80 )
	return 0; // ignore as long as message is displayed
      return SEQ_UI_PageSet(SEQ_UI_PAGE_CV);

    case SEQ_UI_BUTTON_GP12: // select Options Page
      if( depressed ) return -1;
      if( in_menu_msg & 0x80 )
	return 0; // ignore as long as message is displayed
      return SEQ_UI_PageSet(SEQ_UI_PAGE_OPT);

    case SEQ_UI_BUTTON_GP13: // select Port Mute page
      if( depressed ) return -1;
      if( in_menu_msg & 0x80 )
	return 0; // ignore as long as message is displayed
      return SEQ_UI_PageSet(SEQ_UI_PAGE_PMUTE);

    case SEQ_UI_BUTTON_GP14: // Disk Page
      if( depressed ) return -1;
      if( in_menu_msg & 0x80 )
	return 0; // ignore as long as message is displayed
      return SEQ_UI_PageSet(SEQ_UI_PAGE_DISK);

    case SEQ_UI_BUTTON_GP15: // mute all tracks
      if( depressed ) return -1;
      if( in_menu_msg & 0x80 )
	return 0; // ignore as long as message is displayed
      SEQ_UI_UTIL_MuteAllTracks();
      SEQ_UI_PageSet(SEQ_UI_PAGE_MUTE);
      return 1;

    case SEQ_UI_BUTTON_GP16: // unmute all tracks
      if( depressed ) return -1;
      if( in_menu_msg & 0x80 )
	return 0; // ignore as long as message is displayed
      SEQ_UI_UTIL_UnMuteAllTracks();
      SEQ_UI_PageSet(SEQ_UI_PAGE_MUTE);
      return 1;

    case SEQ_UI_BUTTON_Select:
    case SEQ_UI_BUTTON_Right:
      if( depressed ) return -1;
      return 0;

    case SEQ_UI_BUTTON_Left:
      if( depressed ) return -1;
      return 0;

    case SEQ_UI_BUTTON_Up:
      if( depressed ) return -1;
      return Encoder_Handler(SEQ_UI_ENCODER_Datawheel, 1);

    case SEQ_UI_BUTTON_Down:
      if( depressed ) return -1;
      return Encoder_Handler(SEQ_UI_ENCODER_Datawheel, -1);
  }

  return -1; // invalid or unsupported button
}


/////////////////////////////////////////////////////////////////////////////
// Local Display Handler function
// IN: <high_prio>: if set, a high-priority LCD update is requested
/////////////////////////////////////////////////////////////////////////////
static s32 LCD_Handler(u8 high_prio)
{
  // branch to edit page if copy/paste/move/scroll button is pressed
  switch( in_menu_msg ) {
    case MSG_COPY: return SEQ_UI_EDIT_LCD_Handler(high_prio, SEQ_UI_EDIT_MODE_COPY);
    case MSG_PASTE: return SEQ_UI_EDIT_LCD_Handler(high_prio, SEQ_UI_EDIT_MODE_PASTE);
    case MSG_MOVE: return SEQ_UI_EDIT_LCD_Handler(high_prio, SEQ_UI_EDIT_MODE_MOVE);
    case MSG_SCROLL: return SEQ_UI_EDIT_LCD_Handler(high_prio, SEQ_UI_EDIT_MODE_SCROLL);
  }

  if( high_prio )
    return 0; // there are no high-priority update

  // layout:
  // 00000000001111111111222222222233333333330000000000111111111122222222223333333333
  // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
  // <--------------------------------------><-------------------------------------->
  // Trk.        Utility Functions                       Quick Menu Change           
  // G1T1 Copy Paste Clr Move Scrl Rand Undo Save  Jam  CV  Opt. PMte Disk Mute UnMte

  ///////////////////////////////////////////////////////////////////////////
  SEQ_LCD_CursorSet(0, 0);
  SEQ_LCD_PrintString("Trk.        Utility Functions   ");

  if( (in_menu_msg & 0x80) || ((in_menu_msg & 0x7f) && ui_hold_msg_ctr) ) {
    SEQ_LCD_PrintString((char *)in_menu_msg_str[(in_menu_msg & 0x7f)-1]);
  } else {
    SEQ_LCD_PrintSpaces(8);
  }
  SEQ_LCD_PrintString("            Quick Menu Change           ");


  ///////////////////////////////////////////////////////////////////////////
  SEQ_LCD_CursorSet(0, 1);

  //  if( ui_selected_item == ITEM_GXTY && ui_cursor_flash ) {
  if( ui_cursor_flash ) {
    SEQ_LCD_PrintSpaces(4);
  } else {
    SEQ_LCD_PrintGxTy(ui_selected_group, ui_selected_tracks);
  }

  SEQ_LCD_PrintString(" Copy Paste Clr Move Scrl Rand Undo ");
  SEQ_LCD_PrintString("Save  Jam  CV  Opt. PMte Disk Mute UnMte");

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Initialisation
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_UI_UTIL_Init(u32 mode)
{
  // install callback routines
  SEQ_UI_InstallButtonCallback(Button_Handler);
  SEQ_UI_InstallEncoderCallback(Encoder_Handler);
  SEQ_UI_InstallLEDCallback(LED_Handler);
  SEQ_UI_InstallLCDCallback(LCD_Handler);

  ui_hold_msg_ctr = 0;

  //  copypaste_begin = copypaste_end = 0;
  // in_menu_msg = MSG_DEFAULT; // clashes with external Copy/Paste button function, since
  // SEQ_UI_UTIL_Init() function will be called later by background task, while buttons are handled
  // by SRIO task -- therefore don't init in_menu_msg and copypaste pointers

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Functions used by SEQ_UI_EDIT
/////////////////////////////////////////////////////////////////////////////
u8 SEQ_UI_UTIL_CopyPasteBeginGet(void)
{
  return copypaste_begin;
}

u8 SEQ_UI_UTIL_CopyPasteEndGet(void)
{
  return copypaste_end;
}


/////////////////////////////////////////////////////////////////////////////
// Copy track
/////////////////////////////////////////////////////////////////////////////
static s32 COPY_Track(u8 track)
{
  int i;

  // copy layers into buffer
  memcpy((u8 *)copypaste_par_layer, (u8 *)&seq_par_layer_value[track], SEQ_PAR_MAX_BYTES);
  memcpy((u8 *)copypaste_trg_layer, (u8 *)&seq_trg_layer_value[track], SEQ_TRG_MAX_BYTES);

  // copy track name
  memcpy((u8 *)copypaste_trk_name, (u8 *)seq_core_trk[track].name, 81);

  // copy CCs
  for(i=0; i<128; ++i)
    copypaste_cc[i] = SEQ_CC_Get(track, i);

  copypaste_par_layers = SEQ_PAR_NumLayersGet(track);
  copypaste_par_steps = SEQ_PAR_NumStepsGet(track);
  copypaste_trg_layers = SEQ_TRG_NumLayersGet(track);
  copypaste_trg_steps = SEQ_TRG_NumStepsGet(track);
  copypaste_num_instruments = SEQ_PAR_NumInstrumentsGet(track);
  copypaste_selected_par_layer = ui_selected_par_layer;
  copypaste_selected_trg_layer = ui_selected_trg_layer;
  copypaste_selected_instrument = ui_selected_instrument;

  // notify that copy&paste buffer is filled
  copypaste_buffer_filled = 1;
  copypaste_track = track;

  return 0; // no error
}

/////////////////////////////////////////////////////////////////////////////
// Paste a track with selectable offset (stored in ui_selected_step)
/////////////////////////////////////////////////////////////////////////////
static s32 PASTE_Track(u8 track, paste_clear_mode_t paste_clear_mode)
{
  int instrument;
  int layer;
  int step;

  // branch to clear function if copy&paste buffer not filled
  if( !copypaste_buffer_filled )
    return CLEAR_Track(track, paste_clear_mode);

  if( paste_clear_mode == PASTE_CLEAR_MODE_SELECT ) {
    if( seq_ui_button_state.SELECT_PRESSED ) {
      u8 event_mode = SEQ_CC_Get(track, SEQ_CC_MIDI_EVENT_MODE);
      paste_clear_mode = (event_mode == SEQ_EVENT_MODE_Drum) ? PASTE_CLEAR_MODE_INS_LAYER : PASTE_CLEAR_MODE_PAR_LAYER;
    } else {
      paste_clear_mode = PASTE_CLEAR_MODE_TRACK;
    }
  }

  // determine begin/end boundary
  int step_begin = copypaste_begin;
  int step_end = copypaste_end;
  // swap if required
  if( step_begin > step_end ) {
    int tmp = step_end;
    step_end = step_begin;
    step_begin = tmp;
  }

  int num_par_instruments = SEQ_PAR_NumInstrumentsGet(track);
  int num_par_layers = SEQ_PAR_NumLayersGet(track);
  int num_par_steps = SEQ_PAR_NumStepsGet(track);
  int num_trg_instruments = SEQ_TRG_NumInstrumentsGet(track);
  int num_trg_layers = SEQ_TRG_NumLayersGet(track);
  int num_trg_steps = SEQ_TRG_NumStepsGet(track);

  // branch depending on paste/clear mode
  switch( paste_clear_mode ) {
  case PASTE_CLEAR_MODE_TRACK: {
    //seq_event_mode_t prev_event_mode = SEQ_CC_Get(track, SEQ_CC_MIDI_EVENT_MODE);

    // take over mode - but only if it has been changed so that new partitioning is required!
    if( SEQ_CC_Get(track, SEQ_CC_MIDI_EVENT_MODE) != copypaste_cc[SEQ_CC_MIDI_EVENT_MODE] ) {
      SEQ_CC_Set(track, SEQ_CC_MIDI_EVENT_MODE, copypaste_cc[SEQ_CC_MIDI_EVENT_MODE]);
      SEQ_CC_LinkUpdate(track);
      SEQ_PAR_TrackInit(track, copypaste_par_steps, copypaste_par_layers, copypaste_num_instruments);
      SEQ_TRG_TrackInit(track, copypaste_trg_steps, copypaste_trg_layers, copypaste_num_instruments);
    }

    // copy CCs
    if( seq_core_options.PASTE_CLR_ALL ) {
      int i;

      for(i=0; i<128; ++i)
	SEQ_CC_Set(track, i, copypaste_cc[i]);
    } else {
      // we have to copy the 48 lower CCs to avoid garbage output
      int i;
      for(i=0; i<48; ++i)
	SEQ_CC_Set(track, i, copypaste_cc[i]);

      // and we have to copy the drum parameter layer assignments
      for(i=0; i<4; ++i) {
	SEQ_CC_Set(track, SEQ_CC_PAR_ASG_DRUM_LAYER_A+i, copypaste_cc[SEQ_CC_PAR_ASG_DRUM_LAYER_A+i]);
      }
    }

    // copy layers from buffer
    for(instrument=0; instrument<num_par_instruments && instrument < copypaste_num_instruments; ++instrument) {
      for(layer=0; layer<num_par_layers && layer<copypaste_par_layers; ++layer) {
	int step_offset = ui_selected_step;
	for(step=step_begin; step<=step_end && step<copypaste_par_steps; ++step, ++step_offset) {
	  if( step_offset < num_par_steps ) {
	    u16 step_ix = (instrument * copypaste_par_layers * copypaste_par_steps) + layer * copypaste_par_steps + step;
	    SEQ_PAR_Set(track, step_offset, layer, instrument, copypaste_par_layer[step_ix]);
	  }
	}
      }
    }

    // copy triggers from buffer
    for(instrument=0; instrument<num_trg_instruments && instrument < copypaste_num_instruments; ++instrument) {
      for(layer=0; layer<num_trg_layers && layer<copypaste_trg_layers; ++layer) {
	int step_offset = ui_selected_step;
	for(step=step_begin; step<=step_end && step<copypaste_trg_steps; ++step, ++step_offset) {
	  if( step_offset < num_trg_steps ) {
	    u8 step8_ix = (instrument * copypaste_trg_layers * (copypaste_trg_steps/8)) + layer * (copypaste_trg_steps/8) + (step/8);
	    u8 step_mask = (1 << (step&7));
	    SEQ_TRG_Set(track, step_offset, layer, instrument, (copypaste_trg_layer[step8_ix] & step_mask) ? 1 : 0);
	  }
	}
      }
    }

    // copy track name
    memcpy((u8 *)seq_core_trk[track].name, (u8 *)copypaste_trk_name, 81);
  } break;

  case PASTE_CLEAR_MODE_PAR_LAYER: {
    int step_offset = ui_selected_step;
    for(step=step_begin; step<=step_end && step<copypaste_par_steps; ++step, ++step_offset) {
      if( step_offset < num_par_steps ) {
	u16 step_ix = (ui_selected_instrument * copypaste_par_layers * copypaste_par_steps) + copypaste_selected_par_layer * copypaste_par_steps + step;
	SEQ_PAR_Set(track, step_offset, ui_selected_par_layer, ui_selected_instrument, copypaste_par_layer[step_ix]);
      }
    }

    // copy parameter type
    seq_par_layer_type_t par_type = copypaste_cc[SEQ_CC_LAY_CONST_A1 + copypaste_selected_par_layer];
    SEQ_CC_Set(track, SEQ_CC_LAY_CONST_A1 + ui_selected_par_layer, (u8)par_type);
    SEQ_CC_LinkUpdate(track);

    // some additional operations depending on parameter type
    if( par_type == SEQ_PAR_Type_Note || par_type == SEQ_PAR_Type_Chord1 || par_type == SEQ_PAR_Type_Chord2 || par_type == SEQ_PAR_Type_Chord3 ) {
      // set gates (don't clear already enabled gates)
      u8 trg_gate_assignment = copypaste_cc[SEQ_CC_ASG_GATE];
      if( trg_gate_assignment >= 1 ) {
	trg_gate_assignment -= 1;

	int step_offset = ui_selected_step;
	for(step=step_begin; step<=step_end && step<copypaste_trg_steps; ++step, ++step_offset) {
	  if( step_offset < num_trg_steps ) {
	    u8 step8_ix = (ui_selected_instrument * copypaste_trg_layers * (copypaste_trg_steps/8)) + trg_gate_assignment * (copypaste_trg_steps/8) + (step/8);
	    u8 step_mask = (1 << (step&7));
	    if( copypaste_trg_layer[step8_ix] & step_mask ) {
	      SEQ_TRG_GateSet(track, step_offset, ui_selected_instrument, 1);
	    }
	  }
	}
      }
    }
  } break;

  case PASTE_CLEAR_MODE_TRG_LAYER: {
    int step_offset = ui_selected_step;
    for(step=step_begin; step<=step_end && step<copypaste_trg_steps; ++step, ++step_offset) {
      if( step_offset < num_trg_steps ) {
	u8 step8_ix = (copypaste_selected_instrument * copypaste_trg_layers * (copypaste_trg_steps/8)) + ui_selected_trg_layer * (copypaste_trg_steps/8) + (step/8);
	u8 step_mask = (1 << (step&7));
	SEQ_TRG_Set(track, step_offset, ui_selected_trg_layer, ui_selected_instrument, (copypaste_trg_layer[step8_ix] & step_mask) ? 1 : 0);
      }
    }
  } break;

  case PASTE_CLEAR_MODE_INS_LAYER: {
    // parameter layer
    for(layer=0; layer<num_par_layers && layer<copypaste_par_layers; ++layer) {
      int step_offset = ui_selected_step;
      for(step=step_begin; step<=step_end && step<copypaste_par_steps; ++step, ++step_offset) {
	if( step_offset < num_par_steps ) {
	  u16 step_ix = (ui_selected_instrument * copypaste_par_layers * copypaste_par_steps) + layer * copypaste_par_steps + step;
	  SEQ_PAR_Set(track, step_offset, layer, ui_selected_instrument, copypaste_par_layer[step_ix]);
	}
      }
    }

    // copy parameter type
    seq_par_layer_type_t par_type = copypaste_cc[SEQ_CC_LAY_CONST_A1 + copypaste_selected_par_layer];
    SEQ_CC_Set(track, SEQ_CC_LAY_CONST_A1 + ui_selected_par_layer, (u8)par_type);
    SEQ_CC_LinkUpdate(track);

    // trigger layer
    for(layer=0; layer<num_trg_layers && layer<copypaste_trg_layers; ++layer) {
      int step_offset = ui_selected_step;
      for(step=step_begin; step<=step_end && step<copypaste_trg_steps; ++step, ++step_offset) {
	if( step_offset < num_trg_steps ) {
	  u8 step8_ix = (copypaste_selected_instrument * copypaste_trg_layers * (copypaste_trg_steps/8)) + layer * (copypaste_trg_steps/8) + (step/8);
	  u8 step_mask = (1 << (step&7));
	  SEQ_TRG_Set(track, step_offset, layer, ui_selected_instrument, (copypaste_trg_layer[step8_ix] & step_mask) ? 1 : 0);
	}
      }
    }
  } break;

  default:
    DEBUG_MSG("[PASTE_Track] unsupported paste_clear_mode=%d\n", paste_clear_mode);
  }

  // cancel sustain if there are no steps played by the track anymore.
  SEQ_CORE_CancelSustainedNotes(track);

  return 0; // no error
}

/////////////////////////////////////////////////////////////////////////////
// COPY+PASTE pressed: duplicates the steps
// return 1 if successfull, and 0 if no duplication possible (track full)
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_UI_UTIL_PasteDuplicateSteps(u8 track)
{
  COPY_Track(track);

  int length = (int)SEQ_CC_Get(track, SEQ_CC_LENGTH) + 1;
  int num_trg_steps = SEQ_TRG_NumStepsGet(track);
  if( 2*length > num_trg_steps )
    return 0; // no duplication possible

  copypaste_begin = 0;
  copypaste_end = length - 1;
  SEQ_UI_SelectedStepSet(length);
  PASTE_Track(track, PASTE_CLEAR_MODE_SELECT);

  copypaste_end = 2*length - 1;
  SEQ_CC_Set(track, SEQ_CC_LENGTH, copypaste_end);

  return 1;
}


/////////////////////////////////////////////////////////////////////////////
// CLEAR a specific step
// return 1 if successfull
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_UI_UTIL_ClearStep(u8 track, u8 step, u8 instrument)
{
  {
    int num_par_layers = SEQ_TRG_NumLayersGet(track);
    int par_layer;
    for(par_layer=0; par_layer<num_par_layers; ++par_layer) {
      u8 init_value = SEQ_PAR_InitValueGet(SEQ_PAR_AssignmentGet(track, par_layer), par_layer);
      SEQ_PAR_Set(track, step, par_layer, instrument, init_value);
    }
  }

  {
    int num_trg_layers = SEQ_TRG_NumLayersGet(track);
    int trg_layer;
    for(trg_layer=0; trg_layer<num_trg_layers; ++trg_layer) {
      SEQ_TRG_Set(track, step, trg_layer, instrument, 0);
    }
  }

  return 1;
}


/////////////////////////////////////////////////////////////////////////////
// clear a track
/////////////////////////////////////////////////////////////////////////////
static s32 CLEAR_Track(u8 track, paste_clear_mode_t paste_clear_mode)
{
  if( paste_clear_mode == PASTE_CLEAR_MODE_SELECT ) {
    if( seq_ui_button_state.SELECT_PRESSED ) {
      u8 event_mode = SEQ_CC_Get(track, SEQ_CC_MIDI_EVENT_MODE);
      paste_clear_mode = (event_mode == SEQ_EVENT_MODE_Drum) ? PASTE_CLEAR_MODE_INS_LAYER : PASTE_CLEAR_MODE_PAR_LAYER;
    } else {
      paste_clear_mode = PASTE_CLEAR_MODE_TRACK;
    }
  }

  // branch depending on paste/clear mode
  switch( paste_clear_mode ) {
  case PASTE_CLEAR_MODE_TRACK: {
    u8 only_layers = seq_core_options.PASTE_CLR_ALL ? 0 : 1;
    u8 all_triggers_cleared = 0;
    u8 init_assignments = 0;
    SEQ_LAYER_CopyPreset(track, only_layers, all_triggers_cleared, init_assignments);

    // clear all triggers
    memset((u8 *)&seq_trg_layer_value[track], 0, SEQ_TRG_MAX_BYTES);
  } break;

  case PASTE_CLEAR_MODE_PAR_LAYER: {
    SEQ_LAYER_CopyParLayerPreset(track, ui_selected_par_layer);
  } break;

  case PASTE_CLEAR_MODE_TRG_LAYER: {
    int num_trg_steps = SEQ_TRG_NumStepsGet(track);
    int step;
    for(step=0; step<num_trg_steps; ++step) {
      SEQ_TRG_Set(track, step, ui_selected_trg_layer, ui_selected_instrument, 0);
    }
  } break;

  case PASTE_CLEAR_MODE_INS_LAYER: {
    SEQ_LAYER_CopyParLayerPreset(track, ui_selected_par_layer);

    int num_trg_steps = SEQ_TRG_NumStepsGet(track);
    int step;
    for(step=0; step<num_trg_steps; ++step) {
      SEQ_TRG_Set(track, step, ui_selected_trg_layer, ui_selected_instrument, 0);
    }
  } break;

  default:
    DEBUG_MSG("[CLEAR_Track] unsupported paste_clear_mode=%d\n", paste_clear_mode);
  }

  // cancel sustain if there are no steps played by the track anymore.
  SEQ_CORE_CancelSustainedNotes(track);

  return 0; // no error
}

/////////////////////////////////////////////////////////////////////////////
// UnDo function
/////////////////////////////////////////////////////////////////////////////
static s32 UNDO_Track(void)
{
#if UNDO_ENABLED
  // exit if undo buffer not filled
  if( !undo_buffer_filled )
    return 0; // no error

  SEQ_CC_Set(undo_track, SEQ_CC_MIDI_EVENT_MODE, undo_cc[SEQ_CC_MIDI_EVENT_MODE]);
  SEQ_CC_LinkUpdate(undo_track);
  SEQ_PAR_TrackInit(undo_track, undo_par_steps, undo_par_layers, undo_num_instruments);
  SEQ_TRG_TrackInit(undo_track, undo_trg_steps, undo_trg_layers, undo_num_instruments);

  // copy layers from buffer
  memcpy((u8 *)&seq_par_layer_value[undo_track], (u8 *)undo_par_layer, SEQ_PAR_MAX_BYTES);
  memcpy((u8 *)&seq_trg_layer_value[undo_track], (u8 *)undo_trg_layer, SEQ_TRG_MAX_BYTES);

  // copy track name
  memcpy((u8 *)seq_core_trk[undo_track].name, (u8 *)undo_trk_name, 81);

  // copy CCs
  if( seq_core_options.PASTE_CLR_ALL ) {
    int i;

    for(i=0; i<128; ++i)
	SEQ_CC_Set(undo_track, i, undo_cc[i]);
  }

  // cancel sustain if there are no steps played by the track anymore.
  SEQ_CORE_CancelSustainedNotes(undo_track);
#endif

  return 0; // no error
}

/////////////////////////////////////////////////////////////////////////////
// Updates the UnDo buffer - can also be called from external (e.g. TRKRND)
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_UI_UTIL_UndoUpdate(u8 track)
{
  int i;

#if UNDO_ENABLED
  // store track in special variable, so that we restore to the right one later
  undo_track = track;

  // copy layers into buffer
  memcpy((u8 *)undo_par_layer, (u8 *)&seq_par_layer_value[track], SEQ_PAR_MAX_BYTES);
  memcpy((u8 *)undo_trg_layer, (u8 *)&seq_trg_layer_value[track], SEQ_TRG_MAX_BYTES);

  // copy track name
  memcpy((u8 *)undo_trk_name, (u8 *)seq_core_trk[undo_track].name, 81);

  // copy CCs
  for(i=0; i<128; ++i)
    undo_cc[i] = SEQ_CC_Get(track, i);

  undo_par_layers = SEQ_PAR_NumLayersGet(track);
  undo_par_steps = SEQ_PAR_NumStepsGet(track);
  undo_trg_layers = SEQ_TRG_NumLayersGet(track);
  undo_trg_steps = SEQ_TRG_NumStepsGet(track);
  undo_num_instruments = SEQ_PAR_NumInstrumentsGet(track);

  // notify that undo buffer is filled
  undo_buffer_filled = 1;
#endif

  return 0; // no error
}

/////////////////////////////////////////////////////////////////////////////
// Copy/Paste/Clear Live pattern
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_UI_UTIL_CopyLivePattern(void)
{
  seq_live_pattern_slot_t *slot = SEQ_LIVE_CurrentSlotGet();
  if( slot->pattern >= SEQ_LIVE_NUM_ARP_PATTERNS )
    return -1; // invalid pattern
  seq_live_arp_pattern_t *pattern = (seq_live_arp_pattern_t *)&seq_live_arp_pattern[slot->pattern];

  u8 visible_track = SEQ_UI_VisibleTrackGet();

  // start with common copy operation
  COPY_Track(visible_track);
  copypaste_begin = 0;
  copypaste_end = 15;

  u8 event_mode = copypaste_cc[SEQ_CC_MIDI_EVENT_MODE];

  u8 trg_instrument = (event_mode == SEQ_EVENT_MODE_Drum) ? ui_selected_instrument : 0;
  u8 num_t_layers = copypaste_trg_layers;
  u8 num_t_steps8 = copypaste_trg_steps / 8;

  {
    u8 gate_assignment = copypaste_cc[SEQ_CC_ASG_GATE];
    if( gate_assignment > 0 ) {
      // similar to SEQ_TRG_Get16:
      u8 trg_layer = gate_assignment - 1;
      u16 step_ix = (trg_instrument * num_t_layers * num_t_steps8) + (trg_layer * num_t_steps8);
      if( step_ix >= SEQ_TRG_MAX_BYTES )
	step_ix = 0;

      {
	u8 *values = (u8 *)&copypaste_trg_layer[step_ix];
	*values = pattern->gate;
	++values;
	*values = (pattern->gate >> 8);
      }
    }
  }

  {
    u8 accent_assignment = copypaste_cc[SEQ_CC_ASG_ACCENT];
    if( accent_assignment > 0 ) {
      // similar to SEQ_TRG_Get16:
      u8 trg_layer = accent_assignment - 1;
      u16 step_ix = (trg_instrument * num_t_layers * num_t_steps8) + (trg_layer * num_t_steps8);
      if( step_ix >= SEQ_TRG_MAX_BYTES )
	step_ix = 0;

      {
	u8 *values = (u8 *)&copypaste_trg_layer[step_ix];
	*values = pattern->accent;
	++values;
	*values = (pattern->accent >> 8);
      }
    }
  }

  return 0; // no error
}

s32 SEQ_UI_UTIL_PasteLivePattern(void)
{
  if( !copypaste_buffer_filled )
    return -1; // edit buffer empty

  seq_live_pattern_slot_t *slot = SEQ_LIVE_CurrentSlotGet();
  if( slot->pattern >= SEQ_LIVE_NUM_ARP_PATTERNS )
    return -1; // invalid pattern
  seq_live_arp_pattern_t *pattern = (seq_live_arp_pattern_t *)&seq_live_arp_pattern[slot->pattern];

  //u8 visible_track = SEQ_UI_VisibleTrackGet();
  u8 event_mode = copypaste_cc[SEQ_CC_MIDI_EVENT_MODE];

  u8 trg_instrument = (event_mode == SEQ_EVENT_MODE_Drum) ? ui_selected_instrument : 0;
  u8 num_t_layers = copypaste_trg_layers;
  u8 num_t_steps8 = copypaste_trg_steps / 8;

  {
    u16 gate = 0;
    u8 gate_assignment = copypaste_cc[SEQ_CC_ASG_GATE];
    if( gate_assignment == 0 ) {
      gate = 0xffff; // no assignment -> gate always on
    } else {
      // similar to SEQ_TRG_Get16:
      u8 trg_layer = gate_assignment - 1;
      u16 step_ix = (trg_instrument * num_t_layers * num_t_steps8) + (trg_layer * num_t_steps8);
      if( step_ix >= SEQ_TRG_MAX_BYTES )
	step_ix = 0;

      {
	u8 *values = (u8 *)&copypaste_trg_layer[step_ix];
	gate = *values;
	++values;
	gate |= ((u16)*values << 8);
      }
    }
    pattern->gate = gate;
  }

  {
    u16 accent = 0;
    u8 accent_assignment = copypaste_cc[SEQ_CC_ASG_ACCENT];
    if( accent_assignment == 0 ) {
      accent = 0; // no assignment -> accent always off
    } else {
      // similar to SEQ_TRG_Get16:
      u8 trg_layer = accent_assignment - 1;
      u16 step_ix = (trg_instrument * num_t_layers * num_t_steps8) + (trg_layer * num_t_steps8);
      if( step_ix >= SEQ_TRG_MAX_BYTES )
	step_ix = 0;

      {
	u8 *values = (u8 *)&copypaste_trg_layer[step_ix];
	accent = *values;
	++values;
	accent |= ((u16)*values << 8);
      }
    }
    pattern->accent = accent;
  }

  return 0; // no error
}

s32 SEQ_UI_UTIL_ClearLivePattern(void)
{
  seq_live_pattern_slot_t *slot = SEQ_LIVE_CurrentSlotGet();
  if( slot->pattern >= SEQ_LIVE_NUM_ARP_PATTERNS )
    return -1; // invalid pattern
  seq_live_arp_pattern_t *pattern = (seq_live_arp_pattern_t *)&seq_live_arp_pattern[slot->pattern];

  pattern->gate = 0;
  pattern->accent = 0;

  return 0; // no error
}

/////////////////////////////////////////////////////////////////////////////
// Copy/Paste/Clear Parameter Layer
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_UI_UTIL_CopyParLayer(void)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();
  return COPY_Track(visible_track);
}

s32 SEQ_UI_UTIL_PasteParLayer(void)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();

  // currently only supports to paste from first step
  SEQ_UI_SelectedStepSet(0); // set new visible step/view

  // update undo buffer
  SEQ_UI_UTIL_UndoUpdate(visible_track);

  return PASTE_Track(visible_track, PASTE_CLEAR_MODE_PAR_LAYER);
}

s32 SEQ_UI_UTIL_ClearParLayer(void)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();

  // update undo buffer
  SEQ_UI_UTIL_UndoUpdate(visible_track);

  return CLEAR_Track(visible_track, PASTE_CLEAR_MODE_PAR_LAYER);
}

/////////////////////////////////////////////////////////////////////////////
// Copy/Paste/Clear Trigger Layer
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_UI_UTIL_CopyTrgLayer(void)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();
  return COPY_Track(visible_track);
}

s32 SEQ_UI_UTIL_PasteTrgLayer(void)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();

  // currently only supports to paste from first step
  SEQ_UI_SelectedStepSet(0); // set new visible step/view

  // update undo buffer
  SEQ_UI_UTIL_UndoUpdate(visible_track);

  return PASTE_Track(visible_track, PASTE_CLEAR_MODE_TRG_LAYER);
}

s32 SEQ_UI_UTIL_ClearTrgLayer(void)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();

  // currently only supports to paste from first step
  SEQ_UI_SelectedStepSet(0); // set new visible step/view

  // update undo buffer
  SEQ_UI_UTIL_UndoUpdate(visible_track);

  return CLEAR_Track(visible_track, PASTE_CLEAR_MODE_TRG_LAYER);
}

/////////////////////////////////////////////////////////////////////////////
// Copy/Paste/Clear Instrument
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_UI_UTIL_CopyInsLayer(void)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();
  return COPY_Track(visible_track);
}

s32 SEQ_UI_UTIL_PasteInsLayer(void)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();

  // update undo buffer
  SEQ_UI_UTIL_UndoUpdate(visible_track);

  return PASTE_Track(visible_track, PASTE_CLEAR_MODE_INS_LAYER);
}

s32 SEQ_UI_UTIL_ClearInsLayer(void)
{
  u8 visible_track = SEQ_UI_VisibleTrackGet();

  // update undo buffer
  SEQ_UI_UTIL_UndoUpdate(visible_track);

  return CLEAR_Track(visible_track, PASTE_CLEAR_MODE_INS_LAYER);
}

/////////////////////////////////////////////////////////////////////////////
// Help functions for move step feature
/////////////////////////////////////////////////////////////////////////////
static s32 MOVE_StoreStep(u8 track, u16 step, u8 buffer, u8 clr_triggers)
{
  int layer;

  for(layer=0; layer<16; ++layer)
    move_par_layer[buffer][layer] = SEQ_PAR_Get(track, step, layer, ui_selected_instrument);

  move_trg_layer[buffer] = 0;
  if( !clr_triggers ) {
    for(layer=0; layer<16; ++layer)
      if( SEQ_TRG_Get(track, step, layer, ui_selected_instrument) )
	move_trg_layer[buffer] |= (1 << layer);
  }

  return 0; // no error
}

static s32 MOVE_RestoreStep(u8 track, u16 step, u8 buffer)
{
  int layer;

  for(layer=0; layer<16; ++layer)
    SEQ_PAR_Set(track, step, layer, ui_selected_instrument, move_par_layer[buffer][layer]);

  for(layer=0; layer<16; ++layer)
    SEQ_TRG_Set(track, step, layer, ui_selected_instrument, (move_trg_layer[buffer] & (1 << layer)) ? 1 : 0);

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// Scroll function
/////////////////////////////////////////////////////////////////////////////
static s32 SCROLL_Track(u8 track, u16 first_step, s32 incrementer)
{
  int instrument;
  int layer;
  int step;

  // determine the last step which has to be rotated
  int last_step = SEQ_CC_Get(track, SEQ_CC_LENGTH);
  if( first_step > last_step ) {
    // loop point behind track len -> rotate complete track
    first_step = 0;
    last_step = SEQ_TRG_NumStepsGet(track)-1;
  }

  if( first_step < last_step ) {
    if( incrementer >= 0 ) {
      // rightrotate parameter layers
      int num_instruments = SEQ_PAR_NumInstrumentsGet(track);
      int num_layers = SEQ_PAR_NumLayersGet(track);
      for(instrument=0; instrument<num_instruments; ++instrument) {
	for(layer=0; layer<num_layers; ++layer) {
	  u8 tmp = SEQ_PAR_Get(track, last_step, layer, instrument);
	  for(step=last_step; step>first_step; --step)
	    SEQ_PAR_Set(track, step, layer, instrument, SEQ_PAR_Get(track, step-1, layer, instrument));
	  SEQ_PAR_Set(track, step, layer, instrument, tmp);
	}
      }

      // rightrotate trigger layers
      num_instruments = SEQ_TRG_NumInstrumentsGet(track);
      num_layers = SEQ_TRG_NumLayersGet(track);
      for(instrument=0; instrument<num_instruments; ++instrument) {
	for(layer=0; layer<num_layers; ++layer) {
	  u8 tmp = SEQ_TRG_Get(track, last_step, layer, instrument);
	  for(step=last_step; step>first_step; --step)
	    SEQ_TRG_Set(track, step, layer, instrument, SEQ_TRG_Get(track, step-1, layer, instrument));
	  SEQ_TRG_Set(track, step, layer, instrument, tmp);
	}
      }
    } else {
      // leftrotate parameter layers
      int num_instruments = SEQ_PAR_NumInstrumentsGet(track);
      int num_layers = SEQ_PAR_NumLayersGet(track);
      for(instrument=0; instrument<num_instruments; ++instrument) {
	for(layer=0; layer<num_layers; ++layer) {
	  u8 tmp = SEQ_PAR_Get(track, first_step, layer, instrument);
	  for(step=first_step; step<last_step; ++step)
	    SEQ_PAR_Set(track, step, layer, instrument, SEQ_PAR_Get(track, step+1, layer, instrument));
	  SEQ_PAR_Set(track, step, layer, instrument, tmp);
	}
      }

      // leftrotate trigger layers
      num_instruments = SEQ_TRG_NumInstrumentsGet(track);
      num_layers = SEQ_TRG_NumLayersGet(track);
      for(instrument=0; instrument<num_instruments; ++instrument) {
	for(layer=0; layer<num_layers; ++layer) {
	  u8 tmp = SEQ_TRG_Get(track, first_step, layer, instrument);
	  for(step=first_step; step<last_step; ++step)
	    SEQ_TRG_Set(track, step, layer, instrument, SEQ_TRG_Get(track, step+1, layer, instrument));
	  SEQ_TRG_Set(track, step, layer, instrument, tmp);
	}
      }
    }
  }

  return 0; // no error
}


/////////////////////////////////////////////////////////////////////////////
// This function mutes all tracks
/////////////////////////////////////////////////////////////////////////////
static s32 SEQ_UI_UTIL_MuteAllTracks(void)
{
  seq_core_trk_muted = 0xffff;

  return 0; // no error
}

/////////////////////////////////////////////////////////////////////////////
// This function unmutes all tracks
/////////////////////////////////////////////////////////////////////////////
static s32 SEQ_UI_UTIL_UnMuteAllTracks(void)
{
  seq_core_trk_muted = 0x0000;

  return 0; // no error
}



/////////////////////////////////////////////////////////////////////////////
// For direct access to copy/paste/clear/undo function
// (e.g. via special button, but also from other pages -> TRKRND)
/////////////////////////////////////////////////////////////////////////////
s32 SEQ_UI_UTIL_CopyButton(s32 depressed)
{
  return Button_Handler(SEQ_UI_BUTTON_GP2, depressed);
}

s32 SEQ_UI_UTIL_PasteButton(s32 depressed)
{
  return Button_Handler(SEQ_UI_BUTTON_GP3, depressed);
}

s32 SEQ_UI_UTIL_ClearButton(s32 depressed)
{
  return Button_Handler(SEQ_UI_BUTTON_GP4, depressed);
}

s32 SEQ_UI_UTIL_MoveButton(s32 depressed)
{
  return Button_Handler(SEQ_UI_BUTTON_GP5, depressed);
}

s32 SEQ_UI_UTIL_ScrollButton(s32 depressed)
{
  return Button_Handler(SEQ_UI_BUTTON_GP6, depressed);
}

s32 SEQ_UI_UTIL_UndoButton(s32 depressed)
{
  return Button_Handler(SEQ_UI_BUTTON_GP8, depressed);
}
