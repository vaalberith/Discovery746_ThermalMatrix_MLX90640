/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.44                          *
*        Compiled Nov 10 2017, 08:53:57                              *
*        (c) 2017 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
#include "main.h"
#include "math.h"
// USER END

#include "DIALOG.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0              (GUI_ID_USER + 0x00)
#define ID_BUTTON_0              (GUI_ID_USER + 0x01)
#define ID_BUTTON_1              (GUI_ID_USER + 0x02)
#define ID_SLIDER_0              (GUI_ID_USER + 0x03)
#define ID_SLIDER_1              (GUI_ID_USER + 0x04)
#define ID_BUTTON_2              (GUI_ID_USER + 0x05)
#define ID_CHECKBOX_0            (GUI_ID_USER + 0x06)
#define ID_TEXT_0            (GUI_ID_USER + 0x07)
#define ID_TEXT_1            (GUI_ID_USER + 0x08)
#define ID_SLIDER_2            (GUI_ID_USER + 0x09)
#define ID_SLIDER_3            (GUI_ID_USER + 0x0A)
#define ID_TEXT_2            (GUI_ID_USER + 0x0B)
#define ID_CHECKBOX_1            (GUI_ID_USER + 0x0C)


// USER START (Optionally insert additional defines)
#include "stdio.h"
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)

extern uint8_t chess_mode;
extern uint8_t freq;
extern float scale_factor;
extern uint8_t better_color_ramping;

extern uint8_t scan_enabled;
extern uint8_t need_to_clear;
extern uint8_t need_to_redraw;
extern uint8_t need_to_save;
extern uint8_t need_to_cfg;
extern uint8_t need_to_cfg_scale;

extern float T_MIN_param;
extern float T_MAX_param;

WM_HWIN temp_text_inst;

void temp_setlabel(char *str)
{
  TEXT_SetText(temp_text_inst, str);
}

// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 45, 0, 75, 272, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Clear", ID_BUTTON_0, 0, 40, 75, 40, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Save", ID_BUTTON_1, 0, 85, 75, 26, 0, 0x0, 0 },
  { SLIDER_CreateIndirect, "SliderL", ID_SLIDER_0, 0, 250, 75, 23, 0, 0x0, 0 },
  { SLIDER_CreateIndirect, "SliderH", ID_SLIDER_1, 0, 225, 75, 23, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Launch", ID_BUTTON_2, 0, 0, 75, 40, 0, 0x0, 0 },
  { CHECKBOX_CreateIndirect, "Chess", ID_CHECKBOX_0, 0, 155, 75, 18, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Hz", ID_TEXT_0, 51, 203, 24, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Temp", ID_TEXT_1, 18, 113, 45, 18, 0, 0x0, 0 },
  { SLIDER_CreateIndirect, "SliderFreq", ID_SLIDER_2, 0, 200, 50, 23, 0, 0x0, 0 },
  { SLIDER_CreateIndirect, "SliderScale", ID_SLIDER_3, 0, 175, 50, 23, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Sc", ID_TEXT_2, 53, 178, 24, 20, 0, 0x0, 0 },
  { CHECKBOX_CreateIndirect, "BetterColorRamping", ID_CHECKBOX_1, 0, 133, 75, 18, 0, 0x0, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Window'
    //
    hItem = pMsg->hWin;
    WINDOW_SetBkColor(hItem, GUI_MAKE_COLOR(0x00000000));
    //
    // Initialization of 'Clear'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    BUTTON_SetFont(hItem, GUI_FONT_16B_1);
    //
    // Initialization of 'Save'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
    BUTTON_SetFont(hItem, GUI_FONT_16B_1);
    //
    // Initialization of 'Launch'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_2);
    BUTTON_SetFont(hItem, GUI_FONT_16B_1);
    //
    // Initialization of 'Chess'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_0);
    CHECKBOX_SetText(hItem, "Chess");
    CHECKBOX_SetTextColor(hItem, GUI_MAKE_COLOR(0x00FFFFFF));
    CHECKBOX_SetFont(hItem, GUI_FONT_16B_1);
    //
    // Initialization of 'Hz'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetFont(hItem, GUI_FONT_16B_1);
    TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00FFFFFF));
    //
    // Initialization of 'Temp'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00FFFFFF));
    TEXT_SetFont(hItem, GUI_FONT_16B_1);
    //
    // Initialization of 'Sc'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetFont(hItem, GUI_FONT_16B_1);
    TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00FFFFFF));
    //
    // Initialization of 'BetterColorRamping'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_1);
    CHECKBOX_SetText(hItem, "BetterColor");
    CHECKBOX_SetFont(hItem, GUI_FONT_8_1);
    CHECKBOX_SetTextColor(hItem, GUI_MAKE_COLOR(0x00FFFFFF));
    // USER START (Optionally insert additional code for further widget initialization)
    
    temp_text_inst = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_0);
    CHECKBOX_SetState(hItem, chess_mode);
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_1);
    CHECKBOX_SetState(hItem, better_color_ramping);
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_0);
    SLIDER_SetRange(hItem, -20, 50);
    SLIDER_SetValue(hItem, T_MIN_param);
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_1);
    SLIDER_SetRange(hItem, -20, 50);
    SLIDER_SetValue(hItem, T_MAX_param);
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_2);
    SLIDER_SetRange(hItem, 0, 7);
    SLIDER_SetValue(hItem, freq);
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_3);
    SLIDER_SetRange(hItem, 1, 8);
    SLIDER_SetValue(hItem, (int)(scale_factor * 2));
    
    // USER END
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_BUTTON_0: // Notifications sent by 'Clear'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        
        need_to_clear = 1;
        
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_1: // Notifications sent by 'Save'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        
        need_to_save = 1;
        
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_SLIDER_0: // Notifications sent by 'SliderL'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        
        hItem = WM_GetDialogItem(pMsg->hWin, Id);
        T_MIN_param = SLIDER_GetValue(hItem);
        
        need_to_redraw = 1;
        
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_SLIDER_1: // Notifications sent by 'SliderH'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        
        hItem = WM_GetDialogItem(pMsg->hWin, Id);
        T_MAX_param = SLIDER_GetValue(hItem);
        
        need_to_redraw = 1;
        
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_2: // Notifications sent by 'Launch'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        scan_enabled =! scan_enabled;
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_CHECKBOX_0: // Notifications sent by 'Chess'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        
        hItem = WM_GetDialogItem(pMsg->hWin, Id);
        chess_mode = CHECKBOX_IsChecked(hItem);
        
        need_to_cfg = 1;
        
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_SLIDER_2: // Notifications sent by 'SliderFreq'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        
        hItem = WM_GetDialogItem(pMsg->hWin, Id);
    	  freq = SLIDER_GetValue(hItem);
        
        float r_freq = pow(2.0, (float)freq) / 2.0;
        char str[8] = {0};
        snprintf(str, sizeof(str), "%1.1f", r_freq);
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
        TEXT_SetText(hItem, str);
        
        need_to_cfg = 1;
        
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_SLIDER_3: // Notifications sent by 'SliderScale'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        
        hItem = WM_GetDialogItem(pMsg->hWin, Id);
    	  scale_factor = SLIDER_GetValue(hItem) / 2.0;
        
        char str[8] = {0};
        snprintf(str, sizeof(str), "%1.1f", scale_factor);
        hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
        TEXT_SetText(hItem, str);
        
        need_to_cfg_scale = 1;
        
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_CHECKBOX_1: // Notifications sent by 'BetterColorRamping'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        
        hItem = WM_GetDialogItem(pMsg->hWin, Id);
        better_color_ramping = CHECKBOX_IsChecked(hItem);
        
        need_to_redraw = 1;
        
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateWindow
*/
WM_HWIN CreateWindow(void);
WM_HWIN CreateWindow(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
