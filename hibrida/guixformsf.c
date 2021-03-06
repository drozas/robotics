/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "guixformsf.h"

FD_guixformsf *create_form_guixformsf(void)
{
  FL_OBJECT *obj;
  FD_guixformsf *fdui = (FD_guixformsf *) fl_calloc(1, sizeof(*fdui));

  fdui->guixformsf = fl_bgn_form(FL_NO_BOX, 710, 630);
  obj = fl_add_box(FL_UP_BOX,0,0,710,630,"");
    fl_set_object_lcolor(obj,FL_BLUE);
  fdui->exit = obj = fl_add_button(FL_NORMAL_BUTTON,650,10,50,20,"EXIT");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_DARKCYAN,FL_COL1);
    fl_set_object_lcolor(obj,FL_CYAN);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
  fdui->escala = obj = fl_add_valslider(FL_VERT_NICE_SLIDER,20,30,30,150,"scale,m");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_COL1,FL_YELLOW);
    fl_set_object_lcolor(obj,FL_YELLOW);
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_TOP);
    fl_set_object_lstyle(obj,FL_BOLD_STYLE);
  fdui->center = obj = fl_add_positioner(FL_NORMAL_POSITIONER,10,200,50,50,"center");
    fl_set_object_color(obj,FL_DARKER_COL1,FL_YELLOW);
    fl_set_object_lcolor(obj,FL_YELLOW);
    fl_set_object_lalign(obj,FL_ALIGN_TOP_RIGHT);
    fl_set_object_lstyle(obj,FL_BOLD_STYLE);
  fdui->track_robot = obj = fl_add_button(FL_PUSH_BUTTON,10,280,50,30,"track\n robot");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_DARKER_COL1,FL_TOMATO);
    fl_set_object_lcolor(obj,FL_YELLOW);
  fdui->sonars = obj = fl_add_lightbutton(FL_PUSH_BUTTON,500,50,70,20,"sonars");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_GREEN);
    fl_set_object_lcolor(obj,FL_GREEN);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->robot = obj = fl_add_lightbutton(FL_PUSH_BUTTON,500,30,70,20,"robot");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_GREEN);
    fl_set_object_lcolor(obj,FL_GREEN);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->joystick = obj = fl_add_positioner(FL_NORMAL_POSITIONER,530,220,110,50,"");
    fl_set_object_color(obj,FL_DARKER_COL1,FL_DARKTOMATO);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
    fl_set_object_lalign(obj,FL_ALIGN_TOP);
  fdui->teleoperator = obj = fl_add_lightbutton(FL_PUSH_BUTTON,480,200,120,20,"base teleoperator");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_DARKTOMATO);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->ventana0 = obj = fl_add_frame(FL_ENGRAVED_FRAME,10,380,323,243,"Image0");
    fl_set_object_color(obj,FL_COL1,FL_COL1);
  fdui->micanvas = obj = fl_add_canvas(FL_NORMAL_CANVAS,70,10,400,360,"canvas");
    fl_set_object_lcolor(obj,FL_RIGHT_BCOL);
  fdui->visgreyimageA = obj = fl_add_lightbutton(FL_PUSH_BUTTON,630,180,70,20,"greyA");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_lcolor(obj,FL_YELLOW);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->frame_rateA = obj = fl_add_text(FL_NORMAL_TEXT,570,90,50,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->myschema = obj = fl_add_lightbutton(FL_PUSH_BUTTON,480,180,120,20,"myschema");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_DARKTOMATO);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->back = obj = fl_add_button(FL_PUSH_BUTTON,650,250,50,20,"back");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_DARKER_COL1,FL_DARKGOLD);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fdui->stop = obj = fl_add_button(FL_MENU_BUTTON,650,220,50,20,"stop");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_DARKER_COL1,FL_DARKGOLD);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fdui->laser = obj = fl_add_lightbutton(FL_PUSH_BUTTON,500,10,70,20,"laser");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_GREEN);
    fl_set_object_lcolor(obj,FL_GREEN);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->colorimageA = obj = fl_add_lightbutton(FL_PUSH_BUTTON,500,90,70,20,"colorA");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_GREEN);
    fl_set_object_lcolor(obj,FL_GREEN);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->pantilt_joystick = obj = fl_add_positioner(FL_NORMAL_POSITIONER,530,300,110,50,"");
    fl_set_object_color(obj,FL_DARKER_COL1,FL_DARKTOMATO);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
    fl_set_object_lalign(obj,FL_ALIGN_TOP);
  fdui->pantilt_origin = obj = fl_add_button(FL_MENU_BUTTON,650,330,50,20,"origin");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_DARKER_COL1,FL_DARKGOLD);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fdui->pantilt_enable = obj = fl_add_lightbutton(FL_PUSH_BUTTON,480,280,120,20,"pantilt joystick");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_DARKTOMATO);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->pantilt_stop = obj = fl_add_button(FL_MENU_BUTTON,650,300,50,20,"stop");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_DARKER_COL1,FL_DARKGOLD);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fdui->ventana1 = obj = fl_add_frame(FL_ENGRAVED_FRAME,370,380,323,243,"Image1");
    fl_set_object_color(obj,FL_COL1,FL_COL1);
  fdui->frame_rateB = obj = fl_add_text(FL_NORMAL_TEXT,570,110,50,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->visgreyimageB = obj = fl_add_lightbutton(FL_PUSH_BUTTON,630,200,70,20,"greyB");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_lcolor(obj,FL_YELLOW);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->colorimageB = obj = fl_add_lightbutton(FL_PUSH_BUTTON,500,110,70,20,"colorB");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_GREEN);
    fl_set_object_lcolor(obj,FL_GREEN);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->ptspeed = obj = fl_add_slider(FL_VERT_SLIDER,510,300,10,50,"speed");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_LIGHTER_COL1,FL_DARKTOMATO);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
    fl_set_slider_size(obj, 0.00);
  fdui->pantilt_encoders = obj = fl_add_lightbutton(FL_PUSH_BUTTON,500,70,70,20,"pantilt");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_GREEN);
    fl_set_object_lcolor(obj,FL_GREEN);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->guifps = obj = fl_add_text(FL_NORMAL_TEXT,10,320,50,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->fpslaser = obj = fl_add_text(FL_NORMAL_TEXT,570,10,50,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->fpssonars = obj = fl_add_text(FL_NORMAL_TEXT,570,50,50,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->fpsencoders = obj = fl_add_text(FL_NORMAL_TEXT,570,30,50,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->fpspantiltencoders = obj = fl_add_text(FL_NORMAL_TEXT,570,70,50,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->motors = obj = fl_add_lightbutton(FL_PUSH_BUTTON,500,130,70,20,"motors");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_RED);
    fl_set_object_lcolor(obj,FL_RED);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->pantiltmotors = obj = fl_add_lightbutton(FL_PUSH_BUTTON,500,150,70,20,"ptmot");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_COL1,FL_RED);
    fl_set_object_lcolor(obj,FL_RED);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLD_STYLE);
  fdui->fpsmotors = obj = fl_add_text(FL_NORMAL_TEXT,570,130,50,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->fpspantiltmotors = obj = fl_add_text(FL_NORMAL_TEXT,570,150,50,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->vislaser = obj = fl_add_button(FL_PUSH_BUTTON,480,10,20,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_COL1,FL_YELLOW);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fdui->visrobot = obj = fl_add_button(FL_PUSH_BUTTON,480,30,20,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_COL1,FL_YELLOW);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fdui->vissonars = obj = fl_add_button(FL_PUSH_BUTTON,480,50,20,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_COL1,FL_YELLOW);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fdui->vispantiltencoders = obj = fl_add_button(FL_PUSH_BUTTON,480,70,20,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_COL1,FL_YELLOW);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fdui->viscolorimageA = obj = fl_add_button(FL_PUSH_BUTTON,480,90,20,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_COL1,FL_YELLOW);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fdui->viscolorimageB = obj = fl_add_button(FL_PUSH_BUTTON,480,110,20,20,"");
    fl_set_object_boxtype(obj,FL_FRAME_BOX);
    fl_set_object_color(obj,FL_COL1,FL_YELLOW);
    fl_set_object_lcolor(obj,FL_DARKTOMATO);
  fl_end_form();

  fdui->guixformsf->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

