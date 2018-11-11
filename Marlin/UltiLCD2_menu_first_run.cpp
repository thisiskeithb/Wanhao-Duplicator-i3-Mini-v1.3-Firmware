#include "ultralcd.h"
#ifdef ENABLE_ULTILCD2
#include "cardreader.h"//This code uses the card.longFilename as buffer to store data, to save memory.
#include "temperature.h"
#include "stepper.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_menu_material.h"
#include "UltiLCD2_menu_first_run.h"
#include "UltiLCD2_menu_print.h"

#define BED_CENTER_ADJUST_X 92//第一个点x
#define BED_CENTER_ADJUST_Y 10//第一个点y
#define BED_LEFT_ADJUST_X 30//第二个点x
#define BED_LEFT_ADJUST_Y (Y_MAX_LENGTH - 10)//第三个点y
#define BED_RIGHT_ADJUST_X (X_MAX_POS - 48)//第三个点x
#define BED_RIGHT_ADJUST_Y (Y_MAX_LENGTH - 10)//第三个点y
#define BED_CENTER_ADJUST_Z 15 //Z轴下降距离

static void lcd_menu_first_run_init_2();
static void lcd_menu_first_run_init_3();

static void lcd_menu_first_run_bed_level_center_adjust();
static void lcd_menu_first_run_bed_level_left_adjust();
static void lcd_menu_first_run_bed_level_right_adjust();
static void lcd_menu_first_run_bed_level_paper();
static void lcd_menu_first_run_bed_level_paper_center();
static void lcd_menu_first_run_bed_level_paper_left();
static void lcd_menu_first_run_bed_level_paper_right();

static void lcd_menu_first_run_material_load();
static void lcd_menu_first_run_material_load_heatup();
static void lcd_menu_first_run_material_load_insert();
static void lcd_menu_first_run_material_load_forward();
static void lcd_menu_first_run_material_load_wait();

static void lcd_menu_first_run_material_select_1();
static void lcd_menu_first_run_material_select_material();
static void lcd_menu_first_run_material_select_confirm_material();
static void lcd_menu_first_run_material_select_2();

static void lcd_menu_first_run_print_1();
static void lcd_menu_first_run_print_card_detect();

#define DRAW_PROGRESS_NR_IF_NOT_DONE(nr) do { if (!IS_FIRST_RUN_DONE()) { lcd_lib_draw_stringP((nr < 10) ? 100 : 94, 0, PSTR( #nr "/7")); } } while(0)
#define DRAW_PROGRESS_NR(nr) do { lcd_lib_draw_stringP((nr < 10) ? 100 : 94, 0, PSTR( #nr "/21")); } while(0)
#define CLEAR_PROGRESS_NR(nr) do { lcd_lib_clear_stringP((nr < 10) ? 100 : 94, 0, PSTR( #nr "/21")); } while(0)

//Run the first time you start-up the machine or after a factory reset.
void lcd_menu_first_run_init()
{
	SELECT_MAIN_MENU_ITEM(0);
	lcd_info_screen(lcd_menu_first_run_init_2, NULL, PSTR("CONTINUE"));
	DRAW_PROGRESS_NR_IF_NOT_DONE(1);
	lcd_lib_draw_string_centerP(10, PSTR("Welcome to the first"));
	lcd_lib_draw_string_centerP(20, PSTR("startup of your"));
	lcd_lib_draw_string_centerP(30, PSTR("Ultimaker! Press the"));
	lcd_lib_draw_string_centerP(40, PSTR("button to continue"));
	lcd_lib_update_screen();
}
static void homeAndParkHeadForCenterAdjustment2()
{
    //add_homeing[Z_AXIS] = 0;
	home_offset[Z_AXIS] = 0;
	_enqueuecommand("G28 X0 Y0 Z0");
    char buffer[32];
	sprintf(buffer, "G1 F%i Z%i X%i Y%i", int(homing_feedrate_mm_s[0] * 60.0), BED_CENTER_ADJUST_Z, BED_CENTER_ADJUST_X, BED_CENTER_ADJUST_Y);
	_enqueuecommand(buffer);
}
//Started bed leveling from the calibration menu
void lcd_menu_first_run_start_bed_leveling()
{
    lcd_question_screen(lcd_menu_first_run_bed_level_center_adjust, homeAndParkHeadForCenterAdjustment2, PSTR("CONTINUE"), lcd_menu_main, NULL, PSTR("CANCEL"));
    lcd_lib_draw_string_centerP(10, PSTR("I will guide you"));
    lcd_lib_draw_string_centerP(20, PSTR("through the process"));
    lcd_lib_draw_string_centerP(30, PSTR("of adjusting your"));
    lcd_lib_draw_string_centerP(40, PSTR("buildplate."));
    lcd_lib_update_screen();
}

static void homeAndRaiseBed()
{
    _enqueuecommand("G28 Z0");
    char buffer[32];
	sprintf(buffer, "G1 F%i Z%i", int(homing_feedrate_mm_s[0] * 60.0), 35);
    _enqueuecommand(buffer);
}

static void lcd_menu_first_run_init_2()
{
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_init_3, homeAndRaiseBed, PSTR("CONTINUE"));
    //DRAW_PROGRESS_NR_IF_NOT_DONE(2);
    lcd_lib_draw_string_centerP(10, PSTR("Because this is the"));
    lcd_lib_draw_string_centerP(20, PSTR("first startup I will"));
    lcd_lib_draw_string_centerP(30, PSTR("walk you through"));
    lcd_lib_draw_string_centerP(40, PSTR("a first run wizard."));
    lcd_lib_update_screen();
}

static void homeAndParkHeadForCenterAdjustment()
{
    _enqueuecommand("G28 X0 Y0");
    char buffer[32];
	sprintf(buffer, "G1 F%i Z%i X%i Y%i", int(homing_feedrate_mm_s[0] * 60.0), 35, BED_CENTER_ADJUST_X, BED_CENTER_ADJUST_Y);
    _enqueuecommand(buffer);
}

static void lcd_menu_first_run_init_3()
{
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_bed_level_center_adjust, homeAndParkHeadForCenterAdjustment, PSTR("CONTINUE"));
    //DRAW_PROGRESS_NR_IF_NOT_DONE(3);
    lcd_lib_draw_string_centerP(10, PSTR("After transportation"));
    lcd_lib_draw_string_centerP(20, PSTR("we need to do some"));
    lcd_lib_draw_string_centerP(30, PSTR("adjustments, we are"));
    lcd_lib_draw_string_centerP(40, PSTR("going to do that now."));
    lcd_lib_update_screen();
}

static void parkHeadForLeftAdjustment()
{
    //add_homeing[Z_AXIS] -= current_position[Z_AXIS];
	home_offset[Z_AXIS] -= current_position[Z_AXIS];
    current_position[Z_AXIS] = 0;
	planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
	char buffer[32] = { 0 };
    sprintf(buffer, "G1 F%i Z5", int(homing_feedrate_mm_s[Z_AXIS]*60.0));
    _enqueuecommand(buffer);
    sprintf(buffer, "G1 F%i X%i Y%i", int(homing_feedrate_mm_s[X_AXIS]*60.0), BED_LEFT_ADJUST_X, BED_LEFT_ADJUST_Y);
    _enqueuecommand(buffer);
    sprintf(buffer, "G1 F%i Z0", int(homing_feedrate_mm_s[Z_AXIS]*60.0));
    _enqueuecommand(buffer);
}

static void lcd_menu_first_run_bed_level_center_adjust()
{
    LED_GLOW();

    if (lcd_lib_encoder_pos == ENCODER_NO_SELECTION)
        lcd_lib_encoder_pos = 0;

	if (printing_state == PRINT_STATE_NORMAL && lcd_lib_encoder_pos != 0 && planner.movesplanned() < 4)
    {
        current_position[Z_AXIS] -= float(lcd_lib_encoder_pos) * 0.05;
		planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 60, 0);
    }
    lcd_lib_encoder_pos = 0;

	if (planner.movesplanned() > 0)
        lcd_info_screen(NULL, NULL, PSTR("CONTINUE"));
    else
        lcd_info_screen(lcd_menu_first_run_bed_level_left_adjust, parkHeadForLeftAdjustment, PSTR("CONTINUE"));
    //DRAW_PROGRESS_NR_IF_NOT_DONE(4);
	lcd_lib_draw_stringP(100, 0, PSTR("1/7"));
    lcd_lib_draw_string_centerP(10, PSTR("Rotate the button"));
    lcd_lib_draw_string_centerP(20, PSTR("until the nozzle is"));
    lcd_lib_draw_string_centerP(30, PSTR("a millimeter away"));
    lcd_lib_draw_string_centerP(40, PSTR("from the buildplate."));
    lcd_lib_update_screen();
}

static void parkHeadForRightAdjustment()
{
    char buffer[32];
    sprintf(buffer, "G1 F%i Z5", int(homing_feedrate_mm_s[Z_AXIS]*60.0));
    _enqueuecommand(buffer);
    sprintf(buffer, "G1 F%i X%i Y%i", int(homing_feedrate_mm_s[X_AXIS]*60.0), BED_RIGHT_ADJUST_X, BED_RIGHT_ADJUST_Y);
    _enqueuecommand(buffer);
    sprintf(buffer, "G1 F%i Z0", int(homing_feedrate_mm_s[Z_AXIS]*60.0));
    _enqueuecommand(buffer);
}

static void lcd_menu_first_run_bed_level_left_adjust()
{
    LED_GLOW();
    SELECT_MAIN_MENU_ITEM(0);

    lcd_info_screen(lcd_menu_first_run_bed_level_right_adjust, parkHeadForRightAdjustment, PSTR("CONTINUE"));
    //DRAW_PROGRESS_NR_IF_NOT_DONE(5);
	lcd_lib_draw_stringP(100, 0, PSTR("2/7"));
    lcd_lib_draw_string_centerP(10, PSTR("Turn left buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is a millimeter away"));
    lcd_lib_draw_string_centerP(40, PSTR("from the buildplate."));

    lcd_lib_update_screen();
}

static void lcd_menu_first_run_bed_level_right_adjust()
{
    LED_GLOW();
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_bed_level_paper, NULL, PSTR("CONTINUE"));
    //DRAW_PROGRESS_NR_IF_NOT_DONE(6);
	lcd_lib_draw_stringP(100, 0, PSTR("3/7"));
    lcd_lib_draw_string_centerP(10, PSTR("Turn right buildplate"));
    lcd_lib_draw_string_centerP(20, PSTR("screw till the nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("is a millimeter away"));
    lcd_lib_draw_string_centerP(40, PSTR("from the buildplate."));

    lcd_lib_update_screen();
}

static void parkHeadForCenterAdjustment()
{
    char buffer[32];
	sprintf(buffer, "G1 F%i Z5", int(homing_feedrate_mm_s[Z_AXIS] * 60.0));
    _enqueuecommand(buffer);
	sprintf(buffer, "G1 F%i X%i Y%i", int(homing_feedrate_mm_s[X_AXIS] * 60.0), BED_CENTER_ADJUST_X, BED_CENTER_ADJUST_Y);
    _enqueuecommand(buffer);
	sprintf(buffer, "G1 F%i Z0", int(homing_feedrate_mm_s[Z_AXIS] * 60.0));
    _enqueuecommand(buffer);
}

static void lcd_menu_first_run_bed_level_paper()
{
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_bed_level_paper_center, parkHeadForCenterAdjustment, PSTR("CONTINUE"));
    //DRAW_PROGRESS_NR_IF_NOT_DONE(7);
	lcd_lib_draw_stringP(100, 0, PSTR("4/7"));
    lcd_lib_draw_string_centerP(10, PSTR("Repeat this step, but"));
    lcd_lib_draw_string_centerP(20, PSTR("now use a sheet of"));
    lcd_lib_draw_string_centerP(30, PSTR("paper to fine-tune"));
    lcd_lib_draw_string_centerP(40, PSTR("the buildplate level."));
    lcd_lib_update_screen();
}

static void lcd_menu_first_run_bed_level_paper_center()
{
    LED_GLOW();

    if (lcd_lib_encoder_pos == ENCODER_NO_SELECTION)
        lcd_lib_encoder_pos = 0;

	if (printing_state == PRINT_STATE_NORMAL && lcd_lib_encoder_pos != 0 && planner.movesplanned() < 4)
    {
        current_position[Z_AXIS] -= float(lcd_lib_encoder_pos) * 0.05;
        lcd_lib_encoder_pos = 0;
        planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 60, 0);
    }

    if (planner.movesplanned() > 0)
        lcd_info_screen(NULL, NULL, PSTR("CONTINUE"));
    else
        lcd_info_screen(lcd_menu_first_run_bed_level_paper_left, parkHeadForLeftAdjustment, PSTR("CONTINUE"));
    //DRAW_PROGRESS_NR_IF_NOT_DONE(8);
	lcd_lib_draw_stringP(100, 0, PSTR("5/7"));
    lcd_lib_draw_string_centerP(10, PSTR("Slide a paper between"));
    lcd_lib_draw_string_centerP(20, PSTR("buildplate and nozzle"));
    lcd_lib_draw_string_centerP(30, PSTR("until you feel a"));
    lcd_lib_draw_string_centerP(40, PSTR("bit resistance."));
    lcd_lib_update_screen();
}

static void lcd_menu_first_run_bed_level_paper_left()
{
    LED_GLOW();

    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_bed_level_paper_right, parkHeadForRightAdjustment, PSTR("CONTINUE"));
	lcd_lib_draw_stringP(100, 0, PSTR("6/7"));
	//DRAW_PROGRESS_NR_IF_NOT_DONE(9);
    lcd_lib_draw_string_centerP(20, PSTR("Repeat this for"));
    lcd_lib_draw_string_centerP(30, PSTR("the left corner..."));
    lcd_lib_update_screen();
}

static void homeBed()
{
    //add_homeing[Z_AXIS] += LEVELING_OFFSET;  //Adjust the Z homing position to account for the thickness of the paper.
	//home_offset[Z_AXIS] += LEVELING_OFFSET;
    // now that we are finished, save the settings to EEPROM
    //Config_StoreSettings();
    _enqueuecommand("G28 X0 Y0 Z0");
	_enqueuecommand("G1 Z35");
}

static void lcd_menu_first_run_bed_level_paper_right()
{
    LED_GLOW();

    SELECT_MAIN_MENU_ITEM(0);
    //if (IS_FIRST_RUN_DONE())
    if(1)
        lcd_info_screen(lcd_menu_main, homeBed, PSTR("DONE"));
    else
        lcd_info_screen(lcd_menu_first_run_material_load, homeBed, PSTR("CONTINUE"));
    //DRAW_PROGRESS_NR_IF_NOT_DONE(10);
	lcd_lib_draw_stringP(100, 0, PSTR("7/7"));
	lcd_lib_draw_string_centerP(20, PSTR("Repeat this for"));
    lcd_lib_draw_string_centerP(30, PSTR("the right corner..."));
    lcd_lib_update_screen();
}

static void parkHeadForHeating()
{
    lcd_material_reset_defaults();
    _enqueuecommand("G1 F12000 X110 Y10");
    _enqueuecommand("M84");//Disable motor power.
}

static void lcd_menu_first_run_material_load()
{
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_material_load_heatup, parkHeadForHeating, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR(11);
    lcd_lib_draw_string_centerP(10, PSTR("Now that we leveled"));
    lcd_lib_draw_string_centerP(20, PSTR("the buildplate"));
    lcd_lib_draw_string_centerP(30, PSTR("the next step is"));
    lcd_lib_draw_string_centerP(40, PSTR("to insert material."));
    lcd_lib_update_screen();
}

static void lcd_menu_first_run_material_load_heatup()
{
    Temperature::setTargetHotend(230, 0);
	int16_t temp = Temperature::degHotend(0) - 20;
	int16_t target = Temperature::degTargetHotend(0) - 10 - 20;
    if (temp < 0) temp = 0;
    if (temp > target)
    {
        //for(uint8_t e=0; e<EXTRUDERS; e++)
            //volume_to_filament_length[e] = 1.0;//Set the extrusion to 1mm per given value, so we can move the filament a set distance.

        currentMenu = lcd_menu_first_run_material_load_insert;
        temp = target;
    }

    uint8_t progress = uint8_t(temp * 125 / target);
    if (progress < minProgress)
        progress = minProgress;
    else
        minProgress = progress;

    lcd_basic_screen();
    DRAW_PROGRESS_NR(12);
    lcd_lib_draw_string_centerP(10, PSTR("Please wait,"));
    lcd_lib_draw_string_centerP(20, PSTR("printhead heating for"));
    lcd_lib_draw_string_centerP(30, PSTR("material loading"));

    lcd_progressbar(progress);

    lcd_lib_update_screen();
}

static void runMaterialForward()
{
    //Override the max feedrate and acceleration values to get a better insert speed and speedup/slowdown
	float old_max_feedrate_e = planner.max_feedrate_mm_s[E_AXIS];
    float old_retract_acceleration = planner.retract_acceleration;
	planner.max_feedrate_mm_s[E_AXIS] = FILAMENT_INSERT_FAST_SPEED;
	planner.retract_acceleration = FILAMENT_LONG_MOVE_ACCELERATION;

    current_position[E_AXIS] = 0;
	planner.set_e_position_mm(current_position[E_AXIS]);
    current_position[E_AXIS] = FILAMENT_FORWARD_LENGTH;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_FAST_SPEED, 0);

    //Put back origonal values.
	planner.max_feedrate_mm_s[E_AXIS] = old_max_feedrate_e;
	planner.retract_acceleration = old_retract_acceleration;
}

static void lcd_menu_first_run_material_load_insert()
{
    LED_GLOW();

    if (planner.movesplanned() < 2)
    {
        current_position[E_AXIS] += 0.5;
        planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_SPEED, 0);
    }

    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_material_load_forward, runMaterialForward, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR(13);
    lcd_lib_draw_string_centerP(10, PSTR("Insert new material"));
    lcd_lib_draw_string_centerP(20, PSTR("from the rear of"));
    lcd_lib_draw_string_centerP(30, PSTR("your Ultimaker2,"));
    lcd_lib_draw_string_centerP(40, PSTR("above the arrow."));
    lcd_lib_update_screen();
}

static void lcd_menu_first_run_material_load_forward()
{
    lcd_basic_screen();
    DRAW_PROGRESS_NR(14);
    lcd_lib_draw_string_centerP(20, PSTR("Loading material..."));

    if (!planner.blocks_queued())
    {
        lcd_lib_beep();
        led_glow_dir = led_glow = 0;
#if HAS_MOTOR_CURRENT_PWM
		stepper.digipot_current(2, Stepper::motor_current_setting[2] * 2 / 3);//Set E motor power lower so the motor will skip instead of grind.
#endif
        currentMenu = lcd_menu_first_run_material_load_wait;
        SELECT_MAIN_MENU_ITEM(0);
    }

	long pos = stepper.get_axis_position_mm(E_AXIS);
	long targetPos = lround(FILAMENT_FORWARD_LENGTH*planner.axis_steps_per_mm[E_AXIS]);
    uint8_t progress = (pos * 125 / targetPos);
    lcd_progressbar(progress);

    lcd_lib_update_screen();
}

static void lcd_menu_first_run_material_load_wait()
{
    LED_GLOW();

    lcd_info_screen(lcd_menu_first_run_material_select_1, doCooldown, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR(15);
    lcd_lib_draw_string_centerP(10, PSTR("Push button when"));
    lcd_lib_draw_string_centerP(20, PSTR("material exits"));
    lcd_lib_draw_string_centerP(30, PSTR("from nozzle..."));

    if (planner.movesplanned() < 2)
    {
        current_position[E_AXIS] += 0.5;
        planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_EXTRUDE_SPEED, 0);
    }

    lcd_lib_update_screen();
}

static void lcd_menu_first_run_material_select_1()
{
    if (eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET()) == 1)
    {
#if HAS_MOTOR_CURRENT_PWM
		stepper.digipot_current(2, Stepper::motor_current_setting[2]);//Set E motor power to default.
#endif
        for(uint8_t e=0; e<EXTRUDERS; e++)
            lcd_material_set_material(0, e);
        SET_FIRST_RUN_DONE();
        
        currentMenu = lcd_menu_first_run_print_1;
        return;
    }
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_material_select_material, doCooldown, PSTR("READY"));
    DRAW_PROGRESS_NR(16);
    lcd_lib_draw_string_centerP(10, PSTR("Next, select the"));
    lcd_lib_draw_string_centerP(20, PSTR("material you have"));
    lcd_lib_draw_string_centerP(30, PSTR("inserted in this"));
    lcd_lib_draw_string_centerP(40, PSTR("Ultimaker2."));
    lcd_lib_update_screen();
}

static char* lcd_material_select_callback(uint8_t nr)
{
    eeprom_read_block(card.longFilename, EEPROM_MATERIAL_NAME_OFFSET(nr), 8);
    return card.longFilename;
}

static void lcd_material_select_details_callback(uint8_t nr)
{
    lcd_lib_draw_stringP(5, 53, PSTR("Select the material"));
}

static void lcd_menu_first_run_material_select_material()
{
    LED_GLOW();
    uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());

    lcd_scroll_menu(PSTR("MATERIAL"), count, lcd_material_select_callback, lcd_material_select_details_callback);
    CLEAR_PROGRESS_NR(17);
    lcd_lib_update_screen();

    if (lcd_lib_button_pressed)
    {
#if HAS_MOTOR_CURRENT_PWM
		stepper.digipot_current(2, Stepper::motor_current_setting[2]);//Set E motor power to default.
#endif
        for(uint8_t e=0; e<EXTRUDERS; e++)
            lcd_material_set_material(SELECTED_SCROLL_MENU_ITEM(), e);
        SET_FIRST_RUN_DONE();
        lcd_change_to_menu(lcd_menu_first_run_material_select_confirm_material);
        strcat_P(card.longFilename, PSTR(" as material,"));
    }
}

static void lcd_menu_first_run_material_select_confirm_material()
{
    LED_GLOW();
    lcd_question_screen(lcd_menu_first_run_material_select_2, NULL, PSTR("YES"), lcd_menu_first_run_material_select_material, NULL, PSTR("NO"));
    DRAW_PROGRESS_NR(18);
    lcd_lib_draw_string_centerP(20, PSTR("You have chosen"));
    lcd_lib_draw_string_center(30, card.longFilename);
    lcd_lib_draw_string_centerP(40, PSTR("is this right?"));
    lcd_lib_update_screen();
}

static void lcd_menu_first_run_material_select_2()
{
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_print_1, NULL, PSTR("CONTINUE"));
    DRAW_PROGRESS_NR(19);
    lcd_lib_draw_string_centerP(10, PSTR("Now your Ultimaker2"));
    lcd_lib_draw_string_centerP(20, PSTR("knows what kind"));
    lcd_lib_draw_string_centerP(30, PSTR("of material"));
    lcd_lib_draw_string_centerP(40, PSTR("it is using."));
    lcd_lib_update_screen();
}

static void lcd_menu_first_run_print_1()
{
    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_first_run_print_card_detect, NULL, PSTR("ARE YOU READY?"));
    DRAW_PROGRESS_NR(20);
    lcd_lib_draw_string_centerP(20, PSTR("I'm ready let's"));
    lcd_lib_draw_string_centerP(30, PSTR("make a 3D Print!"));
    lcd_lib_update_screen();
}

static void lcd_menu_first_run_print_card_detect()
{
    if (!card.cardOK)
    {
        lcd_info_screen(lcd_menu_main);
        DRAW_PROGRESS_NR(21);
        lcd_lib_draw_string_centerP(30, PSTR("Reading card..."));
        lcd_lib_update_screen();
        card.initsd();
        return;
    }

    SELECT_MAIN_MENU_ITEM(0);
    lcd_info_screen(lcd_menu_print_select, NULL, PSTR("LET'S PRINT"));
    DRAW_PROGRESS_NR(21);
    lcd_lib_draw_string_centerP(10, PSTR("Select a print file"));
    lcd_lib_draw_string_centerP(20, PSTR("on the SD-card"));
    lcd_lib_draw_string_centerP(30, PSTR("and press the button"));
    lcd_lib_draw_string_centerP(40, PSTR("to print it!"));
    lcd_lib_update_screen();
}
#endif//ENABLE_ULTILCD2
