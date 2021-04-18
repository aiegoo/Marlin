#ifndef LANGUAGE_H
#define LANGUAGE_H

#define LANGUAGE_CONCAT(M)       #M
#define GENERATE_LANGUAGE_INCLUDE(M)  LANGUAGE_CONCAT(language_##M.h)


// NOTE: IF YOU CHANGE LANGUAGE FILES OR MERGE A FILE WITH CHANGES
//
//   ==> ALWAYS TRY TO COMPILE MARLIN WITH/WITHOUT "ULTIPANEL" / "ULTRALCD" / "SDSUPPORT" #define IN "Configuration.h"
//   ==> ALSO TRY ALL AVAILABLE LANGUAGE OPTIONS
<<<<<<< HEAD

// Languages
// en English
// pl Polish
// fr French
// de German
// es Spanish
// ru Russian
// it Italian
// pt Portuguese
// fi Finnish
// an Aragonese
// nl Dutch
// ca Catalan
// eu Basque-Euskera
// cz Czech

#ifndef LANGUAGE_INCLUDE
  // pick your language from the list above
  #define LANGUAGE_INCLUDE GENERATE_LANGUAGE_INCLUDE(en)
#endif
=======
// See also http://marlinfw.org/docs/development/lcd_language.html

// Languages
// an         Aragonese
// bg         Bulgarian
// ca         Catalan
// cn         Chinese
// cz         Czech
// cz_utf8    Czech (UTF8)
// de         German
// el         Greek
// el-gr      Greek (Greece)
// en         English
// es         Spanish
// es_utf8    Spanish (UTF8)
// eu         Basque-Euskera
// fi         Finnish
// fr         French
// fr_utf8    French (UTF8)
// gl         Galician
// hr         Croatian
// it         Italian
// kana       Japanese
// kana_utf8  Japanese (UTF8)
// ko_kr      Korean
// nl         Dutch
// pl         Polish
// pl_utf8    Polish (UTF8)
// pt         Portuguese
// pt-br      Portuguese (Brazilian)
// pt-br_utf8 Portuguese (Brazilian) (UTF8)
// pt_utf8    Portuguese (UTF8)
// ru         Russian
// sk         Slovak (UTF8)
// tr         Turkish
// uk         Ukrainian
// zh_CN      Chinese (Simplified)
// zh_TW      Chinese (Taiwan)
>>>>>>> 1314b31d97bba8cd74c6625c47176d4692f57790

#if defined(USE_AUTOMATIC_VERSIONING)
  #include "_Version.h"
#else
  #include "Default_Version.h"
#endif

#define PROTOCOL_VERSION "1.0"

#if MB(ULTIMAKER)|| MB(ULTIMAKER_OLD)|| MB(ULTIMAIN_2)
  #define MACHINE_NAME "Ultimaker"
  #define FIRMWARE_URL "https://github.com/Ultimaker/Marlin"
#elif MB(RUMBA)
  #define MACHINE_NAME "Rumba"
  #define FIRMWARE_URL "https://github.com/MarlinFirmware/Marlin"
#elif MB(3DRAG)
  #define MACHINE_NAME "3Drag"
  #define FIRMWARE_URL "http://3dprint.elettronicain.it/"
#elif MB(5DPRINT)
  #define MACHINE_NAME "Makibox"
  #define FIRMWARE_URL "https://github.com/MarlinFirmware/Marlin"
#elif MB(SAV_MKI)
  #define MACHINE_NAME "SAV MkI"
  #define FIRMWARE_URL "https://github.com/fmalpartida/Marlin/tree/SAV-MkI-config"
#else
  #ifdef CUSTOM_MENDEL_NAME
    #define MACHINE_NAME CUSTOM_MENDEL_NAME
  #else
    #define MACHINE_NAME "Mendel"
  #endif

// Default firmware set to Mendel
  #define FIRMWARE_URL "https://github.com/MarlinFirmware/Marlin"
#endif

#ifdef CUSTOM_MACHINE_NAME
  #undef MACHINE_NAME
  #define MACHINE_NAME CUSTOM_MACHINE_NAME
#endif

#ifndef SOURCE_CODE_URL
  #define SOURCE_CODE_URL "https://github.com/MarlinFirmware/Marlin"
#endif

#ifndef DETAILED_BUILD_VERSION
  #error BUILD_VERSION Information must be specified
#endif

#ifndef MACHINE_UUID
   #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"
#endif


#define STRINGIFY_(n) #n
#define STRINGIFY(n) STRINGIFY_(n)


// Common LCD messages

  /* nothing here yet */

// Common serial messages
#define MSG_MARLIN "Marlin"

// Serial Console Messages (do not translate those!)

#define MSG_Enqueing                        "enqueing \""
#define MSG_POWERUP                         "PowerUp"
#define MSG_EXTERNAL_RESET                  " External Reset"
#define MSG_BROWNOUT_RESET                  " Brown out Reset"
#define MSG_WATCHDOG_RESET                  " Watchdog Reset"
#define MSG_SOFTWARE_RESET                  " Software Reset"
#define MSG_AUTHOR                          " | Author: "
#define MSG_CONFIGURATION_VER               " Last Updated: "
#define MSG_FREE_MEMORY                     " Free Memory: "
#define MSG_PLANNER_BUFFER_BYTES            "  PlannerBufferBytes: "
#define MSG_OK                              "ok"
#define MSG_FILE_SAVED                      "Done saving file."
#define MSG_ERR_LINE_NO                     "Line Number is not Last Line Number+1, Last Line: "
#define MSG_ERR_CHECKSUM_MISMATCH           "checksum mismatch, Last Line: "
#define MSG_ERR_NO_CHECKSUM                 "No Checksum with line number, Last Line: "
#define MSG_FILE_PRINTED                    "Done printing file"
#define MSG_BEGIN_FILE_LIST                 "Begin file list"
#define MSG_END_FILE_LIST                   "End file list"
#define MSG_M104_INVALID_EXTRUDER           "M104 Invalid extruder "
#define MSG_M105_INVALID_EXTRUDER           "M105 Invalid extruder "
#define MSG_M200_INVALID_EXTRUDER           "M200 Invalid extruder "
#define MSG_M218_INVALID_EXTRUDER           "M218 Invalid extruder "
#define MSG_M221_INVALID_EXTRUDER           "M221 Invalid extruder "
#define MSG_ERR_NO_THERMISTORS              "No thermistors - no temperature"
#define MSG_M109_INVALID_EXTRUDER           "M109 Invalid extruder "
#define MSG_HEATING                         "Heating..."
#define MSG_HEATING_COMPLETE                "Heating done."
#define MSG_BED_HEATING                     "Bed Heating."
#define MSG_BED_DONE                        "Bed done."
#define MSG_M115_REPORT                     "FIRMWARE_NAME:Marlin " DETAILED_BUILD_VERSION " SOURCE_CODE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:" MACHINE_NAME " EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#define MSG_COUNT_X                         " Count X: "
#define MSG_ERR_KILLED                      "Printer halted. kill() called!"
#define MSG_ERR_STOPPED                     "Printer stopped due to errors. Fix the error and use M999 to restart. (Temperature is reset. Set it after restarting)"
<<<<<<< HEAD
#define MSG_RESEND                          "Resend: "
#define MSG_UNKNOWN_COMMAND                 "Unknown command: \""
#define MSG_ACTIVE_EXTRUDER                 "Active Extruder: "
#define MSG_INVALID_EXTRUDER                "Invalid extruder"
#define MSG_X_MIN                           "x_min: "
#define MSG_X_MAX                           "x_max: "
#define MSG_Y_MIN                           "y_min: "
#define MSG_Y_MAX                           "y_max: "
#define MSG_Z_MIN                           "z_min: "
#define MSG_Z_MAX                           "z_max: "
=======
#define MSG_BUSY_PROCESSING                 "busy: processing"
#define MSG_BUSY_PAUSED_FOR_USER            "busy: paused for user"
#define MSG_BUSY_PAUSED_FOR_INPUT           "busy: paused for input"
#define MSG_Z_MOVE_COMP                     "Z_move_comp"
#define MSG_RESEND                          "Resend: "
#define MSG_UNKNOWN_COMMAND                 "Unknown command: \""
#define MSG_ACTIVE_EXTRUDER                 "Active Extruder: "
#define MSG_X_MIN                           "x_min"
#define MSG_X_MAX                           "x_max"
#define MSG_X2_MIN                          "x2_min"
#define MSG_X2_MAX                          "x2_max"
#define MSG_Y_MIN                           "y_min"
#define MSG_Y_MAX                           "y_max"
#define MSG_Y2_MIN                          "y2_min"
#define MSG_Y2_MAX                          "y2_max"
#define MSG_Z_MIN                           "z_min"
#define MSG_Z_MAX                           "z_max"
#define MSG_Z2_MIN                          "z2_min"
#define MSG_Z2_MAX                          "z2_max"
#define MSG_Z_PROBE                         "z_probe"
#define MSG_FILAMENT_RUNOUT_SENSOR          "filament"
#define MSG_PROBE_Z_OFFSET                  "Probe Z Offset"
#define MSG_SKEW_MIN                        "min_skew_factor: "
#define MSG_SKEW_MAX                        "max_skew_factor: "
#define MSG_ERR_MATERIAL_INDEX              "M145 S<index> out of range (0-1)"
#define MSG_ERR_M355_NONE                   "No case light"
#define MSG_ERR_M421_PARAMETERS             "M421 incorrect parameter usage"
#define MSG_ERR_BAD_PLANE_MODE              "G5 requires XY plane mode"
#define MSG_ERR_MESH_XY                     "Mesh point cannot be resolved"
#define MSG_ERR_ARC_ARGS                    "G2/G3 bad parameters"
#define MSG_ERR_PROTECTED_PIN               "Protected Pin"
#define MSG_ERR_M420_FAILED                 "Failed to enable Bed Leveling"
#define MSG_ERR_M428_TOO_FAR                "Too far from reference point"
#define MSG_ERR_M303_DISABLED               "PIDTEMP disabled"
>>>>>>> 1314b31d97bba8cd74c6625c47176d4692f57790
#define MSG_M119_REPORT                     "Reporting endstop status"
#define MSG_ENDSTOP_HIT                     "TRIGGERED"
#define MSG_ENDSTOP_OPEN                    "open"
#define MSG_HOTEND_OFFSET                   "Hotend offsets:"

#define MSG_SD_CANT_OPEN_SUBDIR             "Cannot open subdir"
#define MSG_SD_INIT_FAIL                    "SD init fail"
#define MSG_SD_VOL_INIT_FAIL                "volume.init failed"
#define MSG_SD_OPENROOT_FAIL                "openRoot failed"
#define MSG_SD_CARD_OK                      "SD card ok"
#define MSG_SD_WORKDIR_FAIL                 "workDir open failed"
#define MSG_SD_OPEN_FILE_FAIL               "open failed, File: "
#define MSG_SD_FILE_OPENED                  "File opened: "
#define MSG_SD_SIZE                         " Size: "
#define MSG_SD_FILE_SELECTED                "File selected"
#define MSG_SD_WRITE_TO_FILE                "Writing to file: "
#define MSG_SD_PRINTING_BYTE                "SD printing byte "
#define MSG_SD_NOT_PRINTING                 "Not SD printing"
#define MSG_SD_ERR_WRITE_TO_FILE            "error writing to file"
#define MSG_SD_CANT_ENTER_SUBDIR            "Cannot enter subdir: "

#define MSG_STEPPER_TOO_HIGH                "Steprate too high: "
#define MSG_ENDSTOPS_HIT                    "endstops hit: "
#define MSG_ERR_COLD_EXTRUDE_STOP           " cold extrusion prevented"
#define MSG_ERR_LONG_EXTRUDE_STOP           " too long extrusion prevented"
<<<<<<< HEAD
#define MSG_BABYSTEPPING_X                  "Babystepping X"
#define MSG_BABYSTEPPING_Y                  "Babystepping Y"
#define MSG_BABYSTEPPING_Z                  "Babystepping Z"
#define MSG_SERIAL_ERROR_MENU_STRUCTURE     "Error in menu structure"

// LCD Menu Messages

#include LANGUAGE_INCLUDE
=======
#define MSG_HOTEND_TOO_COLD                 "Hotend too cold"

#define MSG_FILAMENT_CHANGE_HEAT            "Press button (or M108) to heat nozzle"
#define MSG_FILAMENT_CHANGE_INSERT          "Insert filament and press button (or M108)"
#define MSG_FILAMENT_CHANGE_HEAT_LCD        "Press button to heat nozzle"
#define MSG_FILAMENT_CHANGE_INSERT_LCD      "Insert filament and press button"
#define MSG_FILAMENT_CHANGE_HEAT_M108       "Send M108 to heat nozzle"
#define MSG_FILAMENT_CHANGE_INSERT_M108     "Insert filament and send M108"

#define MSG_ERR_EEPROM_WRITE                "Error writing to EEPROM!"

#define MSG_STOP_BLTOUCH                    "STOP called because of BLTouch error - restart with M999"
#define MSG_STOP_UNHOMED                    "STOP called because of unhomed error - restart with M999"
#define MSG_KILL_INACTIVE_TIME              "KILL caused by too much inactive time - current command: "
#define MSG_KILL_BUTTON                     "KILL caused by KILL button/pin"

// temperature.cpp strings
#define MSG_PID_AUTOTUNE                    "PID Autotune"
#define MSG_PID_AUTOTUNE_START              MSG_PID_AUTOTUNE " start"
#define MSG_PID_AUTOTUNE_FAILED             MSG_PID_AUTOTUNE " failed!"
#define MSG_PID_BAD_EXTRUDER_NUM            MSG_PID_AUTOTUNE_FAILED " Bad extruder number"
#define MSG_PID_TEMP_TOO_HIGH               MSG_PID_AUTOTUNE_FAILED " Temperature too high"
#define MSG_PID_TIMEOUT                     MSG_PID_AUTOTUNE_FAILED " timeout"
#define MSG_BIAS                            " bias: "
#define MSG_D                               " d: "
#define MSG_T_MIN                           " min: "
#define MSG_T_MAX                           " max: "
#define MSG_KU                              " Ku: "
#define MSG_TU                              " Tu: "
#define MSG_CLASSIC_PID                     " Classic PID "
#define MSG_KP                              " Kp: "
#define MSG_KI                              " Ki: "
#define MSG_KD                              " Kd: "
#define MSG_AT                              " @:"
#define MSG_PID_AUTOTUNE_FINISHED           MSG_PID_AUTOTUNE " finished! Put the last Kp, Ki and Kd constants from below into Configuration.h"
#define MSG_PID_DEBUG                       " PID_DEBUG "
#define MSG_PID_DEBUG_INPUT                 ": Input "
#define MSG_PID_DEBUG_OUTPUT                " Output "
#define MSG_PID_DEBUG_PTERM                 " pTerm "
#define MSG_PID_DEBUG_ITERM                 " iTerm "
#define MSG_PID_DEBUG_DTERM                 " dTerm "
#define MSG_PID_DEBUG_CTERM                 " cTerm "
#define MSG_INVALID_EXTRUDER_NUM            " - Invalid extruder number !"

#define MSG_HEATER_BED                      "bed"
#define MSG_STOPPED_HEATER                  ", system stopped! Heater_ID: "
#define MSG_REDUNDANCY                      "Heater switched off. Temperature difference between temp sensors is too high !"
#define MSG_T_HEATING_FAILED                "Heating failed"
#define MSG_T_THERMAL_RUNAWAY               "Thermal Runaway"
#define MSG_T_MAXTEMP                       "MAXTEMP triggered"
#define MSG_T_MINTEMP                       "MINTEMP triggered"

// Debug
#define MSG_DEBUG_PREFIX                    "DEBUG:"
#define MSG_DEBUG_OFF                       "off"
#define MSG_DEBUG_ECHO                      "ECHO"
#define MSG_DEBUG_INFO                      "INFO"
#define MSG_DEBUG_ERRORS                    "ERRORS"
#define MSG_DEBUG_DRYRUN                    "DRYRUN"
#define MSG_DEBUG_COMMUNICATION             "COMMUNICATION"
#define MSG_DEBUG_LEVELING                  "LEVELING"

// LCD Menu Messages

#define LANGUAGE_INCL_(M) STRINGIFY_(language_##M.h)
#define LANGUAGE_INCL(M) LANGUAGE_INCL_(M)
#define INCLUDE_LANGUAGE LANGUAGE_INCL(LCD_LANGUAGE)

// Never translate these strings
#define MSG_X "X"
#define MSG_Y "Y"
#define MSG_Z "Z"
#define MSG_E "E"
#if IS_KINEMATIC
  #define MSG_A "A"
  #define MSG_B "B"
  #define MSG_C "C"
#else
  #define MSG_A "X"
  #define MSG_B "Y"
  #define MSG_C "Z"
#endif
#define MSG_H1 "1"
#define MSG_H2 "2"
#define MSG_H3 "3"
#define MSG_H4 "4"
#define MSG_H5 "5"
#define MSG_N1 " 1"
#define MSG_N2 " 2"
#define MSG_N3 " 3"
#define MSG_N4 " 4"
#define MSG_N5 " 5"
#define MSG_E1 "E1"
#define MSG_E2 "E2"
#define MSG_E3 "E3"
#define MSG_E4 "E4"
#define MSG_E5 "E5"
#define MSG_MOVE_E1 "1"
#define MSG_MOVE_E2 "2"
#define MSG_MOVE_E3 "3"
#define MSG_MOVE_E4 "4"
#define MSG_MOVE_E5 "5"
#define MSG_DIAM_E1 " 1"
#define MSG_DIAM_E2 " 2"
#define MSG_DIAM_E3 " 3"
#define MSG_DIAM_E4 " 4"
#define MSG_DIAM_E5 " 5"

#include INCLUDE_LANGUAGE

#if DISABLED(SIMULATE_ROMFONT) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_1) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_5) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_KANA) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_KO_KR) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_GREEK) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_CN) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_TR) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_PL) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_CZ) \
 && DISABLED(DISPLAY_CHARSET_ISO10646_SK)
  #define DISPLAY_CHARSET_ISO10646_1 // use the better font on full graphic displays.
#endif

#include "language_en.h"
>>>>>>> 1314b31d97bba8cd74c6625c47176d4692f57790

#endif //__LANGUAGE_H
