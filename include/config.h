// Vendored & trimmed from GoldHEN_Plugins_Repository/plugin_src/gamepad_helper/include/config.h
// Original attribution (per upstream source comment): https://github.com/Teklad/tconfig
// via https://github.com/gimli2/tconfig
// Dropped the ScePadButton/vibration-intensity helpers (not needed here) and
// added a float getter for sensitivity/deadzone values.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct ini_entry_s {
    char* key;
    char* value;
} ini_entry_s;

typedef struct ini_section_s {
    char* name;
    ini_entry_s* entry;
    int size;
} ini_section_s;

typedef struct ini_table_s {
    ini_section_s* section;
    int size;
} ini_table_s;

ini_table_s* ini_table_create(void);
void ini_table_destroy(ini_table_s* table);
bool ini_table_read_from_file(ini_table_s* table, const char* file);
bool ini_table_write_to_file(ini_table_s* table, const char* file);

void ini_table_create_entry(ini_table_s* table, const char* section_name, const char* key,
                             const char* value);
bool ini_table_check_entry(ini_table_s* table, const char* section_name, const char* key);
const char* ini_table_get_entry(ini_table_s* table, const char* section_name, const char* key);

bool ini_table_get_entry_as_int(ini_table_s* table, const char* section_name, const char* key,
                                 int* value);
bool ini_table_get_entry_as_bool(ini_table_s* table, const char* section_name, const char* key,
                                  bool* value);
bool ini_table_get_entry_as_float(ini_table_s* table, const char* section_name, const char* key,
                                   float* value);

ini_section_s* _ini_section_find(ini_table_s* table, const char* name);
