#pragma once

#include "config.h"

struct UiLayout {
    int status_h;
    int status_text_size;

    int channel_x;
    int channel_y;
    int channel_w;
    int channel_h;
    int channel_radius;
    int channel_label_x;
    int channel_label_y;
    int channel_value_y;
    bool channel_compact_font;

    int info_y;
    int info_h;
    int info_w;
    int info_radius;
    int volume_x;
    int rssi_x;

    int volume_label_x;
    int volume_label_y;
    int volume_value_text_size;
    int volume_value_y;

    int rssi_label_x;
    int rssi_label_y;
    int rssi_value_x;
    int rssi_value_y;
    int rssi_value_text_size;

    int tx_label_x;
    int tx_label_y;
    int tx_value_x;
    int tx_value_y;
    int tx_value_text_size;

    int bar_x;
    int bar_y;
    int bar_w;
    int bar_h;
    int bar_radius;
    int bar_label_x;
    int bar_label_y;
    int bar_start_x;
    int bar_base_y;
    int bar_col_w;
    int bar_col_gap;
    int bar_min_h;
    int bar_step_h;
    int bar_col_radius;
};

#if TALKIE_TARGET_M5ATOMS3_ECHO_BASE
inline constexpr UiLayout kUiLayout = {
    18, 1,
    6, 22, 116, 44, 6, 6, 4, 32, true,
    70, 24, 56, 6, 6, 66,
    6, 3, 2, 4,
    70, 73, 72, 81, 2,
    68, 74, 74, 85, 1,
    6, 98, 116, 24, 6, 10, 100, 18, 119, 8, 4, 3, 2, 2
};
#else
inline constexpr UiLayout kUiLayout = {
    22, 2,
    8, 28, 119, 88, 10, 8, 6, 54, false,
    122, 44, 56, 8, 8, 71,
    8, 8, 3, 8,
    77, 130, 77, 145, 2,
    73, 130, 75, 145, 2,
    8, 170, 119, 62, 8, 12, 173, 13, 224, 11, 3, 6, 5, 3
};
#endif
