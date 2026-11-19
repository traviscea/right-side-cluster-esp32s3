SET(SOURCES screens/ui_Screen1.c
    fonts/ui_font_status_20.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_black_boot_png.c
    fonts/ui_font_rlabel_32.c
    fonts/ui_font_rvalue_62.c
    fonts/ui_font_rhub_32.c
    fonts/ui_font_rcaption_20.c)

add_library(ui ${SOURCES})
