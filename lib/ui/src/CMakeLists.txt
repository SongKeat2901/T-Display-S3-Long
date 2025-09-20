SET(SOURCES screens/ui_MainScreen.c
    screens/ui_SettingScreen.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_2089747144.c
    images/ui_img_1544916161.c
    images/ui_img_515815694.c
    images/ui_img_timer_png.c
    images/ui_img_951525100.c
    images/ui_img_weight_png.c
    images/ui_img_1495795081.c
    images/ui_img_80803761.c
    images/ui_img_420894227.c
    images/ui_img_light_png.c
    images/ui_img_return_png.c)

add_library(ui ${SOURCES})
