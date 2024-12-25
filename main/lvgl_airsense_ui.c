
#include "stdio.h"
#include "lvgl.h"
char buffer[32]; // buffer for sprintf

struct dataSensor_st
{
    uint64_t timeStamp;

    float temperature;
    float humidity;
    float pressure;

    uint32_t pm1_0;
    uint32_t pm2_5;
    uint32_t pm10;

    uint32_t CO2;
};

lv_obj_t *scr;
lv_obj_t *wifi_icon;
lv_obj_t *sdcard_icon;
lv_obj_t *wifi_off;
lv_obj_t *sdcard_off;
lv_obj_t *aqi_label;
lv_obj_t *temp_label;
lv_obj_t *humd_label;
lv_obj_t *press_label;
lv_obj_t *pm25_label;
lv_obj_t *pm10_label;
lv_obj_t *co2_label;
lv_obj_t *aqi_value_label;
lv_obj_t *temp_value_label;
lv_obj_t *humd_value_label;
lv_obj_t *press_value_label;
lv_obj_t *pm25_value_label;
lv_obj_t *pm10_value_label;
lv_obj_t *co2_value_label;
lv_obj_t *line_above;
lv_obj_t *title_lable;
lv_obj_t *title_lable2;
lv_obj_t *line_bellow;

lv_obj_t *date_time_label;
lv_obj_t *error_code_label;

lv_style_t text_style;
lv_style_t data_style;

void airsense_wifi_state_display(bool wifi_state)
{
    if (wifi_state == true)
    {
        lv_obj_set_style_text_color(wifi_icon, lv_color_hex(0x00cc00), LV_PART_MAIN);
    }
    else
    {
        lv_obj_set_style_text_color(wifi_icon, lv_color_hex(0xd92626), LV_PART_MAIN);
    }
    lv_timer_handler();
}

void airsense_sdcard_state_display(bool sdcard_state)
{
    if (sdcard_state == true)
    {
        lv_obj_set_style_text_color(sdcard_icon, lv_color_hex(0x00cc00), LV_PART_MAIN);
    }
    else
    {
        lv_obj_set_style_text_color(sdcard_icon, lv_color_hex(0xd92626), LV_PART_MAIN);
    }
    lv_timer_handler();
}

void airsense_date_time_display(char *date_time_value)
{
    lv_label_set_text(date_time_label, date_time_value);
    lv_timer_handler();
}

// void error_code_display()
// {
//     sprintf(buffer, "%s", error_code_value);
//     lv_label_set_text(error_code_label, buffer);
// }

void airsense_sensor_data_display(struct dataSensor_st sensor_data)
{
    // Giá trị AQI
    /*    sprintf(buffer, "%.1f", aqi_data);
       lv_label_set_text(aqi_value_label, buffer);
       lv_obj_align_to(aqi_value_label, aqi_label, LV_ALIGN_CENTER, 60, 0); */

    // Giá trị nhiệt độ
    sprintf(buffer, "%.1f°C", sensor_data.temperature);
    lv_label_set_text(temp_value_label, buffer);
    lv_obj_align_to(temp_value_label, humd_label, LV_ALIGN_CENTER, 5, -25);

    // Giá trị độ ẩm
    sprintf(buffer, "%.0f%%", sensor_data.humidity);
    lv_label_set_text(humd_value_label, buffer);
    lv_obj_align_to(humd_value_label, humd_label, LV_ALIGN_CENTER, 5, 25);

    // Giá trị áp suất
    sprintf(buffer, "%.0fhPa", (sensor_data.pressure / 100));
    lv_label_set_text(press_value_label, buffer);
    lv_obj_align_to(press_value_label, press_label, LV_ALIGN_CENTER, 0, 25);

    // Giá trị PM2.5
    sprintf(buffer, "%ldug/m3", sensor_data.pm2_5);
    lv_label_set_text(pm25_value_label, buffer);
    lv_obj_align_to(pm25_value_label, pm10_label, LV_ALIGN_CENTER, 0, -25);

    // Giá trị PM10
    sprintf(buffer, "%ldug/m3", sensor_data.pm10);
    lv_label_set_text(pm10_value_label, buffer);
    lv_obj_align_to(pm10_value_label, pm10_label, LV_ALIGN_CENTER, 0, 25);

    // Giá trị CO2
    sprintf(buffer, "%ldppm", sensor_data.CO2);
    lv_label_set_text(co2_value_label, buffer);
    lv_obj_align_to(co2_value_label, co2_label, LV_ALIGN_CENTER, 0, 25);

    lv_timer_handler();
}

// Hàm tạo giao diện
void ui_airsense_init(lv_disp_t *disp)
{

    static lv_style_t style_base;
    lv_style_init(&style_base);
    lv_style_set_border_width(&style_base, LV_PART_MAIN); // đặt độ rộng viền

    scr = lv_disp_get_scr_act(disp);                                // lấy màn hình hiện tại
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN); // Đặt nền màu đen
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);       // Đảm bảo nền được phủ đầy hoàn toàn
    lv_obj_center(scr);
    lv_obj_add_style(scr, &style_base, LV_PART_MAIN); // Thêm style
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);   // Xóa cờ scrollable

    // Tạo style cho text
    lv_style_init(&text_style);
    lv_style_set_text_color(&text_style, lv_color_hex(0xECAF15));
    lv_style_set_text_font(&text_style, &lv_font_montserrat_16);

    // Tạo style cho dữ liệu
    lv_style_init(&data_style);
    lv_style_set_text_color(&data_style, lv_color_hex(0x68EC15));
    lv_style_set_text_font(&data_style, &lv_font_montserrat_16);

    wifi_icon = lv_label_create(scr);
    lv_obj_set_width(wifi_icon, disp->driver->hor_res);
    lv_obj_align(wifi_icon, LV_ALIGN_TOP_LEFT, 3, 4);
    lv_label_set_text(wifi_icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(wifi_icon, lv_color_hex(0xd92626), LV_PART_MAIN);
    lv_obj_set_style_text_font(wifi_icon, &lv_font_montserrat_16, LV_PART_MAIN);

    sdcard_icon = lv_label_create(scr);
    lv_obj_set_width(sdcard_icon, disp->driver->hor_res);
    lv_obj_align(sdcard_icon, LV_ALIGN_TOP_LEFT, 35, 4);
    lv_label_set_text(sdcard_icon, LV_SYMBOL_SD_CARD);
    lv_obj_set_style_text_color(sdcard_icon, lv_color_hex(0xd92626), LV_PART_MAIN);
    lv_obj_set_style_text_font(sdcard_icon, &lv_font_montserrat_16, LV_PART_MAIN);

    // Tạo style cho line
    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 3);
    lv_style_set_line_color(&style_line, lv_color_hex(0x1BCCFF));
    lv_style_set_line_rounded(&style_line, true);

    // Tạo line trên
    static lv_point_t line_points_above[] = {{0, 30}, {240, 30}};
    line_above = lv_line_create(scr);
    lv_line_set_points(line_above, line_points_above, 2);    /*Set the points*/
    lv_obj_add_style(line_above, &style_line, LV_PART_MAIN); /*Apply the new style*/

    // Tạo line dưới
    static lv_point_t line_points_bellow[] = {{0, 290}, {240, 290}};
    line_bellow = lv_line_create(scr);
    lv_line_set_points(line_bellow, line_points_bellow, 2);   /*Set the points*/
    lv_obj_add_style(line_bellow, &style_line, LV_PART_MAIN); /*Apply the new style*/

    // Tạo title label
    title_lable = lv_label_create(scr);
    lv_label_set_text(title_lable, "AIRSENSE V4");
    lv_obj_align(title_lable, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_set_style_text_font(title_lable, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(title_lable, lv_color_hex(0xff4d4d), LV_PART_MAIN);

    title_lable2 = lv_label_create(scr);
    lv_label_set_text(title_lable2, "Designed by SPARC LAB");
    lv_obj_align(title_lable2, LV_ALIGN_TOP_MID, 0, 65);
    lv_obj_set_style_text_font(title_lable2, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_set_style_text_color(title_lable2, lv_color_hex(0x1BCCFF), LV_PART_MAIN);

    // Tạo AQI label
    aqi_label = lv_label_create(scr);
    lv_label_set_text(aqi_label, "AQI: ");
    lv_obj_set_style_text_color(aqi_label, lv_color_hex(0xECAF15), LV_PART_MAIN);
    lv_obj_align(aqi_label, LV_ALIGN_TOP_MID, -28, 80);
    lv_obj_set_style_text_font(aqi_label, &lv_font_montserrat_30, LV_PART_MAIN);

    // Tạo temperature label
    temp_label = lv_label_create(scr);
    lv_label_set_text(temp_label, "Temperature");
    lv_obj_align(temp_label, LV_ALIGN_LEFT_MID, 5, -20);
    lv_obj_add_style(temp_label, &text_style, LV_PART_MAIN);

    // Tạo humidity label
    humd_label = lv_label_create(scr);
    lv_label_set_text(humd_label, "Humidity");
    lv_obj_align_to(humd_label, temp_label, LV_ALIGN_CENTER, -5, 50);
    lv_obj_add_style(humd_label, &text_style, LV_PART_MAIN);

    // Tạo pressure label
    press_label = lv_label_create(scr);
    lv_label_set_text(press_label, "Pressure");
    lv_obj_align_to(press_label, humd_label, LV_ALIGN_CENTER, -3, 50);
    lv_obj_add_style(press_label, &text_style, LV_PART_MAIN);

    // Tạo PM2.5 label
    pm25_label = lv_label_create(scr);
    lv_label_set_text(pm25_label, "PM2.5");
    lv_obj_align(pm25_label, LV_ALIGN_LEFT_MID, 165, -20);
    lv_obj_add_style(pm25_label, &text_style, LV_PART_MAIN);

    // Tạo PM10 label
    pm10_label = lv_label_create(scr);
    lv_label_set_text(pm10_label, "PM10");
    lv_obj_align_to(pm10_label, pm25_label, LV_ALIGN_CENTER, -3, 50);
    lv_obj_add_style(pm10_label, &text_style, LV_PART_MAIN);

    // Tạo CO2 label
    co2_label = lv_label_create(scr);
    lv_label_set_text(co2_label, "CO2");
    lv_obj_align_to(co2_label, pm10_label, LV_ALIGN_CENTER, -3, 50);
    lv_obj_add_style(co2_label, &text_style, LV_PART_MAIN);

    // Tạo giá trị AQI
    aqi_value_label = lv_label_create(scr);
    lv_obj_add_style(aqi_value_label, &data_style, LV_PART_MAIN);
    lv_obj_set_style_text_font(aqi_value_label, &lv_font_montserrat_30, LV_PART_MAIN);
    lv_label_set_text(aqi_value_label, "");

    // Tạo giá trị nhiệt độ
    temp_value_label = lv_label_create(scr);
    lv_obj_add_style(temp_value_label, &data_style, LV_PART_MAIN);
    lv_label_set_text(temp_value_label, "");

    // Tạo giá trị độ ẩm
    humd_value_label = lv_label_create(scr);
    lv_obj_add_style(humd_value_label, &data_style, LV_PART_MAIN);
    lv_label_set_text(humd_value_label, "");

    // Tạo giá trị áp suất
    press_value_label = lv_label_create(scr);
    lv_obj_add_style(press_value_label, &data_style, LV_PART_MAIN);
    lv_label_set_text(press_value_label, "");

    // Tạo giá trị PM2.5
    pm25_value_label = lv_label_create(scr);
    lv_obj_add_style(pm25_value_label, &data_style, LV_PART_MAIN);
    lv_label_set_text(pm25_value_label, "");

    // Tạo giá trị PM10
    pm10_value_label = lv_label_create(scr);
    lv_obj_add_style(pm10_value_label, &data_style, LV_PART_MAIN);
    lv_label_set_text(pm10_value_label, "");

    // Tạo giá trị CO2
    co2_value_label = lv_label_create(scr);
    lv_obj_add_style(co2_value_label, &data_style, LV_PART_MAIN);
    lv_label_set_text(co2_value_label, "");

    // Tạo label ngày giờ
    date_time_label = lv_label_create(scr);
    lv_obj_align(date_time_label, LV_ALIGN_TOP_RIGHT, -5, 4);
    lv_obj_set_style_text_font(date_time_label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_label_set_text(date_time_label, "");
    lv_obj_set_style_text_color(date_time_label, lv_color_hex(0x1BCCFF), LV_PART_MAIN);

    // Tạo label mã lỗi
    error_code_label = lv_label_create(scr);
    lv_obj_align(error_code_label, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_label_set_text(error_code_label, "");
    lv_obj_set_style_text_font(error_code_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(error_code_label, lv_color_hex(0x1BCCFF), LV_PART_MAIN);
    lv_timer_handler();
}
