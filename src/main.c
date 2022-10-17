#include <stdio.h>
#include <string.h>
#include <drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/zephyr.h>
#include <lvgl.h>
#include <hal/nrf_saadc.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>

#define ADC_DEVICE_NAME DT_INST(0, nordic_nrf_saadc)
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN0 // Arduino Naming A0
#define ADC_2ND_CHANNEL_ID 2
#define ADC_2ND_CHANNEL_INPUT NRF_SAADC_INPUT_AIN2 // Arduino Naming A2
#define ADC_MAX_VAL  ((1U << ADC_RESOLUTION) - 1U)

#define DISP_WIDTH  320
#define DISP_HEIGHT 240
#define BUFFER_SIZE 512
#define WINDOW_SIZE 512
#define MAG_SCALE   400

static int16_t m_sample_buffer_1[BUFFER_SIZE];
static int16_t m_sample_buffer_2[BUFFER_SIZE];
static float32_t sample_buffer_norm_1[BUFFER_SIZE];
static float32_t sample_buffer_norm_2[BUFFER_SIZE];
static float32_t fftInput_1[WINDOW_SIZE*2];
static float32_t fftOutput_1[WINDOW_SIZE];
static float32_t fftInput_2[WINDOW_SIZE*2];
static float32_t fftOutput_2[BUFFER_SIZE];


LOG_MODULE_REGISTER(app);


uint32_t colors[] = {
    0x24c4e6, 0x05a6fb, 0x0571fb, 0x053ffb, 0x0509fb, 0x3305fb, 0x6905fb,
    0x9705fb, 0xcd05fb, 0xfb05f7, 0xfb05c1, 0xfb058f, 0xfb055a, 0xfb0528,
    0xfb1505, 0xfb4a05, 0xfb7c05, 0xfbb205, 0xfbe405, 0xe0fb05, 0xaefb05,
    0x78fb05, 0x46fb05, 0x11fb05, 0x05fb2c, 0x05fb5d, 0x05fb93, 0x05fbc5,
    0x05fbfb, 0x05c9fb, 0x0593fb, 0x0584fb,
};

typedef struct {
    lv_obj_t *obj;
    float values[WINDOW_SIZE];
    float peaks[WINDOW_SIZE];
    int mid_point;
} spectrum_t;

spectrum_t spectrum_1;
spectrum_t spectrum_2;

const struct device *adc_dev;

static const struct adc_channel_cfg m_1st_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive = ADC_1ST_CHANNEL_INPUT,
#endif
};

static const struct adc_channel_cfg m_2nd_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_2ND_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive = ADC_2ND_CHANNEL_INPUT,
#endif
};

const struct adc_sequence_options sequence_opts = {
    .interval_us = 0,
    .callback = NULL,
    .user_data = NULL,
    .extra_samplings = BUFFER_SIZE -1,
};

static int adc_sample(void)
{
    int ret;

    const struct adc_sequence sequence_1 = {
        .options = &sequence_opts,
        .channels = BIT(ADC_1ST_CHANNEL_ID),
        .buffer = m_sample_buffer_1,
        .buffer_size = sizeof(m_sample_buffer_1),
        .resolution = ADC_RESOLUTION,
    };

    const struct adc_sequence sequence_2 = {
        .options = &sequence_opts,
        .channels = BIT(ADC_2ND_CHANNEL_ID),
        .buffer = m_sample_buffer_2,
        .buffer_size = sizeof(m_sample_buffer_2),
        .resolution = ADC_RESOLUTION,
    };

    if (!adc_dev) {
        return -1;
    }

    ret = adc_read(adc_dev, &sequence_1);
    //LOG_ERR("ADC [0] read err: %d\n", ret);
    ret = adc_read(adc_dev, &sequence_2);
    //LOG_ERR("ADC [2] read err: %d\n", ret);

    /* print the AIN0, AIN2 values */
    //for (int i = 0; i < BUFFER_SIZE; i++) {
    //    LOG_INF("%d, %d\n", m_sample_buffer_1[i], m_sample_buffer_2[i]);
    //}
    
    return ret;
}

static void spectrum_draw_event_cb(lv_event_t *e) 
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_REFR_EXT_DRAW_SIZE) {
        lv_event_set_ext_draw_size(e, LV_VER_RES);
    } else if (code == LV_EVENT_COVER_CHECK) {
        lv_event_set_cover_res(e, LV_COVER_RES_NOT_COVER);
    } else if (code == LV_EVENT_DRAW_POST) {
        lv_obj_t *obj       = lv_event_get_target(e);
        spectrum_t *spectrum =  lv_event_get_user_data(e);
        lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

        lv_opa_t opa = lv_obj_get_style_opa(obj, LV_PART_MAIN);
        if (opa < LV_OPA_MIN) return;

        lv_draw_rect_dsc_t draw_rect_dsc;
        lv_draw_rect_dsc_init(&draw_rect_dsc);
        draw_rect_dsc.bg_opa = LV_OPA_COVER;

        lv_draw_line_dsc_t draw_line_dsc;
        lv_draw_line_dsc_init(&draw_line_dsc);
        draw_line_dsc.width = 1;

        int x_step    = (int)(DISP_WIDTH - 16) / (WINDOW_SIZE / 16);
        int bar_count = 1;
        // skip first 2
        for (int i = 2; i < WINDOW_SIZE / 4; i += 4) {
            float ave = 0;
            for (int j = 0; j < 4; j++) {
                ave += spectrum->values[i + j];
            }
            ave /= 4;
            int bar_value = MIN(125.0f, 0.25f * ave);
            ave           = 0;
            for (int j = 0; j < 4; j++) {
                ave += spectrum->peaks[i + j];
            }
            ave /= 4;
            int peak_value = MIN(125.0f, 0.25f * ave);

            draw_rect_dsc.bg_color = lv_color_hex(colors[bar_count - 1]);
            /* 5 is the bar width,  bar_value is bar height */
            lv_area_t above_rect;
            above_rect.x1 = bar_count * x_step;
            above_rect.x2 = bar_count * x_step + 5;
            above_rect.y1 = spectrum->mid_point - (int)(bar_value / 2);
            above_rect.y2 = spectrum->mid_point;
            lv_draw_rect(draw_ctx, &draw_rect_dsc, &above_rect);

            lv_area_t below_rect;
            below_rect.x1 = bar_count * x_step;
            below_rect.x2 = bar_count * x_step + 5;
            below_rect.y1 = spectrum->mid_point;
            below_rect.y2 = spectrum->mid_point + (int)(bar_value / 2);
            lv_draw_rect(draw_ctx, &draw_rect_dsc, &below_rect);

            draw_line_dsc.color = lv_color_hex(colors[bar_count - 1]);

            lv_point_t above_line[2];
            /* upside line always 2 px above the bar */
            above_line[0].x = bar_count * x_step;
            above_line[0].y = spectrum->mid_point - (int)(peak_value / 2) - 2;
            above_line[1].x = bar_count * x_step + 6;
            above_line[1].y = spectrum->mid_point - (int)(peak_value / 2) - 2;
            lv_draw_line(draw_ctx, &draw_line_dsc, &above_line[0],
                         &above_line[1]);

            lv_point_t below_line[2];
            /* under line always 2 px below the bar */
            below_line[0].x = bar_count * x_step;
            below_line[0].y = spectrum->mid_point + (int)(peak_value / 2) + 2;
            below_line[1].x = bar_count * x_step + 6;
            below_line[1].y = spectrum->mid_point + (int)(peak_value / 2) + 2;
            lv_draw_line(draw_ctx, &draw_line_dsc, &below_line[0],
                         &below_line[1]);

            bar_count++;
        }
    }
}

void create_spectrum_object(spectrum_t *spectrum)
{
    spectrum->obj = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(spectrum->obj);
    lv_obj_refresh_ext_draw_size(spectrum->obj);
    lv_obj_set_size(spectrum->obj, DISP_WIDTH - 16, (DISP_HEIGHT - 16) / 2);
    lv_obj_set_pos(spectrum->obj, 16, spectrum->mid_point - 58);
    lv_obj_clear_flag(spectrum->obj, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(spectrum->obj, spectrum_draw_event_cb, LV_EVENT_ALL, spectrum);
}

static void update_spectrum(spectrum_t *spectrum, float *magnitudes) 
{
    for (int i = 0; i < WINDOW_SIZE; i++) {
        float mag = magnitudes[i] * MAG_SCALE;
        if (mag > spectrum->values[i]) {
            spectrum->values[i] = mag;
        } else {
            spectrum->values[i] = 0.7 * spectrum->values[i] + 0.3 * mag;
        }
        if (mag > spectrum->peaks[i]) {
            spectrum->peaks[i] = mag;
        } else {
            spectrum->peaks[i] = 0.95 * spectrum->peaks[i] + 0.05 * mag;
        }
    }
}

int main(void)
{
    /* Set CPU frequency to 128 MHz  */
    NRF_CLOCK_S->HFCLKCTRL = (CLOCK_HFCLKCTRL_HCLK_Div1 << CLOCK_HFCLKCTRL_HCLK_Pos);
    adc_dev = DEVICE_DT_GET(ADC_DEVICE_NAME);

    if (!adc_dev) {
        LOG_ERR("device_get_binding ADC_0 failed\n");
        return -1;
    }

    int err;
    err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
    err = adc_channel_setup(adc_dev, &m_2nd_channel_cfg);

    if (err) {
        LOG_ERR("Error in adc setup: %d\n", err);
    }

    /* Trigger offset calibration
     * As this generates a _DONE and _RESULT event
     * the first result will be incorrect.
     */
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;

    while (1) {
        err = adc_sample();
        if (err) {
            LOG_ERR("Error in adc sampling: %d\n", err);
        }
        
        // normalize audio buffer [0.0 - 1.0]
        for (int i = 0; i < BUFFER_SIZE; i++) {
             sample_buffer_norm_1[i] = (m_sample_buffer_1[i] * 1.0f) / ADC_MAX_VAL;
             sample_buffer_norm_2[i] = (m_sample_buffer_2[i] * 1.0f) / ADC_MAX_VAL;
        }
      
        // calculate mean
        float32_t mean_1, mean_2;
        arm_mean_f32(sample_buffer_norm_1, BUFFER_SIZE, &mean_1);  
        arm_mean_f32(sample_buffer_norm_2, BUFFER_SIZE, &mean_2);  

        // populate FFT inputs by removing DC bias
        for (int i = 0; i < WINDOW_SIZE*2; i += 2) { 
            fftInput_1[i] = sample_buffer_norm_1[i/2] - mean_1; // Re
            fftInput_1[i+1] = 0; // Im
            fftInput_2[i] = sample_buffer_norm_2[i/2] - mean_2;  // Re
            fftInput_2[i+1] = 0; // Im
            //printf("%f, %f\n", fftInput_1[i], fftInput_2[i]);
        }

        // calculate FFT
        arm_cfft_f32(&arm_cfft_sR_f32_len512, fftInput_1, 0, 1);
        arm_cfft_f32(&arm_cfft_sR_f32_len512, fftInput_2, 0, 1);

        // calculate magnitutes 
        arm_cmplx_mag_f32(fftInput_1, fftOutput_1, WINDOW_SIZE);
        arm_cmplx_mag_f32(fftInput_2, fftOutput_2, WINDOW_SIZE);

        k_sleep(K_MSEC(1));
    }
}

void display_main(void)
{

    const struct device *display_dev;
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    if (!device_is_ready(display_dev)) {
        LOG_ERR("Device not ready, aborting.");
        while (1) {}
    }

    spectrum_1.mid_point = (DISP_HEIGHT / 4) - 1;
    spectrum_2.mid_point = ((3 * DISP_HEIGHT) / 4) - 1;
    create_spectrum_object(&spectrum_1);
    create_spectrum_object(&spectrum_2);

    display_blanking_off(display_dev);

    while (1) {
        update_spectrum(&spectrum_1, fftOutput_1);
        update_spectrum(&spectrum_2, fftOutput_2);
        lv_obj_invalidate(spectrum_1.obj);
        lv_obj_invalidate(spectrum_2.obj);
        lv_task_handler();
        k_sleep(K_MSEC(1));
    }
}

K_THREAD_DEFINE(display_thread, 8192, display_main, NULL, NULL, NULL, 7, 0, 0);

