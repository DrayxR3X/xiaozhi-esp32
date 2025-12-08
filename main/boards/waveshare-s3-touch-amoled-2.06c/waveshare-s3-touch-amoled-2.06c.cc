#include "application.h"
#include "button.h"
#include "codecs/box_audio_codec.h"
#include "config.h"
#include "wifi_board.h"
#include "custom_lcd_display.h"
#include "esp_lcd_sh8601.h"
#include "lvgl.h"
#include "axp2101.h"
#include "esp32_camera.h"
#include "esp_io_expander_tca9554.h"
#include "mcp_server.h"
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <wifi_station.h>

#include "esp_cam_sensor_types.h"

#define TAG "waveshare_s3_touch_amoled_2.06c"

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFE, (uint8_t[]) {0x00}, 1, 0},
    {0xC4, (uint8_t[]) {0x80}, 1, 0},
    {0x3A, (uint8_t[]) {0x55}, 1, 0}, // 0x55 for RGB565, 0x77 for RGB888
    {0x35, (uint8_t[]) {0x00}, 1, 0},
    {0x53, (uint8_t[]) {0x20}, 1, 0},
    {0x51, (uint8_t[]) {0xFF}, 1, 0}, // Brightness
    {0x63, (uint8_t[]) {0xFF}, 1, 0},
    {0x2A, (uint8_t[]) {0x00, 0x06, 0x01, 0xD7}, 4, 0},
    {0x2B, (uint8_t[]) {0x00, 0x00, 0x01, 0xD1}, 4, 0},
    {0x11, (uint8_t[]) {0x00}, 0, 100},
    {0x29, (uint8_t[]) {0x00}, 0, 0},
};

class CustomPmic : public Axp2101 {
public:
    CustomPmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        WriteReg(0x22, 0b110); // PWRON > OFFLEVEL as POWEROFF Source enable
        WriteReg(0x27, 0x10);  // hold 4s to power off

        // Set DC1 to 3.3V
        WriteReg(0x82, (3300 - 1500) / 100);

        // Set ALDO3 to 3.3V
        WriteReg(0x94, (3300 - 500) / 100);
        WriteReg(0x96, (1500 - 500) / 100);
        WriteReg(0x97, (2800 - 500) / 100);
    
        WriteReg(0x64, 0x02); // CV charger voltage setting to 4.1V
        
        WriteReg(0x61, 0x02); // set Main battery precharge current to 50mA
        WriteReg(0x62, 0x08); // set Main battery charger current to 400mA ( 0x08-200mA, 0x09-300mA, 0x0A-400mA )
        WriteReg(0x63, 0x01); // set Main battery term charge current to 25mA
    }
};

class CustomBoard : public WifiBoard {
  private:
    i2c_master_bus_handle_t   i2c_bus_;
    Button                    boot_button_;
    esp_lcd_panel_handle_t    panel_handle = NULL;
    esp_lcd_panel_io_handle_t io_handle    = NULL;
    CustomLcdDisplay         *display_;
    lv_indev_t               *touch_indev           = NULL;
    i2c_master_dev_handle_t   disp_touch_dev_handle = NULL;
    esp_io_expander_handle_t  io_expander  = NULL;
    CustomPmic* pmic_;
    Esp32Camera* camera_;

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg      = {};
        i2c_bus_cfg.i2c_port                     = I2C_NUM_0;
        i2c_bus_cfg.sda_io_num                   = AUDIO_CODEC_I2C_SDA_PIN;
        i2c_bus_cfg.scl_io_num                   = AUDIO_CODEC_I2C_SCL_PIN;
        i2c_bus_cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
        i2c_bus_cfg.glitch_ignore_cnt            = 7;
        i2c_bus_cfg.intr_priority                = 0;
        i2c_bus_cfg.trans_queue_depth            = 0;
        i2c_bus_cfg.flags.enable_internal_pullup = 1;
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new CustomPmic(i2c_bus_, 0x34);
    }

    void SetDispbacklight(uint8_t backlight) {
        uint32_t lcd_cmd = 0x51;
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= 0x02 << 24;
        uint8_t param = backlight;
        esp_lcd_panel_io_tx_param(io_handle, lcd_cmd, &param, 1);
    }

    void InitializeCamera() {
        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAM_PIN_D0,
                [1] = CAM_PIN_D1,
                [2] = CAM_PIN_D2,
                [3] = CAM_PIN_D3,
                [4] = CAM_PIN_D4,
                [5] = CAM_PIN_D5,
                [6] = CAM_PIN_D6,
                [7] = CAM_PIN_D7,
            },
            .vsync_io = CAM_PIN_VSYNC,
            .de_io = CAM_PIN_HREF,
            .pclk_io = CAM_PIN_PCLK,
            .xclk_io = CAM_PIN_XCLK,
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = false,         // 不初始化新的 SCCB，使用现有的 I2C 总线
            .i2c_handle = i2c_bus_,     // 使用现有的 I2C 总线句柄
            .freq = 300000,             // 300kHz
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = CAM_PIN_RESET,
            .pwdn_pin = CAM_PIN_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = 20000000,
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new Esp32Camera(video_config);
        camera_->
    }
    
    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.data0_io_num     = LCD_D0_PIN;
        buscfg.data1_io_num     = LCD_D1_PIN;
        buscfg.sclk_io_num      = LCD_SCL_PIN;
        buscfg.data2_io_num     = LCD_D2_PIN;
        buscfg.data3_io_num     = LCD_D3_PIN;
        buscfg.max_transfer_sz  = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num                   = LCD_CS_PIN;
        io_config.dc_gpio_num                   = -1;
        io_config.spi_mode                      = 0;
        io_config.pclk_hz                       = 40 * 1000 * 1000;
        io_config.trans_queue_depth             = 8;
        io_config.on_color_trans_done           = NULL;
        io_config.user_ctx                      = NULL;
        io_config.lcd_cmd_bits                  = 32;
        io_config.lcd_param_bits                = 8;
        io_config.flags.quad_mode               = true;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &io_handle));

        sh8601_vendor_config_t vendor_config    = {};
        vendor_config.init_cmds                 = lcd_init_cmds;
        vendor_config.init_cmds_size            = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]);
        vendor_config.flags.use_qspi_interface = 1;

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num             = LCD_RST_PIN;
        panel_config.rgb_ele_order              = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel             = 16;
        panel_config.vendor_config              = &vendor_config;

        ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
        esp_lcd_panel_set_gap(panel_handle, 0x16, 0x00);
        ESP_ERROR_CHECK(CustomLcdReset());
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

        display_ = new CustomLcdDisplay(io_handle, panel_handle, LCD_WIDTH, LCD_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeTools() {
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.disp.setbacklight", "设置屏幕亮度", PropertyList({Property("level", kPropertyTypeInteger, 0, 255)}), [this](const PropertyList &properties) -> ReturnValue {
            int level = properties["level"].value<int>();
            ESP_LOGI("setbacklight", "%d", level);
            SetDispbacklight(level);
            return true;
        });

        mcp_server.AddTool("self.disp.network", "重新配网", PropertyList(), [this](const PropertyList &) -> ReturnValue {
            ResetWifiConfiguration();
            return true;
        });
    }

    void InitializeTca9554() {
        ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(i2c_bus_, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander));
	    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_3 | IO_EXPANDER_PIN_NUM_5, IO_EXPANDER_OUTPUT));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_3 | IO_EXPANDER_PIN_NUM_5, 1));
    }

    int CustomLcdReset() {
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 1));
	    vTaskDelay(pdMS_TO_TICKS(100));
	    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 0));
	    vTaskDelay(pdMS_TO_TICKS(100));
	    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 1));
	    vTaskDelay(pdMS_TO_TICKS(50));
	    return ESP_OK;
    }

  public:
    CustomBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeAxp2101();
        InitializeSpi();
        InitializeTca9554();
        InitializeCamera();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeTools();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, 
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display *GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(CustomBoard);
