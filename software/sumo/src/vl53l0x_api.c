#include "vl53l0x_api.h"

static const char* TAG = "VL53L0X";

static esp_err_t writeReg(struct VL53L0X_Data* c, uint8_t reg, uint8_t value);
static esp_err_t writeRegMulti(struct VL53L0X_Data* c, uint8_t reg, uint8_t* data_wr, size_t size);
static esp_err_t writeReg16bit(struct VL53L0X_Data* c, uint8_t reg, uint16_t data);
static esp_err_t readRegMulti(struct VL53L0X_Data* c, uint8_t reg, uint8_t* data, size_t size);
static esp_err_t readReg16bit(struct VL53L0X_Data* c, uint8_t reg, uint16_t* data);

static uint16_t decodeTimeout(uint16_t reg_val);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static esp_err_t getSpadInfo(struct VL53L0X_Data* c, uint8_t * count, bool * type_is_aperture);
static uint32_t getMeasurementTimingBudget(struct VL53L0X_Data* c);

esp_err_t writeReg(struct VL53L0X_Data* c, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( c->address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(c->port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t writeRegMulti(struct VL53L0X_Data* c, uint8_t reg, uint8_t* data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( c->address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(c->port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t writeReg16bit(struct VL53L0X_Data* c, uint8_t reg, uint16_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( c->address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (data >> 8) & 0xFF, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data & 0xFF, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(c->port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t readRegMulti(struct VL53L0X_Data* c, uint8_t reg, uint8_t* data, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( c->address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( c->address << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(c->port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t readReg16bit(struct VL53L0X_Data* c, uint8_t reg, uint16_t* data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( c->address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( c->address << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, (uint8_t*) data+1, ACK_VAL);
    i2c_master_read_byte(cmd, (uint8_t*) data, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(c->port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint16_t decodeTimeout(uint16_t reg_val) {
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) <<
            (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encodeTimeout(uint16_t timeout_mclks) {
    // format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
        ls_byte >>= 1;
        ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    } else {
        return 0;
    }
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static esp_err_t getSpadInfo(struct VL53L0X_Data* c, uint8_t * count, bool * type_is_aperture) {
    uint8_t tmp;

    esp_err_t ok = writeReg(c, 0x80, 0x01);
    if (ok != ESP_OK) return ok;

    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x00, 0x00);

    writeReg(c, 0xFF, 0x06);
    readRegMulti(c, 0x83, &tmp, 1);
    writeReg(c, 0x83, tmp | 0x04);
    writeReg(c, 0xFF, 0x07);
    writeReg(c, 0x81, 0x01);

    writeReg(c, 0x80, 0x01);

    writeReg(c, 0x94, 0x6b);
    writeReg(c, 0x83, 0x00);

    TickType_t start_tick = xTaskGetTickCount();
    tmp = 0;
    while (tmp == 0) {
        readRegMulti(c, 0x83, &tmp, 1);
        if ((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS > 100) {
            return ESP_FAIL;
        }
    }

    writeReg(c, 0x83, 0x01);
    readRegMulti(c, 0x92, &tmp, 1);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    writeReg(c, 0x81, 0x00);
    writeReg(c, 0xFF, 0x06);
    readRegMulti(c, 0x83, &tmp, 1);
    writeReg(c, 0x83, tmp & ~0x04);
    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x00, 0x01);

    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x80, 0x00);

    return ESP_OK;
}

void getSequenceStepEnables(struct VL53L0X_Data* c, struct SequenceStepEnables * enables) {
    uint8_t sequence_config;
    readRegMulti(c, SYSTEM_SEQUENCE_CONFIG, &sequence_config, 1);

    enables->tcc          = (sequence_config >> 4) & 0x1;
    enables->dss          = (sequence_config >> 3) & 0x1;
    enables->msrc         = (sequence_config >> 2) & 0x1;
    enables->pre_range    = (sequence_config >> 6) & 0x1;
    enables->final_range  = (sequence_config >> 7) & 0x1;
}

void getSequenceStepTimeouts(struct VL53L0X_Data* c, struct SequenceStepEnables * enables, struct SequenceStepTimeouts * timeouts) {
    uint8_t data;

    data = readRegMulti(c, PRE_RANGE_CONFIG_VCSEL_PERIOD, &data, 1);
    timeouts->pre_range_vcsel_period_pclks = decodeVcselPeriod(data);

    readRegMulti(c, MSRC_CONFIG_TIMEOUT_MACROP, &data, 1);
    timeouts->msrc_dss_tcc_mclks = data + 1;
    timeouts->msrc_dss_tcc_us =
        timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                timeouts->pre_range_vcsel_period_pclks);

    uint16_t timeout;
    readReg16bit(c, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &timeout);
    timeouts->pre_range_mclks = decodeTimeout(timeout);
    timeouts->pre_range_us =
        timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                timeouts->pre_range_vcsel_period_pclks);

    data = readRegMulti(c, FINAL_RANGE_CONFIG_VCSEL_PERIOD, &data, 1);
    timeouts->final_range_vcsel_period_pclks = decodeVcselPeriod(data);

    readReg16bit(c, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &timeout);
    timeouts->final_range_mclks = decodeTimeout(timeout);

    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us =
        timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                timeouts->final_range_vcsel_period_pclks);
}

static uint32_t getMeasurementTimingBudget(struct VL53L0X_Data* c) {
    struct SequenceStepEnables enables;
    struct SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(c, &enables);
    getSequenceStepTimeouts(c, &enables, &timeouts);

    if (enables.tcc) {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc) {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }
    
    return budget_us;
}

static bool setMeasurementTimingBudget(struct VL53L0X_Data* c) {
    struct SequenceStepEnables enables;
    struct SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000;

    if (c->measurement_timing_budget_us < MinTimingBudget) { return false; }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(c, &enables);
    getSequenceStepTimeouts(c, &enables, &timeouts);

    if (enables.tcc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > c->measurement_timing_budget_us) {
            // "Requested timeout too big."
            return false;
        }

        uint32_t final_range_timeout_us = c->measurement_timing_budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks =
        timeoutMicrosecondsToMclks(final_range_timeout_us,
                                    timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range) {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        writeReg16bit(c, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        // measurement_timing_budget_us = budget_us; // store for internal reuse
    }
    return true;
}

static esp_err_t performSingleRefCalibration(struct VL53L0X_Data* c, uint8_t vhv_init_byte) {
    uint8_t data;
    writeReg(c, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP
    
    TickType_t start_tick = xTaskGetTickCount();
    data = 0;
    while (data == 0) {
        readRegMulti(c, RESULT_INTERRUPT_STATUS, &data, 1);
        data = data & 0x07;
        if ((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS > 100) {
            ESP_LOGI(TAG, "int timed out in performSingleRefCalibration");
            return ESP_FAIL;
        }
    }

    writeReg(c, SYSTEM_INTERRUPT_CLEAR, 0x01);
    writeReg(c, SYSRANGE_START, 0x00);

    return ESP_OK;
}

esp_err_t setAddress(struct VL53L0X_Data* c, uint8_t new_addr) {
    new_addr &= 0x7F;

    esp_err_t ok = writeReg(c, I2C_SLAVE_DEVICE_ADDRESS, new_addr);
    if (ok == 0) {
        ESP_LOGI(TAG, "Changed address of 0x%.2x to 0x%.2x", c->address, new_addr);
        c->address = new_addr;
    } else {
        ESP_LOGI(TAG, "Failed to change address: %.2x", ok);
    }

    return ok;
}

esp_err_t vl53l0x_init(struct VL53L0X_Data* c) {
    uint8_t data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (c->address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ok = i2c_master_cmd_begin(c->port, cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ok != ESP_OK) {
        ESP_LOGI(TAG, "Failed to contact 0x%.2x: %.4x", c->address, ok);
        return ok;
    }
 
    // VL53L0X_DataInit() begin

    // set 2V8 mode
    readRegMulti(c, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &data, 1);
    writeReg(c, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, data | 0x01);

    // "Set I2C standard mode"
    writeReg(c, 0x88, 0x00);

    writeReg(c, 0x80, 0x01);
    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x00, 0x00);
    readRegMulti(c, 0x91, &c->stop_variable, 1);
    writeReg(c, 0x00, 0x01);
    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    readRegMulti(c, MSRC_CONFIG_CONTROL, &data, 1);
    writeReg(c, MSRC_CONFIG_CONTROL, data | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    //setSignalRateLimit(0.25);

    writeReg(c, SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // VL53L0X_DataInit() end

    // VL53L0X_StaticInit() begin

    uint8_t spad_count = 0;
    bool spad_type_is_aperture = false;
    if (getSpadInfo(c, &spad_count, &spad_type_is_aperture) != ESP_OK) { return ESP_FAIL; }

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    readRegMulti(c, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    writeReg(c, 0xFF, 0x01);
    writeReg(c, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    writeReg(c, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    writeReg(c, 0xFF, 0x00);
    writeReg(c, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }

    writeRegMulti(c, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() end

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x00, 0x00);

    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x09, 0x00);
    writeReg(c, 0x10, 0x00);
    writeReg(c, 0x11, 0x00);

    writeReg(c, 0x24, 0x01);
    writeReg(c, 0x25, 0xFF);
    writeReg(c, 0x75, 0x00);

    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x4E, 0x2C);
    writeReg(c, 0x48, 0x00);
    writeReg(c, 0x30, 0x20);

    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x30, 0x09);
    writeReg(c, 0x54, 0x00);
    writeReg(c, 0x31, 0x04);
    writeReg(c, 0x32, 0x03);
    writeReg(c, 0x40, 0x83);
    writeReg(c, 0x46, 0x25);
    writeReg(c, 0x60, 0x00);
    writeReg(c, 0x27, 0x00);
    writeReg(c, 0x50, 0x06);
    writeReg(c, 0x51, 0x00);
    writeReg(c, 0x52, 0x96);
    writeReg(c, 0x56, 0x08);
    writeReg(c, 0x57, 0x30);
    writeReg(c, 0x61, 0x00);
    writeReg(c, 0x62, 0x00);
    writeReg(c, 0x64, 0x00);
    writeReg(c, 0x65, 0x00);
    writeReg(c, 0x66, 0xA0);

    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x22, 0x32);
    writeReg(c, 0x47, 0x14);
    writeReg(c, 0x49, 0xFF);
    writeReg(c, 0x4A, 0x00);

    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x7A, 0x0A);
    writeReg(c, 0x7B, 0x00);
    writeReg(c, 0x78, 0x21);

    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x23, 0x34);
    writeReg(c, 0x42, 0x00);
    writeReg(c, 0x44, 0xFF);
    writeReg(c, 0x45, 0x26);
    writeReg(c, 0x46, 0x05);
    writeReg(c, 0x40, 0x40);
    writeReg(c, 0x0E, 0x06);
    writeReg(c, 0x20, 0x1A);
    writeReg(c, 0x43, 0x40);

    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x34, 0x03);
    writeReg(c, 0x35, 0x44);

    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x31, 0x04);
    writeReg(c, 0x4B, 0x09);
    writeReg(c, 0x4C, 0x05);
    writeReg(c, 0x4D, 0x04);

    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x44, 0x00);
    writeReg(c, 0x45, 0x20);
    writeReg(c, 0x47, 0x08);
    writeReg(c, 0x48, 0x28);
    writeReg(c, 0x67, 0x00);
    writeReg(c, 0x70, 0x04);
    writeReg(c, 0x71, 0x01);
    writeReg(c, 0x72, 0xFE);
    writeReg(c, 0x76, 0x00);
    writeReg(c, 0x77, 0x00);

    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x0D, 0x01);

    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x80, 0x01);
    writeReg(c, 0x01, 0xF8);

    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x8E, 0x01);
    writeReg(c, 0x00, 0x01);
    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x80, 0x00);

    // -- VL53L0X_load_tuning_settings() end

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    writeReg(c, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    readRegMulti(c, GPIO_HV_MUX_ACTIVE_HIGH, &data, 1);
    writeReg(c, GPIO_HV_MUX_ACTIVE_HIGH, data & ~0x10); // active low
    writeReg(c, SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end

    c->measurement_timing_budget_us = getMeasurementTimingBudget(c);

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    writeReg(c, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    setMeasurementTimingBudget(c);

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    writeReg(c, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (performSingleRefCalibration(c, 0x40) != ESP_OK) { return ESP_FAIL; }

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    writeReg(c, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (performSingleRefCalibration(c, 0x00) != ESP_OK) { return ESP_FAIL; }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    writeReg(c, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // VL53L0X_PerformRefCalibration() end

    return ESP_OK;
}

esp_err_t startContinuous(struct VL53L0X_Data* c) {
    esp_err_t ok = writeReg(c, 0x80, 0x01);
    if (ok != ESP_OK) return ok;

    writeReg(c, 0xFF, 0x01);
    writeReg(c, 0x00, 0x00);
    writeReg(c, 0x91, c->stop_variable);
    writeReg(c, 0x00, 0x01);
    writeReg(c, 0xFF, 0x00);
    writeReg(c, 0x80, 0x00);

    // continuous back-to-back mode
    writeReg(c, SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK

    return ESP_OK;
}

esp_err_t readRangeContinuousMillimeters(struct VL53L0X_Data* c, uint16_t* range) {
    uint8_t tmp = 0;
    
    TickType_t start_tick = xTaskGetTickCount();
    while (tmp == 0) {
        esp_err_t ok = readRegMulti(c, RESULT_INTERRUPT_STATUS, &tmp, 1);
        if (ok != ESP_OK) return ok;

        tmp &= 0x07;
        if ((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS > 100) {
            return ESP_FAIL;
        }
    }

    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    readReg16bit(c, RESULT_RANGE_STATUS + 10, range);

    writeReg(c, SYSTEM_INTERRUPT_CLEAR, 0x01);

    return ESP_OK;
}