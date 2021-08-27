#include "mcp23009.h"

mcp23009::mcp23009(/* args */)
{
}

mcp23009::mcp23009(mcpxx9_conf_t *_conf)
{
    conf = _conf;
}

mcp23009::~mcp23009()
{
}

esp_err_t mcp23009::init()
{
    ESP_ERROR_CHECK(i2c_param_config(conf->i2c_port, &(conf->i2c_conf)));
    i2c_set_timeout(conf->i2c_port, 400000);
    esp_err_t ret = i2c_driver_install(conf->i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ESP_LOGW("MCP", "Copying registers");
    for (uint8_t i = 0; i < M_REG_MAX; i++)
    {
        readReg(i, shRegister + i);
        ESP_LOGI("MCP", "Reg 0x%x: 0x%x", i, *(shRegister + i));
    }
    return ret;
}

esp_err_t mcp23009::init(mcpxx9_conf_t *_conf)
{
    conf = _conf;
    return init();
}

esp_err_t mcp23009::writeReg(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    i2c_master_start(link);
    i2c_master_write_byte(link, (conf->mcp_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(link, reg, 1);
    i2c_master_write_byte(link, val, 1);
    i2c_master_stop(link);

    esp_err_t ret = i2c_master_cmd_begin(conf->i2c_port, link, 100);
    i2c_cmd_link_delete(link);
    if (ret != ESP_OK)
        ESP_LOGE("MCP", "Reg write 0x%x error 0x%x", reg, ret);
    else
    {
        shRegister[reg] = val;
        ESP_LOGI("MCP", "Writing OK");
    }
    return ret;
}

esp_err_t mcp23009::readReg(uint8_t reg, uint8_t *val)
{
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    i2c_master_start(link);
    i2c_master_write_byte(link, (conf->mcp_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(link, reg, 1);
    //restart
    i2c_master_start(link);
    i2c_master_write_byte(link, (conf->mcp_addr << 1) | I2C_MASTER_READ, 0);
    i2c_master_read_byte(link, val, I2C_MASTER_LAST_NACK);
    i2c_master_stop(link);

    esp_err_t ret = i2c_master_cmd_begin(conf->i2c_port, link, 100);
    i2c_cmd_link_delete(link);
    if (ret != ESP_OK)
        ESP_LOGE("MCP", "Reg read 0x%x error 0x%x", reg, ret);
    else
        ESP_LOGI("MCP", "Reading OK");
    return ret;
}

esp_err_t mcp23009::setDir(uint8_t val)
{
    return writeReg(M_IODIR, val);
}

esp_err_t mcp23009::getDir(uint8_t *val)
{
    return readReg(M_IODIR, val);
}

esp_err_t mcp23009::setIpol(uint8_t val) {
    return writeReg(M_IPOL, val);
}
esp_err_t mcp23009::getIpol(uint8_t *val) {
    return readReg(M_IPOL, val);
}

esp_err_t mcp23009::setChangeInt(uint8_t val) {
    return writeReg(M_GPINTEN, val);
}
esp_err_t mcp23009::getChangeInt(uint8_t *val) {
    return readReg(M_GPINTEN, val);
}

esp_err_t mcp23009::setDefval(uint8_t val) {
    return writeReg(M_DEFVAL, val);
}
esp_err_t mcp23009::getDefval(uint8_t *val) {
    return readReg(M_DEFVAL, val);
}

esp_err_t mcp23009::setIntcon(uint8_t val) {
    return writeReg(M_INTCON, val);
}
esp_err_t mcp23009::getIntcon(uint8_t *val) {
    return readReg(M_INTCON, val);
}

esp_err_t mcp23009::setIocon(uint8_t val) {
    return writeReg(M_IOCON, val);
}
esp_err_t mcp23009::getIocon(uint8_t *val) {
    return readReg(M_IOCON, val);
}

esp_err_t mcp23009::setPullup(uint8_t val) {
    return writeReg(M_GPPU, val);
}
esp_err_t mcp23009::getPullup(uint8_t *val) {
    return readReg(M_GPPU, val);
}

esp_err_t mcp23009::getIntFlag(uint8_t *val) {
    return readReg(M_INTF, val);
}

esp_err_t mcp23009::getIntCap(uint8_t *val) {
    return readReg(M_INTCAP, val);
}

esp_err_t mcp23009::setGPIO(uint8_t val) {
    return writeReg(M_GPIO, val);
}
esp_err_t mcp23009::getGPIO(uint8_t *val) {
    return readReg(M_GPIO, val);
}

esp_err_t mcp23009::setLatch(uint8_t val) {
    return writeReg(M_OLAT, val);
}
esp_err_t mcp23009::getLatch(uint8_t *val) {
    return readReg(M_OLAT, val);
}

esp_err_t mcp23009::setBit(uint8_t reg, uint8_t bit){
    uint8_t val = shRegister[reg];
    if(val&(1<<bit))
        return ESP_OK;  //nothing to do
    else
        val |= (1<<bit);
    return writeReg(reg,val);
}

esp_err_t mcp23009::clrBit(uint8_t reg, uint8_t bit){
    uint8_t val = shRegister[reg];
    if(!(val&(1<<bit)))
        return ESP_OK;  //nothing to do
    else
        val &=~ (1<<bit);
    return writeReg(reg,val);
}

esp_err_t mcp23009::digitalWrite(uint8_t pin, bool state){
    uint8_t val=shRegister[M_GPIO];
    if((val&(1<<pin))==state){
        //do nothing
        return ESP_OK;
    } else {
        if(!state)
            val &=~ (1<<pin);
        else
            val |= (1<<pin);
        return writeReg(M_GPIO,val);
    }
}