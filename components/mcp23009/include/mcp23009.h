#pragma once
#ifndef MCP23009_H
#define MCP23009_H

#include "driver/i2c.h"
#include "esp_log.h"

enum registers_e{
    M_IODIR=0,
    M_IPOL,
    M_GPINTEN,
    M_DEFVAL,
    M_INTCON,
    M_IOCON,
    M_GPPU,
    M_INTF,
    M_INTCAP,
    M_GPIO,
    M_OLAT,
    M_REG_MAX
};

typedef struct {
    i2c_port_t i2c_port;
    i2c_config_t i2c_conf;
    uint8_t mcp_addr;
} mcpxx9_conf_t;

class mcp23009
{
private:
    mcpxx9_conf_t* conf;
    uint8_t shRegister[M_REG_MAX];
    esp_err_t writeReg(uint8_t, uint8_t);
    esp_err_t readReg(uint8_t, uint8_t*);
public:
    mcp23009();
    mcp23009(mcpxx9_conf_t*);
    ~mcp23009();
    esp_err_t init();
    esp_err_t init(mcpxx9_conf_t*);

    esp_err_t setDir(uint8_t);
    esp_err_t getDir(uint8_t*);

    esp_err_t setIpol(uint8_t);
    esp_err_t getIpol(uint8_t*);

    esp_err_t setChangeInt(uint8_t);
    esp_err_t getChangeInt(uint8_t*);

    esp_err_t setDefval(uint8_t);
    esp_err_t getDefval(uint8_t*);

    esp_err_t setIntcon(uint8_t);
    esp_err_t getIntcon(uint8_t*);

    esp_err_t setIocon(uint8_t);
    esp_err_t getIocon(uint8_t*);

    esp_err_t setPullup(uint8_t);
    esp_err_t getPullup(uint8_t*);

    esp_err_t getIntFlag(uint8_t*);

    esp_err_t getIntCap(uint8_t*);

    esp_err_t setGPIO(uint8_t);
    esp_err_t getGPIO(uint8_t*);

    esp_err_t setLatch(uint8_t);
    esp_err_t getLatch(uint8_t*);

    esp_err_t setBit(uint8_t,uint8_t);
    esp_err_t clrBit(uint8_t,uint8_t);

    //wannabe arduino fn
    esp_err_t digitalWrite(uint8_t, bool);
};

#endif