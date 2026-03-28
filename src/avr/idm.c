#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // struct gpio_adc
#include "board/misc.h" // alloc_maxsize
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "i2ccmds.h"
#include "trsync.h" // trsync_do_trigger
#include "internal.h" // GPIO
#include "board/irq.h" // irq_disable

DECL_CONSTANT("IDM_ADC_SMOOTH_COUNT", 10);
uint32_t trigger_freq,untrigger_freq;
uint8_t bc_trigger_reason,bc_trigger_invert;
struct trsync *bc_ts;
struct i2cdev_s *bc_i2c;
uint8_t idm_status=0;//idm激活指标
uint8_t idm_home_flag=0;//归零flag
//uint32_t bc_time=0;

uint16_t
readRegister(uint8_t reg) 
{
    uint8_t data[2]; // Buffer to store the read data

    // Read 2 bytes of data from LDC1612 channel 0
    i2c_read(bc_i2c->i2c_config, 1, &reg, 2, data);

    // Convert the read data to a 16-bit value
    uint16_t value = (data[0] << 8) | data[1];

    return value;
}

uint32_t
read_channel(void)
{
    //uint8_t error = 0;
    uint8_t MSB=readRegister(0x00);
    uint8_t LSB=readRegister(0x01);
    //error = MSB >> 12;
    uint32_t data = (((uint32_t)(MSB & 0x0FFF)) << 16) | LSB;
    return data;
}

void *
bc_mem_alloc(uint16_t size)
{
    void *data = alloc_chunk(size);
    return data;
}//分配i2c


void writeRegister(uint8_t reg, uint16_t data) {
    uint8_t buffer[3]; // 数据缓冲区

    buffer[0] = reg; // 寄存器地址
    buffer[1] = data >> 8; // 高位字节
    buffer[2] = data; // 低位字节

    i2c_write(bc_i2c->i2c_config, sizeof(buffer), buffer);
}

void
bc_init(void)
{
    gpio_out_setup(GPIO('D',5), 0);
    gpio_out_setup(GPIO('D',6), 0);
    uint32_t bc_delay=timer_read_time();
    while(bc_delay+100000>timer_read_time())
    {
    }
    bc_i2c= bc_mem_alloc(sizeof(*bc_i2c));
    bc_i2c->i2c_config = i2c_setup(0, 400000, 0x2A);
    bc_i2c->flags |= 1;
    writeRegister(0x1B,0x020C);
    writeRegister(0x1E,0x9000);
    writeRegister(0x10,0x000A);
    writeRegister(0x14,0x1002);
    writeRegister(0x08,0x04D6);
    writeRegister(0x1A,0x0401);
}
DECL_INIT(bc_init);

void
command_idm_stream(uint32_t *args)
{
    if(args[0])
        idm_status=1;
    else
        idm_status=0;
}
DECL_COMMAND(command_idm_stream,"idm_stream en=%u");
//切换激活状态
void
command_idm_set_threshold(uint32_t *args)
{
	trigger_freq=args[0];
	untrigger_freq=args[1];
}
DECL_COMMAND(command_idm_set_threshold,"idm_set_threshold trigger=%u untrigger=%u");

void
idm_home_task(void)
{
    if(!idm_home_flag)
        return;
    //if(bc_time+10000>timer_read_time())
    //    return;
    //bc_time=timer_read_time();
    uint32_t data = read_channel();
    if(!bc_trigger_invert)
	{
	    irq_disable();
        if(data>trigger_freq)
	    {
		    trsync_do_trigger(bc_ts, bc_trigger_reason);
		    idm_home_flag=0;
	    }
	    irq_enable();
	}
	else{
	    irq_disable();
	    if(data<untrigger_freq)
	    {
		    trsync_do_trigger(bc_ts, bc_trigger_reason);
		    idm_home_flag=0;
	    }
	    irq_enable();
        }
}
DECL_TASK(idm_home_task);

void
command_idm_home(uint32_t *args)
{
    bc_ts=trsync_oid_lookup(args[0]);
    bc_trigger_reason=args[1];
    bc_trigger_invert=args[2];
    //bc_time=timer_read_time();
    idm_home_flag=1;
}
DECL_COMMAND(command_idm_home,"idm_home trsync_oid=%c trigger_reason=%c trigger_invert=%c");

void
command_idm_stop_home(uint32_t *args)
{
	idm_home_flag=0;
        bc_ts=NULL;
}
DECL_COMMAND(command_idm_stop_home,"idm_stop_home");

void
command_idm_nvm_read(uint32_t *args)
{
    uint8_t data_len=args[0];
    uint8_t offset=args[1];
    uint32_t f_count=6881348;
    uint16_t adc_count=8000;
    uint64_t data=((uint64_t)adc_count)<<32 | f_count;
    sendf("idm_nvm_data bytes=%*s offset=%hu", data_len, &data, offset);
}
DECL_COMMAND(command_idm_nvm_read,"idm_nvm_read len=%c offset=%hu");



void
idm_task(void)
{
    if(!idm_status)
        return;
    for(uint8_t i=0;i<5;i++)
    {
        uint32_t data = read_channel();
        uint32_t temp = 8000;
        sendf("idm_data clock=%u data=%u temp=%u", timer_read_time(), data, temp);
    }
}
DECL_TASK(idm_task);
