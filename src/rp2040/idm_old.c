#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // struct gpio_adc
#include "board/misc.h" // alloc_maxsize
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "i2ccmds.h"
#include "trsync.h" // trsync_do_trigger
#include "internal.h" // GPIO
#include "board/irq.h" // irq_disable
#include "chipid.h"

DECL_CONSTANT("IDM_ADC_SMOOTH_COUNT", 16);
uint32_t trigger_freq=44803800,untrigger_freq=44534977;
uint8_t idm_trigger_reason,idm_trigger_invert;
struct trsync *idm_ts;
struct i2cdev_s *idm_i2c;
uint8_t upload_skip=0;
uint8_t idm_status=0;//激活指标
uint8_t idm_home_flag=0;//归零flag
uint32_t idm_time=-1;
struct gpio_adc temp_in;
struct gpio_out led;
struct gpio_out power;
struct gpio_in complete;

uint16_t
readRegister(uint8_t reg) 
{
    uint8_t data[2]; // Buffer to store the read data

    // Read 2 bytes of data from LDC1612 channel 0
    i2c_read(idm_i2c->i2c_config, 1, &reg, 2, data);

    // Convert the read data to a 16-bit value
    uint16_t value = (data[0] << 8) | data[1];

    return value;
}

uint32_t
read_channel(void)
{
    //uint8_t error = 0;
    uint16_t MSB=readRegister(0x00);
    uint16_t LSB=readRegister(0x01);
    //error = MSB >> 12;
    uint32_t data = (((uint32_t)(MSB & 0x0FFF)) << 16) | LSB;
    //readRegister(0x18);
    return data;
}

void *
idm_mem_alloc(uint16_t size)
{
    void *data = alloc_chunk(size);
    return data;
}//分配i2c


void writeRegister(uint8_t reg, uint16_t data) {
    uint8_t buffer[3]; // 数据缓冲区

    buffer[0] = reg; // 寄存器地址
    buffer[1] = data >> 8; // 高位字节
    buffer[2] = data; // 低位字节

    i2c_write(idm_i2c->i2c_config, 3, buffer);
}

void idm_sleep(uint32_t delay)
{
    uint32_t time=timer_read_time();
    while(time+delay>timer_read_time()){}
}

void configuration(void)
{
    uint8_t addr[7]={0x1B,0x1E,0x10,0x14,0x08,0x19,0x1A};
    uint16_t config[7]={0x020C,0xD000,0x0100,0x1001,0x088C,0x0001,0x1601};
    for(uint8_t i=0;i<7;i++)
    {
        writeRegister(addr[i],config[i]);
    }
}
void
idm_init(void)
{

    /*uint64_t uid = 0;
    for (int i = 0; i < 8; i++) {
        uid = uid<<8;
        uid |= ((uint32_t)global_uid[i]);
    }
    if(uid!=0xE6625C48936C3C2F)
        return;*/
    power=gpio_out_setup(1, 0);
    gpio_pwm_setup(2, 1, 2);
    complete=gpio_in_setup(0,0);
    led=gpio_out_setup(10, 1);
    temp_in=gpio_adc_setup(26);
    irq_disable();
    idm_i2c= idm_mem_alloc(sizeof(*idm_i2c));
    idm_i2c->i2c_config = i2c_setup(2, 400000, 0x2A);
    idm_i2c->flags |= 2;
    configuration();
    irq_enable();
}
DECL_INIT(idm_init);

void
command_idm_stream(uint32_t *args)
{
    if(args[0])
    {
        idm_status=1;
    }
    else
    {
        idm_status=0;
    }
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
    //if(!idm_home_flag)
    //    return;
    //if(idm_time+1000>timer_read_time())
    //    return;
    //idm_time=timer_read_time();
    //if(gpio_in_read(complete))
    //    return;
    uint32_t data = read_channel();
    if(data==0)
        return;
    irq_disable();
    if(data>trigger_freq)
    {
        if(idm_home_flag)
	    trsync_do_trigger(idm_ts, idm_trigger_reason);
	gpio_out_write(led,1);	
    }
    else if(data<untrigger_freq)
        gpio_out_write(led,0);
    irq_enable();
}
DECL_TASK(idm_home_task);

void
command_idm_home(uint32_t *args)
{
    idm_ts=trsync_oid_lookup(args[0]);
    idm_trigger_reason=args[1];
    idm_trigger_invert=args[2];
    idm_home_flag=1;
}
DECL_COMMAND(command_idm_home,"idm_home trsync_oid=%c trigger_reason=%c trigger_invert=%c");

void
command_idm_stop_home(uint32_t *args)
{
    idm_home_flag=0;
    idm_ts=NULL;
}
DECL_COMMAND(command_idm_stop_home,"idm_stop_home");

void
command_idm_base_read(uint32_t *args)
{
    uint8_t data_len=args[0];
    uint8_t offset=args[1];
    uint32_t f_count=43890000;
    uint16_t adc_count=55927;
    uint64_t data=((uint64_t)adc_count)<<32 | f_count;
    sendf("idm_base_data bytes=%*s offset=%hu", data_len, &data, offset);
}
DECL_COMMAND(command_idm_base_read,"idm_base_read len=%c offset=%hu");


void
idm_task(void)
{
    if(!idm_status)
    {
        if(idm_time==-1)
            idm_time=timer_read_time();
        if(idm_time+500000>timer_read_time()){
            if(idm_time-500000>timer_read_time())
                idm_time=timer_read_time();
            return;
        }
        idm_time=timer_read_time();
    }
    uint32_t data,clock;
    for(uint8_t i=0;i<1;i++)
    {
        //if(gpio_in_read(complete))
        //    continue;
        clock=timer_read_time();
        data = read_channel();
        if(data==0)
            continue;
        uint32_t temp = 0;
        uint8_t j=0;
        while(j<16)
            if(gpio_adc_sample(temp_in)==0)
            {
                temp+=gpio_adc_read(temp_in);
                j++;
            }
        if(!upload_skip)
        {
            sendf("idm_data clock=%u data=%u temp=%u", clock, data, temp);
            upload_skip=1;    
        }
        else
            upload_skip=0;
    }
}
DECL_TASK(idm_task);

