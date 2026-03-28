#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // struct gpio_adc
#include "board/misc.h" // alloc_maxsize
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "i2ccmds.h"
#include "i2c_software.h"
#include "trsync.h" // trsync_do_trigger
#include "internal.h" // GPIO
#include "board/irq.h" // irq_disable

DECL_CONSTANT("IDM_ADC_SMOOTH_COUNT", 16);
uint32_t trigger_freq=33784425,untrigger_freq=33581718;
uint8_t idm_trigger_reason,idm_trigger_invert;
struct trsync *idm_ts;
struct i2cdev_s *idm_i2c;
//struct i2c_software *bc_is;
uint8_t idm_status=0;//激活指标
static struct task_wake idm_update;
uint8_t idm_home_flag=0;//归零flag
uint32_t idm_time=-1;
uint32_t idm_hometime=-1;
struct gpio_adc temp_in;
struct gpio_out led;
struct gpio_out power;
struct timer idm_update_timer;
//struct gpio_in complete;
uint16_t
readRegister(uint8_t reg) 
{
    uint8_t data[2]; // Buffer to store the read data

    // Read 2 bytes of data from LDC1612 channel 0
    //i2c_software_read(bc_i2c->i2c_software, 1, &reg, 2, data);
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
    //i2c_software_write(bc_i2c->i2c_software, 3, buffer);
    i2c_write(idm_i2c->i2c_config, 3, buffer);
    
}

void idm_sleep(uint32_t delay)
{
    uint32_t timeout = timer_read_time() + timer_from_us(delay);
    for (;;) {
        if (!timer_is_before(timer_read_time(), timeout))
            break;
    }
}

void configuration(void)
{
    uint8_t addr[7]={0x1B,0x1E,0x10,0x14,0x08,0x19,0x1A};
    uint16_t config[7]={0x020C,0xD000,0x0100,0x1001,0x088C,0x0001,0x1601};
    for(uint8_t i=0;i<7;i++)
    {
        writeRegister(addr[i],config[i]);
        //gpio_out_write(led,0);
    }
    
}
struct i2c_software {
    struct gpio_out scl_out, sda_out;
    struct gpio_in scl_in, sda_in;
    uint8_t addr;
    unsigned int ticks;
};
static uint_fast8_t
idm_task_wakeup(struct timer *timer)
{
    sched_wake_task(&idm_update);
    timer->waketime=timer->waketime+20000000;
        return SF_RESCHEDULE;
}
void
idm_init(void)
{
    //uint64_t uid = 0;
    //for (int i = 0; i < 8; i++) {
    //    uid = uid<<8;
    //    uid |= ((uint32_t)global_uid[i]);
    //}
    //if(uid!=0xE6611032E365152E)
    //    return;
    power=gpio_out_setup(GPIO('A', 15), 0);
    gpio_out_setup(GPIO('A', 1), 0);
    idm_sleep(50000);
    gpio_pwm_setup(GPIO('B', 4), 1, 1);
    //complete=gpio_in_setup(GPIO('B', 4),0);
    led=gpio_out_setup(GPIO('B', 5), 1);
    temp_in=gpio_adc_setup(GPIO('A', 4));
    //irq_disable();
    idm_i2c= idm_mem_alloc(sizeof(*idm_i2c));
    idm_i2c->i2c_config = i2c_setup(0, 100000,(0x2A & 0x7f));
    idm_i2c->flags |= 2;
    configuration();
    idm_update_timer.waketime=timer_read_time()+100000;
    idm_update_timer.func=idm_task_wakeup;
    sched_add_timer(&idm_update_timer);
    //irq_enable();
}
DECL_INIT(idm_init);

void turn_off_idm(void)
{
    sched_del_timer(&idm_update_timer);
    gpio_out_write(led,1);
}

void
command_idm_stream(uint32_t *args)
{
    irq_disable();
    if(args[0])
    {
        idm_status=1;
    }
    else
    {
        idm_status=0;
    }
    irq_enable();
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
    /*if(!idm_home_flag)
    {
        if(idm_hometime==-1)
            idm_hometime=timer_read_time();
        if(idm_hometime+10000000>timer_read_time()){
            if(idm_hometime-10000000>timer_read_time())
                idm_hometime=timer_read_time();
            return;
        }
        idm_hometime=timer_read_time();
    }*/
    if(!idm_home_flag)
        return;
    uint32_t data = read_channel();
    if(data==0)
        return;
    irq_disable();
    if(data>trigger_freq)
    {
        
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
    uint32_t f_count=35791394;
    uint16_t adc_count=55927;
    uint64_t data=((uint64_t)adc_count)<<32 | f_count;
    sendf("idm_base_data bytes=%*s offset=%hu", data_len, &data, offset);
}
DECL_COMMAND(command_idm_base_read,"idm_base_read len=%c offset=%hu");

void
idm_update_task(void)
{
    if((!idm_status)&&(!sched_check_wake(&idm_update)))
        return;
    uint32_t data,clock;
    //if(gpio_in_read(complete))
    //    continue;
    clock=timer_read_time();
    data = read_channel();
    if(data==0)
        return;
    uint32_t temp = 0;
    uint8_t j=0;
    while(j<16)
        if(gpio_adc_sample(temp_in)==0)
        {
            temp+=gpio_adc_read(temp_in);
            j++;
        }
    sendf("idm_data clock=%u data=%u temp=%u", clock, data, temp);
    if(data>trigger_freq)
    {
	gpio_out_write(led,1);	
    }
    else if(data<untrigger_freq)
        gpio_out_write(led,0);
}
DECL_TASK(idm_update_task);
