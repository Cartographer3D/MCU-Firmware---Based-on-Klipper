#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // struct gpio_adc
#include "board/misc.h" // alloc_maxsize
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "i2ccmds.h"
#include "trsync.h" // trsync_do_trigger
#include "internal.h" // GPIO
#include "board/irq.h" // irq_disable
#define num 10

DECL_CONSTANT("CARTOGRAPHER_ADC_SMOOTH_COUNT", 16);
uint32_t trigger_freq=33784425,untrigger_freq=33581718;
uint8_t idm_trigger_reason,idm_trigger_invert;
struct trsync *idm_ts;
struct i2cdev_s *idm_i2c;
//struct i2c_software *bc_is;
static uint8_t idm_status=0;//激活指标
static struct task_wake idm_update;
static struct task_wake idm_delay;
uint8_t trigger_method=0;
uint8_t idm_home_flag=0;//归零flag
uint32_t idm_hometime;
uint32_t homing_freq=0;
int32_t stack[num];
int32_t max=0;
uint8_t dur=20;
uint8_t current=0;
uint8_t start=0;
uint32_t trigger_threshold=1000;
struct gpio_adc temp_in;
struct gpio_out led;
struct gpio_out power;
struct timer idm_update_timer;
struct timer delay_timer;
//struct gpio_in complete;
uint16_t
readRegister(uint8_t reg) 
{
    uint8_t data[2]; // Buffer to store the read data

    // Read 2 bytes of data from LDC1612 channel 0
    //i2c_software_read(bc_i2c->i2c_software, 1, &reg, 2, data);
    int ret = i2c_dev_read(idm_i2c, 1, &reg, 2, data);
    
    i2c_shutdown_on_err(ret);
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
    i2c_dev_write(idm_i2c, 3, buffer);
    
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

static uint_fast8_t
idm_task_wakeup(struct timer *timer)
{
    sched_wake_task(&idm_update);
    timer->waketime=timer->waketime+20000000;
        return SF_RESCHEDULE;
}
static uint_fast8_t
idm_delay_wakeup(struct timer *timer)
{
    sched_wake_task(&idm_delay);
    #if CONFIG_FOR_K1
      timer->waketime=timer->waketime+timer_from_us(4000);
    #else
      timer->waketime=timer->waketime+timer_from_us(2000);
    #endif
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
    power=gpio_out_setup(GPIO('B', 4), 0);
    #if CONFIG_CANSERIAL
      gpio_out_setup(GPIO('A', 3), 1);
    #endif
    //gpio_out_setup(GPIO('A', 3), 0);
    idm_sleep(50000);
    gpio_pwm_setup(GPIO('B', 6), 6, 1<<14);
    //complete=gpio_in_setup(GPIO('B', 4),0);
    led=gpio_out_setup(GPIO('A', 15), 0);
    temp_in=gpio_adc_setup(GPIO('A', 4));
    //irq_disable();
    idm_i2c= idm_mem_alloc(sizeof(*idm_i2c));
    idm_i2c->i2c_hw = i2c_setup(0, 400000,(0x2A & 0x7f));
    idm_i2c->flags |= 2;
    configuration();
    idm_update_timer.waketime=timer_read_time()+100000;
    delay_timer.waketime=timer_read_time()+100000;
    idm_update_timer.func=idm_task_wakeup;
    delay_timer.func=idm_delay_wakeup;
    sched_add_timer(&idm_update_timer);
    sched_add_timer(&delay_timer);
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
DECL_COMMAND(command_idm_stream,"cartographer_stream en=%u");
//切换激活状态
void
command_idm_set_threshold(uint32_t *args)
{
	trigger_freq=args[0];
	untrigger_freq=args[1];
}
DECL_COMMAND(command_idm_set_threshold,"cartographer_set_threshold trigger=%u untrigger=%u");

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
    if(trigger_method)
    {
	if(homing_freq==0)
	{
	    idm_hometime=timer_read_time();
	    homing_freq = read_channel();
	    return;
	}
	uint32_t time=timer_read_time();
	
	if(timer_is_before(time,idm_hometime+timer_from_us(dur*100)))
	{
	    return;
	}
	idm_hometime=time;
	uint32_t data=read_channel();
	if(current<num)
	{
	    stack[current]=data-homing_freq;
	    homing_freq=data;
	    current++;
	    if(start==0)
	    	return;
	}
	else
	{
	    start=1;
	    current=0;
	    stack[current]=data-homing_freq;
	    homing_freq=data;
	    current++;
	}
	int32_t avr=0;
	for(int i=0;i<num;i++)
	    avr+=stack[i];
	avr=avr/10;
	if(max>trigger_threshold+avr && max>100)
	{
	    trsync_do_trigger(idm_ts, idm_trigger_reason);
	    homing_freq=0;
	    start=0;
	    max=0;
	    current=0;
	    }
	else if(avr>max)
	{
	    max=avr;
	}
	//irq_disable();

	//irq_enable();
    }
    else
    {
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
}
DECL_TASK(idm_home_task);
void
command_idm_home(uint32_t *args)
{
    idm_ts=trsync_oid_lookup(args[0]);
    idm_trigger_reason=args[1];
    idm_trigger_invert=args[2];
    trigger_threshold=args[3];
    trigger_method=args[4];
    homing_freq=0;
    start=0;
    max=0;
    current=0;
    idm_home_flag=1;
}
DECL_COMMAND(command_idm_home,"cartographer_home trsync_oid=%c trigger_reason=%c trigger_invert=%c threshold=%u trigger_method=%u");

void
command_idm_stop_home(uint32_t *args)
{
    idm_home_flag=0;
    idm_ts=NULL;
}
DECL_COMMAND(command_idm_stop_home,"cartographer_stop_home");

void
command_idm_base_read(uint32_t *args)
{
    uint8_t data_len=args[0];
    uint8_t offset=args[1];
    uint32_t f_count=31600800;
    uint16_t adc_count=55927;
    uint64_t data=((uint64_t)adc_count)<<32 | f_count;
    sendf("cartographer_base_data bytes=%*s offset=%hu", data_len, &data, offset);
}
DECL_COMMAND(command_idm_base_read,"cartographer_base_read len=%c offset=%hu");

void
idm_update_task(void)
{
    if(!idm_status)
    {
        if(!sched_check_wake(&idm_update))
            return;
    }
    else if(!sched_check_wake(&idm_delay))
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
    sendf("cartographer_data clock=%u data=%u temp=%u", clock, data, temp);
    if(data>trigger_freq)
    {
	gpio_out_write(led,1);	
    }
    else if(data<untrigger_freq)
        gpio_out_write(led,0);
}
DECL_TASK(idm_update_task);
