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

DECL_CONSTANT("CARTOGRAPHER_ADC_SMOOTH_COUNT", 16);
uint32_t trigger_freq=33784425,untrigger_freq=33581718;
uint8_t cartographer_trigger_reason,cartographer_trigger_invert;
struct trsync *cartographer_ts;
struct i2cdev_s *cartographer_i2c;
//struct i2c_software *bc_is;
uint8_t cartographer_status=0;//激活指标
static struct task_wake cartographer_update;
uint8_t cartographer_home_flag=0;//归零flag
uint8_t trigger_method=0;
uint32_t cartographer_hometime;
uint32_t homing_freq=0;
int32_t stack[6]={0,0,0,0,0,0};
int32_t max=0;
uint8_t dur=20;
uint8_t current=0;
uint8_t start=0;
uint32_t trigger_threshold=1000;
struct gpio_adc temp_in;
struct gpio_out led;
struct gpio_out power;
struct timer cartographer_update_timer;
//struct gpio_in complete;
uint16_t
readRegister(uint8_t reg) 
{
    uint8_t data[2]; // Buffer to store the read data

    // Read 2 bytes of data from LDC1612 channel 0
    //i2c_software_read(bc_i2c->i2c_software, 1, &reg, 2, data);
    i2c_dev_read(cartographer_i2c, 1, &reg, 2, data);
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
cartographer_mem_alloc(uint16_t size)
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
    i2c_dev_write(cartographer_i2c, 3, buffer);
    
}

void cartographer_sleep(uint32_t delay)
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
cartographer_task_wakeup(struct timer *timer)
{
    sched_wake_task(&cartographer_update);
    timer->waketime=timer->waketime+20000000;
        return SF_RESCHEDULE;
}
void
cartographer_init(void)
{
    //uint64_t uid = 0;
    //for (int i = 0; i < 8; i++) {
    //    uid = uid<<8;
    //    uid |= ((uint32_t)global_uid[i]);
    //}
    //if(uid!=0xE6611032E365152E)
    //    return;
    power=gpio_out_setup(GPIO('A', 15), 0);
    #if CONFIG_CANSERIAL
      gpio_out_setup(GPIO('A', 1), 1);
    #endif
    cartographer_sleep(50000);
    gpio_pwm_setup(GPIO('B', 4), 1, 1);
    //complete=gpio_in_setup(GPIO('B', 4),0);
    led=gpio_out_setup(GPIO('B', 5), 1);
    temp_in=gpio_adc_setup(GPIO('A', 4));
    //irq_disable();
    cartographer_i2c= cartographer_mem_alloc(sizeof(*cartographer_i2c));
    cartographer_i2c->i2c_hw = i2c_setup(0, 400000,(0x2A & 0x7f));
    cartographer_i2c->flags |= 2;
    configuration();
    cartographer_update_timer.waketime=timer_read_time()+100000;
    cartographer_update_timer.func=cartographer_task_wakeup;
    sched_add_timer(&cartographer_update_timer);
    //irq_enable();
}
DECL_INIT(cartographer_init);

void turn_off_cartographer(void)
{
    sched_del_timer(&cartographer_update_timer);
    gpio_out_write(led,1);
}

void
command_cartographer_stream(uint32_t *args)
{
    irq_disable();
    if(args[0])
    {
        cartographer_status=1;
    }
    else
    {
        cartographer_status=0;
    }
    irq_enable();
}
DECL_COMMAND(command_cartographer_stream,"cartographer_stream en=%u");
//切换激活状态
void
command_cartographer_set_threshold(uint32_t *args)
{
	trigger_freq=args[0];
	untrigger_freq=args[1];
}
DECL_COMMAND(command_cartographer_set_threshold,"cartographer_set_threshold trigger=%u untrigger=%u");

void
cartographer_home_task(void)
{
    /*if(!cartographer_home_flag)
    {
        if(cartographer_hometime==-1)
            cartographer_hometime=timer_read_time();
        if(cartographer_hometime+10000000>timer_read_time()){
            if(cartographer_hometime-10000000>timer_read_time())
                cartographer_hometime=timer_read_time();
            return;
        }
        cartographer_hometime=timer_read_time();
    }*/
    if(!cartographer_home_flag)
        return;
    if(trigger_method)
    {
	if(homing_freq==0)
	{
	    cartographer_hometime=timer_read_time();
	    homing_freq = read_channel();
	    return;
	}
	uint32_t time=timer_read_time();
	
	if(timer_is_before(time,cartographer_hometime+timer_from_us(dur*100)))
	{
	    return;
	}
	cartographer_hometime=time;
	uint32_t data=read_channel();
	if(current<6)
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
	for(int i=0;i<6;i++)
	    avr+=stack[i];
	avr=avr/6;
	if(max>trigger_threshold+avr && max>100)
	{
	    trsync_do_trigger(cartographer_ts, cartographer_trigger_reason);
	    homing_freq=0;
	    for(int i=0;i<6;i++)
	        stack[i]=0;
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
        
	    trsync_do_trigger(cartographer_ts, cartographer_trigger_reason);
	    gpio_out_write(led,1);	
        }
        else if(data<untrigger_freq)
            gpio_out_write(led,0);
        irq_enable();
    }
}
DECL_TASK(cartographer_home_task);
void
command_cartographer_home(uint32_t *args)
{
    cartographer_ts=trsync_oid_lookup(args[0]);
    cartographer_trigger_reason=args[1];
    cartographer_trigger_invert=args[2];
    trigger_threshold=args[3];
    trigger_method=args[4];
    homing_freq=0;
    start=0;
    max=0;
    current=0;
    cartographer_home_flag=1;
}
DECL_COMMAND(command_cartographer_home,"cartographer_home trsync_oid=%c trigger_reason=%c trigger_invert=%c threshold=%u trigger_method=%u");

void
command_cartographer_stop_home(uint32_t *args)
{
    cartographer_home_flag=0;
    cartographer_ts=NULL;
}
DECL_COMMAND(command_cartographer_stop_home,"cartographer_stop_home");

void
command_cartographer_base_read(uint32_t *args)
{
    uint8_t data_len=args[0];
    uint8_t offset=args[1];
    uint32_t f_count=32917500;
    uint16_t adc_count=55927;
    uint64_t data=((uint64_t)adc_count)<<32 | f_count;
    sendf("cartographer_base_data bytes=%*s offset=%hu", data_len, &data, offset);
}
DECL_COMMAND(command_cartographer_base_read,"cartographer_base_read len=%c offset=%hu");

void
cartographer_update_task(void)
{
    if((!cartographer_status)&&(!sched_check_wake(&cartographer_update)))
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
    #if CONFIG_FOR_K1
      idm_sleep(500);
    #endif
    if(data>trigger_freq)
    {
	gpio_out_write(led,1);	
    }
    else if(data<untrigger_freq)
        gpio_out_write(led,0);
}
DECL_TASK(cartographer_update_task);
