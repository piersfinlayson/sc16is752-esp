#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_config.h"

#define user_procTaskPrio        0
#define user_procTaskQueueLen    1
os_event_t    user_procTaskQueue[user_procTaskQueueLen];
static void user_procTask(os_event_t *events);

static volatile os_timer_t some_timer;

uint8_t step;
bool gpios_on = FALSE;
#define SC16IS752_ADDR 0x48
#define SDA_PIN    4
#define SCL_PIN    5
#define IRQ_PIN    12
#define RESET_PIN  13

void ICACHE_FLASH_ATTR init_i2c(void)
{
  ets_printf("I2C stack init ... ");
  brzo_i2c_setup(100);
  ets_printf("success\r\n");
  return;
}

bool ICACHE_FLASH_ATTR write_reg(uint8_t addr, uint8_t reg, uint8_t val) 
{
  uint8_t rc;
  uint8_t bytes[2];
  bool fn_rc = TRUE;

  bytes[0] = reg;
  bytes[1] = val;
  brzo_i2c_start_transaction(addr, 100);
  brzo_i2c_write(bytes, 2, FALSE);
  rc = brzo_i2c_end_transaction();
  if (rc)
  {
    fn_rc = FALSE;
    goto EXIT_LABEL;
  }

  fn_rc = TRUE;

EXIT_LABEL:

  return fn_rc;  
}

bool ICACHE_FLASH_ATTR read_reg(uint8_t addr, uint8_t reg, uint8_t *val) 
{
  uint8_t rc;
  uint8_t bytes[1];
  bool fn_rc = TRUE;

  bytes[0] = reg;
  brzo_i2c_start_transaction(addr, 100);
  brzo_i2c_write(bytes, 1, FALSE);
  brzo_i2c_read(val, 1, FALSE);
  rc = brzo_i2c_end_transaction();
  if (rc)
  {
    fn_rc = FALSE;
    ets_printf("read failure: %d\r\n", rc);
    goto EXIT_LABEL;
  }

EXIT_LABEL:

  return fn_rc;  
}

bool ICACHE_FLASH_ATTR verify_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
  bool rc;
  uint8_t rval;

  rc = read_reg(addr, reg, &rval);
  if (rc)
  {
    if (val == rval)
    {
      ets_printf("success\r\n");
      goto EXIT_LABEL;
    }
    else
    {
      ets_printf("failed read mismatch: expected 0x%02x got 0x%02x\r\n", val, rval);
    }
  }
  else
  {
    ets_printf("failed read\r\n");
  }

EXIT_LABEL:

  return rc;
}

void ICACHE_FLASH_ATTR set_reg(uint8_t addr, char *name, uint8_t uart, uint8_t ver, uint8_t reg, uint8_t val)
{
  bool rc;
  uint8_t areg;

  areg = (reg << 3) | (uart << 2);
  ets_printf("  %s reg: 0x%02x actual reg: 0x%02x val: 0x%02x\r\n", name, reg, areg, val);
  rc = write_reg(addr, areg, val);
  if (ver)
  {
    rc = verify_reg(addr, areg, val);
  }
}

void ICACHE_FLASH_ATTR soft_reset_device(void)
{

  ets_printf("Reset SC16IS752 ... \r\n");
  set_reg(SC16IS752_ADDR, "IOControl", 0, 0, 0x0e, 0b00001000);  // software reset, don't verify

  return;
}

void ICACHE_FLASH_ATTR init_device()
{

  ets_printf("SC16IS752 init ... \r\n");

  // SPR is temporary storage for 1 byte
  // TCR/TLR RX and TX FIFO trigger levels unused (set using FCR)

  // Set basic registers
  set_reg(SC16IS752_ADDR, "IER", 0, 1, 0x01, 0b00000001);  // enable receive holding register interrupt, verify
  set_reg(SC16IS752_ADDR, "FCR", 0, 0, 0x02, 0b00000000);  // set rx trigger to 8 chars, don't verify (read only reg)
  set_reg(SC16IS752_ADDR, "LCR", 0, 1, 0x03, 0b00000011);  // no parity, 1 stop bit, 8 bit word length
  set_reg(SC16IS752_ADDR, "MCR", 0, 1, 0x04, 0b00000000);  // divide by 1 clock
  set_reg(SC16IS752_ADDR, "IODir", 0, 1, 0x0a, 0b00000011);      // Set GPIOs 0/1 to output
  set_reg(SC16IS752_ADDR, "IOIntEna", 0, 1, 0x0c, 0b00000000);   // No interrupts for GPIOs
  set_reg(SC16IS752_ADDR, "IOControl", 0, 1, 0x0e, 0b00000000);  // GPIOs as GPIOs, unlatched
  set_reg(SC16IS752_ADDR, "EFCR", 0, 1, 0x0f, 0b00000000); // defaults

  // Set special registers
  set_reg(SC16IS752_ADDR, "LCR", 0, 1, 0x03, 0b10000011);  // enable special register set
  set_reg(SC16IS752_ADDR, "DLL", 0, 1, 0x00, 48);  // 2400 baud
  set_reg(SC16IS752_ADDR, "DLH", 0, 1, 0x01, 0);   // 2400 baud
  set_reg(SC16IS752_ADDR, "LCR", 0, 1, 0x03, 0b00000011);  // disable special register set

  // Don't bother with enhanced registers (set LCR to 0xbf)

EXIT_LABEL:

  return;
}

void ICACHE_FLASH_ATTR some_timerfunc(void *arg)
{
  uint8_t rc;

  switch (step)
  {

  }

  // Initialise everything if not yet successfully initalised
  if (!inited)
  {
    init_i2c();
    soft_reset_device();
    init_device();

    inited = TRUE;
    goto EXIT_LABEL;
  }

  if (!gpios_on)
  {
    set_reg(SC16IS752_ADDR, "IOState", 0, 1, 0x0b, 0xff);   // Set GPIOs 0-1 on
    gpios_on = TRUE;
  }
  else
  {
    set_reg(SC16IS752_ADDR, "IOState", 0, 1, 0x0b, 0xfc);   // Set GPIOs off
    gpios_on = FALSE;
  }



  // XXX Flash LEDs, send and receive data

EXIT_LABEL:

  return;
}

//Do nothing function
static void ICACHE_FLASH_ATTR
user_procTask(os_event_t *events)
{
    os_delay_us(10);
}

//Init function 
void ICACHE_FLASH_ATTR
user_init()
{
    //Disarm timer
    os_timer_disarm(&some_timer);

    //Setup timer
    os_timer_setfn(&some_timer, (os_timer_func_t *)some_timerfunc, NULL);

    //Arm the timer
    //&some_timer is the pointer
    //1000 is the fire time in ms
    //0 for once and 1 for repeating
    os_timer_arm(&some_timer, 1000, 1);
    
    //Start os task
    system_os_task(user_procTask, user_procTaskPrio,user_procTaskQueue, user_procTaskQueueLen);
}
