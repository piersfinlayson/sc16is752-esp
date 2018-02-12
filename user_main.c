#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_config.h"
#include "pin_map.h"
#include "brzo_i2c/brzo_i2c.h"

#define user_procTaskPrio        0
#define user_procTaskQueueLen    1
os_event_t    user_procTaskQueue[user_procTaskQueueLen];
static void user_procTask(os_event_t *events);

static volatile os_timer_t some_timer;

uint8_t step = 0;
uint32_t cycles = 0;
bool gpios_on = FALSE;
brzo_i2c_info i2c_info;
char *send_string = "Hello World";
unsigned char receive_buf[256];
uint8_t rbuf_ptr = 0;
uint32_t irq_cleared = 0;
uint32_t bytes_received = 0;
#define SC16IS752_ADDR 0x48
#define SDA_PIN    4
#define SCL_PIN    5
#define IRQ_PIN    12
#define RESET_PIN  13

bool ICACHE_FLASH_ATTR write_reg(uint8_t addr, uint8_t reg, uint8_t val) 
{
  uint8_t rc;
  uint8_t bytes[2];
  bool fn_rc = TRUE;

  bytes[0] = reg;
  bytes[1] = val;
  brzo_i2c_start_transaction_info(addr, 100, &i2c_info);
  brzo_i2c_write_info(bytes, 2, FALSE, &i2c_info);
  rc = brzo_i2c_end_transaction_info(&i2c_info);
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
  brzo_i2c_start_transaction_info(addr, 100, &i2c_info);
  brzo_i2c_write_info(bytes, 1, FALSE, &i2c_info);
  brzo_i2c_read_info(val, 1, FALSE, &i2c_info);
  rc = brzo_i2c_end_transaction_info(&i2c_info);
  if (rc)
  {
    fn_rc = FALSE;
    ets_printf("    read failure: %d\r\n", rc);
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
      // ets_printf("success\r\n");
      goto EXIT_LABEL;
    }
    else
    {
      ets_printf("    failed verify due to mismatch: expected 0x%02x got 0x%02x\r\n", val, rval);
    }
  }
  else
  {
    ets_printf("    failed read\r\n");
  }

EXIT_LABEL:

  return rc;
}

bool ICACHE_FLASH_ATTR set_reg(uint8_t addr, char *name, uint8_t uart, uint8_t ver, uint8_t reg, uint8_t val, bool pause)
{
  bool rc;
  bool fn_rc = TRUE;
  uint8_t areg;

  areg = (reg << 3) | (uart << 2);
  ets_printf("  %s reg: 0x%02x actual reg: 0x%02x val: 0x%02x\r\n", name, reg, areg, val);
  rc = write_reg(addr, areg, val);
  if (!rc)
  {
    fn_rc = FALSE;
  }
  else if (ver)
  {
    if (pause)
    {
      os_delay_us(1000);
    }
    rc = verify_reg(addr, areg, val);
    if (!rc)
    {
      fn_rc = FALSE;
    }
  }

  return fn_rc;
}

bool ICACHE_FLASH_ATTR byte_to_read()
{
  bool rc;
  uint8_t rval;

  rc = read_reg(SC16IS752_ADDR, 0x05<<3, &rval);
  if (rc)
  {
    if (!(rval & 1))
    {
      rc = FALSE;
      //ets_printf("nothing to read\r\n");
    }
    else
    {
      //ets_printf("data to read\r\n");
    }
  }
  else
  {
    //ets_printf("failed read\r\n");
  }

  return rc;
}

void ICACHE_FLASH_ATTR interrupt_handler(void *ptr)
{
  uint32_t gpio_status, gpio_st2;
  uint8_t gpio;
  uint8_t level;
  bool found = FALSE;
  bool rc = TRUE;
  uint8_t rval;

  // ets_printf("\r\n***!!! Interrupt received ...\r\n");

  gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  gpio_st2 = gpio_status;
  for (gpio = 0; gpio_st2 != 0; gpio_st2 >>= 1, gpio++)
  {
    if (gpio_st2 & 1)
    {
      found = TRUE;
      break;
    }
  }

  if (!found)
  {
    ets_printf("!!! Interrupt internal error (can't identify pin)\r\n");
    goto EXIT_LABEL;
  }

  if (gpio != IRQ_PIN)
  {
    ets_printf("!!! Interrupt receieved not on IRQ pin: %d\r\n", gpio);
    goto EXIT_LABEL;
  }

  // Disable interrupts on IRQ pin
  gpio_pin_intr_state_set(GPIO_ID_PIN(IRQ_PIN), GPIO_PIN_INTR_DISABLE);
    
  // Get level
  level = GPIO_INPUT_GET(GPIO_ID_PIN(IRQ_PIN));
  if (level)
  {
    //ets_printf("!!! Interrupt was IRQ pin clearing\r\n");
    irq_cleared++;
    goto EXIT_LABEL;
  }

  while (byte_to_read() && (rbuf_ptr < 255))
  {
    rc = read_reg(SC16IS752_ADDR, 0<<3, &rval);
    if (rc)
    {
      receive_buf[rbuf_ptr] = rval;
      //ets_printf("read byte 0x%02x\r\n", rval);
      rbuf_ptr++;
      bytes_received++;
    }
  }
  receive_buf[rbuf_ptr] = 0;

  ets_printf("\r\n!!! Received data: %s\r\n\r\n", receive_buf);
  rbuf_ptr = 0;

EXIT_LABEL:

  if (found)
  {
    // Clear interrupt
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);
    gpio_pin_intr_state_set(GPIO_ID_PIN(IRQ_PIN), GPIO_PIN_INTR_LOLEVEL);
  }

  ets_printf("\r\n");

  return;
}

void ICACHE_FLASH_ATTR register_interrupt()
{
  PIN_FUNC_SELECT(pin_mux[IRQ_PIN], pin_func[IRQ_PIN]);
  GPIO_DIS_OUTPUT(IRQ_PIN);

  ETS_GPIO_INTR_DISABLE();

  ETS_GPIO_INTR_ATTACH(interrupt_handler, NULL);

  gpio_register_set(GPIO_PIN_ADDR(IRQ_PIN),
                GPIO_PIN_INT_TYPE_SET(GPIO_PIN_INTR_DISABLE)  |
                GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_DISABLE) |
                GPIO_PIN_SOURCE_SET(GPIO_AS_PIN_SOURCE));
  
  // clear interrupt handler status writing a low to the output
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(IRQ_PIN));

  // enable interrupt for pin at low level
  gpio_pin_intr_state_set(GPIO_ID_PIN(IRQ_PIN), GPIO_PIN_INTR_LOLEVEL);

  ETS_GPIO_INTR_ENABLE();
}

void ICACHE_FLASH_ATTR init_i2c(void)
{
  ets_printf("  I2C stack init ... ");
  i2c_info.sda_pin = SDA_PIN;
  i2c_info.scl_pin = SCL_PIN;
#ifndef ARDUINO
  i2c_info.sda_pin_func = pin_func[SDA_PIN];
  i2c_info.scl_pin_func = pin_func[SCL_PIN];
  i2c_info.sda_pin_mux = pin_mux[SDA_PIN];
  i2c_info.scl_pin_mux = pin_mux[SCL_PIN];
#endif
  i2c_info.clock_stretch_time_out_usec = 100;
  brzo_i2c_setup_info(&i2c_info);
 
  ets_printf("success\r\n");

  return;
}

void ICACHE_FLASH_ATTR soft_reset_device(void)
{
  uint8_t rval;
  bool rc;
  int ii;

  ets_printf("  Reset SC16IS752 ... \r\n");
  set_reg(SC16IS752_ADDR, "IOControl", 0, 0, 0x0e, 0b00001000, FALSE);  // software reset, don't verify
  ets_printf("  Read LSR register\r\n");
  for (ii = 0; ii < 2; ii++)
  {
    // First read seems to fail, try a second - first read fails even if wait for 100ms-1s
    if (ii == 0)
    {
      ets_printf("  First read may fail:\r\n");
    }
    else
    {
      ets_printf("  Second should succeed:\r\n");
    }
    rc = read_reg(SC16IS752_ADDR, 0x05<<3, &rval);
    if (rc)
    {
      break;
    }
  }
  if (rc)
  {
    if (rval == 0x60)
    {
      ets_printf("    success\r\n");
    }
    else
    {
      ets_printf("  failed got 0x%02x\r\n", rval);
    }
  }
  else
  {
    ets_printf("  repeated read failed\r\n");
  }

  return;
}

void ICACHE_FLASH_ATTR init_device()
{
  bool rc;
  bool orc = TRUE;

  ets_printf("SC16IS752 init ... \r\n");

  // SPR is temporary storage for 1 byte
  // TCR/TLR RX and TX FIFO trigger levels unused (set using FCR)

  // Set basic registers
  rc = set_reg(SC16IS752_ADDR, "IER", 0, 1, 0x01, 0b00000001, FALSE);  // enable receive RHR interrupt, verify
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "FCR", 0, 0, 0x02, 0b00000001, FALSE);  // set rx, tx triggers to 8 chars, don't verify (read only reg), enable FIFO
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "LCR", 0, 1, 0x03, 0b00000011, FALSE);  // no parity, 1 stop bit, 8 bit word length
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "MCR", 0, 1, 0x04, 0b00000000, FALSE);  // divide by 1 clock
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "IODir", 0, 1, 0x0a, 0b00000011, FALSE);      // Set GPIOs 0/1 to output
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "IOIntEna", 0, 1, 0x0c, 0b00000000, FALSE);   // No interrupts for GPIOs
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "IOControl", 0, 1, 0x0e, 0b00000000, FALSE);  // GPIOs as GPIOs, unlatched
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "EFCR", 0, 1, 0x0f, 0b00000000, FALSE); // defaults
  if (rc == FALSE) {orc = FALSE;}

  // Set special registers
  rc = set_reg(SC16IS752_ADDR, "LCR", 0, 1, 0x03, 0b10000011, FALSE);  // enable special register set
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "DLL", 0, 1, 0x00, 48, FALSE);  // 2400 baud assuming 1.8432MHz crystal
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "DLH", 0, 1, 0x01, 0, FALSE);   // 2400 baud assuming 1.8432MHz crystal
  if (rc == FALSE) {orc = FALSE;}
  rc = set_reg(SC16IS752_ADDR, "LCR", 0, 1, 0x03, 0b00000011, FALSE);  // disable special register set
  if (rc == FALSE) {orc = FALSE;}

  // Don't bother with enhanced registers (set LCR to 0xbf)

  if (orc)
  {
    // ets_printf("  success\r\n");
  }
  else
  {
    ets_printf("  failed\r\n");
  }

EXIT_LABEL:

  return;
}

void ICACHE_FLASH_ATTR hard_reset_device()
{
  ets_printf("  Resetting device ... ");
  PIN_FUNC_SELECT(pin_mux[RESET_PIN], pin_func[RESET_PIN]);
  GPIO_OUTPUT_SET(GPIO_ID_PIN(RESET_PIN), 0);
  os_delay_us(1000);
  GPIO_OUTPUT_SET(GPIO_ID_PIN(RESET_PIN), 1);
  ets_printf("done\r\n");

  return;
}

void ICACHE_FLASH_ATTR some_timerfunc(void *arg)
{
  bool rc;
  uint8_t rval;
  int ii;
  char ccycles[16];
  char *ws;

  ets_printf("\r\n");
  switch (step)
  {
    case 0:
      ets_printf("Step %d - init I2C bus\r\n", step);
      init_i2c();
      register_interrupt();  // Need to do this something as a one-off...
      break;

    case 1:
      cycles++;
      ets_printf("Step %d - soft reset device\r\n", step);
      soft_reset_device();
      break;

    case 2:
      ets_printf("Step %d - initialize device\r\n", step);
      init_device();
      break;

    case 3:
      ets_printf("Step %d - turn GPIOs 0,1 on\r\n", step);
      set_reg(SC16IS752_ADDR, "IOState", 0, 1, 0x0b, 0xff, FALSE);   // Set GPIOs 0-1 on
      gpios_on = TRUE;
      break;
    
    case 4:
      ets_printf("Step %d - turn GPIOs 0,1 off\r\n", step);
      set_reg(SC16IS752_ADDR, "IOState", 0, 1, 0x0b, 0xfc, FALSE);   // Set GPIOs off
      gpios_on = FALSE;
      break;

    case 5:
      os_sprintf(ccycles, " #%d", cycles);
      ets_printf("Step %d - send \"%s%s\"\r\n", step, send_string, ccycles);
      os_sprintf(ccycles, " #%d\r\n", cycles);
      ets_printf("  ");
      for (ws = send_string; ws != NULL; )
      {
        for (ii = 0; ii < os_strlen(ws); ii++)
        {
          if ((ws[ii] != '\r') && (ws[ii] != '\n'))
          {
            ets_printf("%c", ws[ii]);
          }
          rc = write_reg(SC16IS752_ADDR, 0<<3, ws[ii]);
          if (!rc)
          {
            ets_printf("\r\n  failed sending 0x%02x %c\r\n", ws[ii], ws[ii]);
            goto EXIT_LABEL;
          }
        }
        if (ws == send_string)
        {
          ws = ccycles;
        }
        else
        {
          ws = NULL;
        }
      }
      ets_printf("\r\n");
      break;

    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
      ets_printf("Step %d - NOP\r\n", step);
      break;

    case 14:
      ets_printf("Step %d - hard reset device\r\n", step);
      hard_reset_device();
      rc = read_reg(SC16IS752_ADDR, 0x03<<3, &rval); // LCR
      ets_printf("  Checking device reset ... ");
      if (rval == 0x1d)  // default LCR value
      {
        ets_printf("success\r\n");
      }
      else
      {
        ets_printf("failure - LCR reg = 0x%02x\r\n", rval);
      }
      break;

    case 15:
      ets_printf("Step %d - Output stats\r\n", step);
      ets_printf("  Cycles:          %d\r\n", cycles);
      ets_printf("  IRQ cleared:     %d\r\n", irq_cleared);
      ets_printf("  Bytes received:  %d\r\n", bytes_received);
      break;
  
    default:
      ets_printf("Internal error\r\n");
      break;
  }

EXIT_LABEL:

  step++;
  if (step >= 16)
  {
    step = 1; // Don't bother to reinit the I2C stack
  }

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
