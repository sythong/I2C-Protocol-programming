//tine time_init() calibrates how many loop iterations are
// required for one milliseconds. It stores this in time_lpms.
// The routine time_wait(us) does the actual wait.

#define INPUT_GPIO_SCL 22
#define INPUT_GPIO_SDA 21
#define getSCL digitalRead(INPUT_GPIO_SCL)
#define getSDA digitalRead(INPUT_GPIO_SDA)

// Calibrate timing
#define TIME_COUNT (1000*1000) // less then 2G/1000
int time_lpms;
void time_init() {
  delay(1); // Feed watchdog
  uint32_t t0 = micros();
  for ( volatile uint32_t i = 0; i < TIME_COUNT; i++ ) /*skip*/ ;
  uint32_t t1 = micros();
  uint32_t dt = t1 - t0;
  time_lpms = 1000 * (TIME_COUNT) / dt;
  Serial.printf("Time: %u loops in %u us: %d loops/ms\n", TIME_COUNT, dt, time_lpms);
  delay(1); // Feed watchdog

  uint32_t t3 = micros();
  getSCL;
  uint32_t t4 = micros();
  uint32_t dt1 = t4 - t3;
  Serial.printf("Time: getSCL =  %u us \n", dt1);

#if 0
  // Test
  for ( int i = 10; i < 1000000; i *= 10 ) {
    uint32_t t0 = micros();
    time_wait_us(i);
    uint32_t t1 = micros();
    uint32_t dt = t1 - t0;
    Serial.printf("Time: %d ~ %u us\n", i, dt);
    delay(1); // Feed watchdog
  }
#endif
}

// Logging the trace of the statemachine
#define LOG_SIZE 1024
int     log_head = 0;
int     log_tail = 0;
char    log_data[LOG_SIZE];


void ICACHE_FLASH_ATTR log_char(char ch) {
  int newhead = (log_head + 1) % LOG_SIZE;
  if ( newhead == log_tail ) {
    // overflow
  } else {
    log_data[log_head] = ch;
    log_head = newhead;
  }
}


// The states of the statemachine tracking the I2C lines
#define I2CSTATE_UNKNOWN       1
#define I2CSTATE_ERROR         2
#define I2CSTATE_IDLE          3
#define I2CSTATE_STARTED       4
#define I2CSTATE_ADDRBIT_SCLLO 5
#define I2CSTATE_ADDRBIT_SCLHI 6
#define I2CSTATE_ADDRACK_SCLLO 7
#define I2CSTATE_ADDRACK_SCLHI 8
#define I2CSTATE_DATABIT_SCLLO 9
#define I2CSTATE_DATABIT_SCLHI 10
#define I2CSTATE_DATAACK_SCLLO 11
#define I2CSTATE_DATAACK_SCLHI 12

#define SLAVE_REGCOUNT              32   // Number of bytes exposed as registers over I2C
uint8_t slave_regs[32]; // The backing memory
uint8_t slave_cra;                  // The CRA or Current Register Address
uint8_t pos_byte;
uint8_t count_start = 0;
// Recording the state
typedef struct i2c_s {
  // Signal level
  int      state;   // Status of I2C state machine (see I2CSTATE_XXX)
  int      pulse;   // The number of SCL pulses seen so far
  uint32_t qus;     // Transaction start, later length, in us
  int      sdacap;  // State of SDA captured on rising SCL
  int      data;    // Bits of ADDR or DATA
  int      bitcnt;  // Bit count 0..7 of bits in data
  int      logbits; // Should individual bits be loggeds
  // Segment level
  int      addr;    // Last slave address on the line
  int      bytecnt; // Byte count in segment
};
i2c_s i2c;


// ===========================================================================================
// The main program


void setup() {
  delay(1000);
  pinMode(INPUT_GPIO_SCL, INPUT_PULLUP);
  pinMode(INPUT_GPIO_SDA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_GPIO_SCL), readTemp, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INPUT_GPIO_SDA), readTemp, CHANGE);
  Serial.begin(9600);
//  Serial.printf("\n\n\nWelcome to I2C-hacking v%d\n");
//  time_init();
  i2c.state = I2CSTATE_UNKNOWN;
}

int byte1, byte2, last_byte1, last_byte2;
bool done_read_temp = false;
long last_milis_time = 0;

void loop() {

  if (millis() - last_milis_time > 500)
  {
    if (done_read_temp)
    {
//      Serial.printf("result byte = %d %d \n", last_byte1, last_byte2);
      float temp = 0.1 * ((float)last_byte2 + 256.0);
      Serial.println(temp,1);
//      Serial.printf("Nhiet do  = %f \n", temp);
      done_read_temp = false;
    }
  }
}

void ICACHE_FLASH_ATTR readTemp()
{
  int scl = getSCL;
  int sda = getSDA;

  // START/STOP detector
  if ( i2c.state == I2CSTATE_ADDRBIT_SCLHI
       || i2c.state == I2CSTATE_ADDRACK_SCLHI
       || i2c.state == I2CSTATE_DATABIT_SCLHI
       || i2c.state == I2CSTATE_DATAACK_SCLHI
     ) { // SCL was high, is it still? Did SDA change?
    if ( scl == 1 && sda != i2c.sdacap ) {
      // SDA changed while CLK was high
      if ( sda == 0 ) {
        // SDA went low during CLK high: (repeated) START
        i2c.state = I2CSTATE_STARTED;
      } else {
        i2c.state = I2CSTATE_IDLE;
        //        Serial.printf("i2c address = %d\n", i2c.addr);
        //        Serial.printf("i2c data = %d\n", slave_cra);
      }
    }
  }

  switch (i2c.state) {
    case I2CSTATE_UNKNOWN:
      count_start = 0;
      if (scl == 1 && sda == 1) {
        i2c.state = I2CSTATE_IDLE;
      } else {
        i2c.state = I2CSTATE_ERROR;
      }
      break;

    case I2CSTATE_ERROR:
      if ( scl == 1 && sda == 1 ) {
        i2c.state = I2CSTATE_IDLE;
      }
      break;

    case I2CSTATE_IDLE:
      // SCL and SDA both high ("idle state")
      if ( scl == 1 ) {
        if ( sda == 1 ) {
          // SCL=1, SDA=1: Stay in IDLE
        } else {
          // SDA went low, while SCL stays high
          i2c.pulse = 0;
          i2c.qus = micros();
          i2c.addr = 0;
          count_start ++;
          i2c.state = I2CSTATE_STARTED;
        }
      }
      break;

    case I2CSTATE_STARTED:
      // SCL was high, SDA was low; but SCL must go down and SDA must be first bit of addr
      if (scl == 0 ) {
        i2c.pulse++;
        //        if( i2c.pulse)
        i2c.data = 0;
        i2c.bitcnt = 0;
        i2c.state = I2CSTATE_ADDRBIT_SCLLO;
        //        Serial.printf("i2c stated to i2c low");
      }
      break;

    case I2CSTATE_ADDRBIT_SCLLO:
      // Reading address bits. SCL is low.
      if ( scl == 1 ) {
        // SCL went high, so SDA has data (if it stays like this the full CLK period)
        i2c.sdacap = sda;
        i2c.state = I2CSTATE_ADDRBIT_SCLHI;
      }
      break;

    case I2CSTATE_ADDRBIT_SCLHI:
      // Reading address bits. SCL is high.
      if ( scl == 0 ) {
        // SCL went low, so we have a complete address bit
        i2c.pulse++;
        if ( i2c.sdacap ) i2c.data |= 1 << (7 - i2c.bitcnt);
        i2c.bitcnt++;
        if ( i2c.bitcnt < 8 ) {
          i2c.state = I2CSTATE_ADDRBIT_SCLLO;
        } else {
          // Received a complete address byte (but not yet the ack)
          i2c.addr = i2c.data;
          //          Serial.printf("i2c address = %d\n", i2c.addr);
          i2c.state = I2CSTATE_ADDRACK_SCLLO;
        }
      }
      break;

    case I2CSTATE_ADDRACK_SCLLO:
      // Reading address ack bit. SCL is low.
      if ( scl == 1 ) {
        // SCL went high, so SDA has data (if it stays like this the full CLK period)
        i2c.sdacap = sda;
        i2c.state = I2CSTATE_ADDRACK_SCLHI;
      }
      break;

    case I2CSTATE_ADDRACK_SCLHI:
      // Reading address ack bit. SCL is high.
      if ( scl == 0 ) {
        // SCL went low, so we have a complete address ack bit
        i2c.pulse++;
        i2c.bitcnt = 0;
        i2c.data = 0;
        i2c.bytecnt = 0;
        i2c.state = I2CSTATE_DATABIT_SCLLO;
      }
      break;

    case I2CSTATE_DATABIT_SCLLO:
      // Reading data bits. SCL is low.
      if ( scl == 1 ) {
        // SCL went high, so SDA has data (if it stays like this the full CLK period)
        i2c.sdacap = sda;
        i2c.state = I2CSTATE_DATABIT_SCLHI;
      }
      break;

    case I2CSTATE_DATABIT_SCLHI:
      // Reading data bits. SCL is high.
      if ( scl == 0 ) {
        // SCL went low, so we have a complete data bit
        i2c.pulse++;
        if ( i2c.sdacap ) i2c.data |= 1 << (7 - i2c.bitcnt);
        i2c.bitcnt++;
        if ( i2c.bitcnt < 8 ) {
          i2c.state = I2CSTATE_DATABIT_SCLLO;
        } else {
          // Received a complete data byte (but not yet the ack)
          if ( i2c.bytecnt == 0 ) {
            // First data byte goes to CRA ...
            //slave_cra = i2c.data;
            last_byte1 = byte1;
            byte1 = i2c.data;

          } else
          {
            last_byte2 = byte2;
            byte2 = i2c.data;
            slave_cra++;
            int interval = micros() - i2c.qus;
            //            if(count_start ==2)
            if ((byte1 == 31)) { // khi nao byte doc duoc 3 byte
              if (count_start >= 3) {
                done_read_temp = true;
                //              i2c.state = I2CSTATE_UNKNOWN;
                //              Serial.printf("interval= %d  -- count start = %d \n", interval, count_start);
//                Serial.printf("result byte = %d %d \n", last_byte1, last_byte2);
                //              float temp = 0.1 * ((float)last_byte2 + 256.0);
                //              Serial.printf("Nhiet do  = %f \n", temp);
                //              last_byte2 = byte2 = 0;
                //              last_byte1 = byte1 = 0;
                //              i2c.bytecnt = 0;
                //    return 1;
              }
            }
          }
          i2c.state = I2CSTATE_DATAACK_SCLLO;
        }
      }
      break;

    case I2CSTATE_DATAACK_SCLLO:
      // Reading data ack bit. SCL is low.
      if ( scl == 1 ) {
        // SCL went high, so SDA has data (if it stays like this the full CLK period)
        i2c.sdacap = sda;
        i2c.state = I2CSTATE_DATAACK_SCLHI;
      }
      break;

    case I2CSTATE_DATAACK_SCLHI:
      // Reading data ack bit. SCL is high.
      if ( scl == 0 ) {
        // SCL went low, so we have a complete data ack bit
        i2c.pulse++;
        i2c.bitcnt = 0;
        i2c.data = 0;
        i2c.bytecnt++;
        i2c.state = I2CSTATE_DATABIT_SCLLO;
      }
      break;
  }
  // return 0;
}
