#ifndef NRF24L01_H
#define NRF24L01_H
/*nrf24l01: MSbit to LSbit, LSbyte to MSbyte*/
#include <stdio.h>
#include <stdint.h>

#define STARTUP_DELAY                 150             /*in milliseconds*/
#define POWER_DOWN_DELAY              2
#define STANDBYI_DELAY                2
#define PRX_MODE_DELAY                100
#define ADDRESS_WIDTH_DEFAULT         5               /*address width in bytes, for default value*/
#define RF_CHANNEL_DEFAULT            32        
#define RF_DATARATE_DEFAULT           1000            /*250, 1000, 2000*/
#define RF_PWR_DEFAULT                6               /*0, -6, -12, -18*/
#define STATIC_PAYLOAD_WIDTH_DEFAULT  1               /*for static payload mode, configurable between 1 and 32 bytes for PRX device ONLY (RX_PW_Pn, n for data pipe n)(no register for payload length in PTX device)*/
#define NUMBER_OF_DP_DEFAULT          1               /*number of datapipes, 1 to 6*/ 
#define RETRANSMIT_DELAY_DEFAULT      500
#define RETRANSMIT_COUNT_DEFAULT      2
 
#define OPEN                          1
#define CLOSE                         0
#define ENABLE                        1
#define DISABLE                       0
#define SPI_OFF                       1
#define SPI_ON                        0
#define CE_OFF                        0
#define CE_ON                         1

#define CONFIG_REGISTER_DEFAULT       0X08
#define EN_AA_REGISTER_DEFAULT        0X3F
#define EN_RXADDR_REGISTER_DEFAULT    0X00
#define SETUP_AW_REGISTER_DEFAULT     0X03
#define SETUP_RETR_REGISTER_DEFAULT   0X03
#define RF_CH_REGISTER_DEFAULT        0X02
#define RF_SETUP_REGISTER_DEFAULT     0X0E
#define STATUS_REGISTER_DEFAULT       0X0E
#define MAXIMUM_NUMBER_OF_DATAPIPES   6

#define POWER_DOWN                    0X00
#define STANDBYI                      0X01
#define STANDBYII                     0X02
#define PTX                           0X03
#define PRX                           0X04
#define DEVICE_NOT_INITIALIZED        0X05

#define TRANSMITTER                   0X00
#define RECEIVER                      0X01
#define POWER_SAVING                  0X02
#define TURN_OFF                      0X03

#define RESET                         1
#define NO_RESET                      0
#define NO_ACK_MODE                   1
#define ACK_MODE                      0
#define TRANSMIT_BEGIN                1
#define TRANSMIT_FAIL                 0
#define TRANSMIT_IN_PROGRESS          0
#define TRANSMIT_DONE                 1
#define TRANSMIT_FAILED               0XFF
#define OPERATION_DONE                1
#define OPERATION_ERROR               0
#define RECEIVE_FIFO_EMPTY            2
#define TX_BUFFER                     1
#define RX_BUFFER                     0

/*bits definition section*/
#define MASK_RX_DR          6               /*mask interrupt caused by RX_DR: 1 interrupt not reflected on IRQ pin (IRQ is active low), inside CONFIG register*/
#define MASK_TX_DS          5               /*mask interrupt caused by TX_DS: 1 interrupt not reflected on IRQ pin (IRQ is active low), inside CONFIG register*/
#define MASK_MAX_RT         4               /*mask interrupt caused by MAX_RT means maximum number of retransmissions reached: 1 interrupt not reflected on IRQ pin (IRQ is active low), inside CONFIG register*/
#define EN_CRC              3               /*enale CRC, forced high if one of the bits in EN_AA is high, inside CONFIG register*/
#define CRCO                2               /*CRC encoding scheme, 0 is 1 byte, 1 is 2 bytes, inside CONFIG register*/
#define PWR_UP              1               /*1 is power up, inside CONFIG register*/
#define PRIM_RX             0               /*RX/TX control, 1: PRX, inside CONFIG register*/
#define ENAA_P5             5               /*enable auto acknowledgement data pipe 5*/
#define ENAA_P4             4
#define ENAA_P3             3
#define ENAA_P2             2
#define ENAA_P1             1
#define ENAA_P0             0
#define ERX_P5              5               /*part of EN_RXADDR, enable data pipe 5*/
#define ERX_P4              4
#define ERX_P3              3
#define ERX_P2              2
#define ERX_P1              1
#define ERX_P0              0
#define AW_1                1               /*RX/TX address field width, 00 illegal, 01 3 bytes, 10 4 bytes, 11 5 bytes*/
#define AW_0                0
#define ARD_3               7               /*auto retransmit delay, 0000 250us, 0001 500us ...> 1111 4000us*/
#define ARD_2               6
#define ARD_1               5
#define ARD_0               4
#define ARC_3               3               /*auto retransmit count, 0000 retransmit deisabled, 1111 up to 15 retransmit on failure of AA. (inside SETUP_RETR register)*/
#define ARC_2               2
#define ARC_1               1
#define ARC_0               0
#define RF_CH_6             6               /*sets the frequencvy channel nRF24L01+ operates on*/
#define RF_CH_5             5
#define RF_CH_4             4
#define RF_CH_3             3
#define RF_CH_2             2
#define RF_CH_1             1
#define RF_CH_0             0
#define CONT_WAVE           7               /*enables continuous carrier transmit when high*/
#define RF_DR_LOW           5               /*sets the RF data rate to 250kbps*/
#define PLL_LOCK            4               /*force PLL lock signal. used for testing ONLY*/
#define RF_DR_HIGH          3               /*select between high speed data rates and works ONLY when RF_DR_LOW is 0. 0 for 1Mbps, 1 for 2Mbps*/
#define RF_PWR_1            2
#define RF_PWR_0            1
#define RX_DR               6               /*IRQ for new packet in RX FIFO (newly received)*/
#define TX_DS               5               /*IRQ for ACK received in TX mode*/
#define MAX_RT              4 
#define RX_P_NO_2           3
#define RX_P_NO_1           2
#define RX_P_NO_0           1
#define TX_FULL             0
#define PLOS_CNT_3          7               /*inside OBSERVE_TX register, counts the total number of retransmissions since last channel change. reset by writing to RF_CH*/
#define PLOS_CNT_2          6
#define PLOS_CNT_1          5
#define PLOS_CNT_0          4
#define ARC_CNT_3           3               /*inside OBSERVE_TX register, counts the number of retransmissions for current transaction. reset by initiating new transaction*/
#define ARC_CNT_2           2
#define ARC_CNT_1           1
#define ARC_CNT_0           0
#define RPD                 0               /*received power detector, if received power is less than -64dbm, RPD = 0*/
#define TX_REUSE            6
#define TX_FULL             5
#define TX_EMPTY            4
#define RX_FULL             1
#define RX_EMPTY            0
#define DPL_P5              5
#define DPL_P4              4
#define DPL_P3              3
#define DPL_P2              2
#define DPL_P1              1
#define DPL_P0              0                 /*must be set on PTX in dynamic payload length mode*/
#define EN_DPL              2                 /*set to enable dynamic payload length*/
#define EN_ACK_PAY          1                 /*used to enable auto acknowledgement with payload in PRX (inside FEATURE register)*/
#define EN_DYN_ACK          0                 /**/

/*registers definition section*/
#define CONFIG_ADDRESS              0X00
#define EN_AA_ADDRESS               0X01              /*enable auto acknowledgement feature*/
#define EN_RXADDR_ADDRESS           0X02              /*register containing bits to enable 6 data pipes individually*/
#define SETUP_AW_ADDRESS            0X03              /*address field length is configured in here to be 3, 4 or 5 bytes long*/
#define SETUP_RETR_ADDRESS          0X04              /*setup ARC bits to configure auto retransmission count*/
#define RF_CH_ADDRESS               0X05
#define RF_SETUP_ADDRESS            0X06
#define STATUS_ADDRESS              0X07              /*contains RX_DR, TX_DS, MAX_RT, RX_P_NO, TX_FULL, send R_REGISTER then NOP to read*/ 
#define OBSERVE_TX_ADDRESS          0X08              /*contains ARC_CNT and PLOS_CNT, two counters for retransmission. these counters could be used to assess the network quality*/
#define RPD_REG_ADDRESS             0X09
#define RX_ADDR_P0_ADDRESS          0X0A              /*the address for PRX device. if a packet contains this address, enhanced shockburst starts validating the packet*/
#define RX_ADDR_P1_ADDRESS          0X0B              /*a total of 6 unique addresses could be assigned to a PRX device (Multiceiver feature)*/
#define RX_ADDR_P2_ADDRESS          0X0C              /*these addresses must NOT be the same*/
#define RX_ADDR_P3_ADDRESS          0X0D
#define RX_ADDR_P4_ADDRESS          0X0E
#define RX_ADDR_P5_ADDRESS          0X0F
#define TX_ADDR_ADDRESS             0X10              /*40 bits long register, transmit address, used for a PTX device only. configure address legth in SETUP_AW register. set RX_ADDR_P0 equal to this address to handle automatic acknowledge*/
#define RX_PW_P0_ADDRESS            0X11              /*these registers are for setting the static payload length in static payload length mode (receiver side)*/
#define RX_PW_P1_ADDRESS            0X12
#define RX_PW_P2_ADDRESS            0X13
#define RX_PW_P3_ADDRESS            0X14
#define RX_PW_P4_ADDRESS            0X15
#define RX_PW_P5_ADDRESS            0X16
#define FIFO_STATUS_ADDRESS         0X17
#define DYNPD_ADDRESS               0X1C              /*on receiver side (RX mode), this register must be set to enable dynamic payload length. a PTX in dynamic mode, must have the DYNPD_P0 set*/
#define FEATURE_ADDRESS             0X1D              /*contains the EN_DPL bit to enable dynamic payload length*/

/*commands definition section*/
#define R_REGISTER          0X00              /*read commmand and STATUS registers, 5 bit register map address*/
#define W_REGISTER          0X20              /*write commmand and STATUS registers, 5 bit register map address, executable in POWER DOWN or STANDBY modes only*/
#define R_RX_PAYLOAD        0X61              /*read RX payload, 1-32 bytes. read operation starts at byte 0. payload is deleted from FIFO after its read*/
#define W_TX_PAYLOAD        0XA0              /*write TX payload, starts at byte 0, 1-32 bytes*/
#define FLUSH_TX            0XE1              /*flush TX FIFO, used in TX mode*/
#define FLUSH_RX            0XE2              /*flush RX FIFO, used in RX mode*/
#define REUSE_TX_PL         0XE3              /*used for a PTX device, reuse last transmitted payload for an exact number. alternative to auto retransmission*/
#define R_RX_PL_WID         0X60              /*command for receiver side, in order to read the payload length in dynamic payload length mode*/
#define W_ACK_PAYLOAD       0XA0              /*used in RX mode, to write payload in TX FIFO and later transmit the payloads along with ACK packet to PTX, if DPL is enabled*/
#define W_TX_PAYLOAD_NOACK  0XB0              /*used in TX mode, disables AUTOACK on this specific packet. must be first enabled in FEATURE register by setting the EN_DYN_ACK bit. if used, PTX will not wait for ACK and goes directly to standby I*/
#define NOP_CMD             0XFF              /*might be used to read the status register*/

void nrf24_reset();                            
void nrf24_device(uint8_t device_mode, uint8_t reset_state);
uint8_t SPI_send_command(uint8_t command);          
void pinout_Initializer();         
void SPI_Initializer();
void nrf24_mode(uint8_t mode);
void nrf24_SPI(uint8_t input);
void nrf24_CE(uint8_t input);
void nrf24_address_width(uint8_t address_width);
void nrf24_rf_channel(uint8_t rf_channel);
void nrf24_rf_power(uint8_t rf_power);
void nrf24_rf_datarate(uint8_t rf_datarate);
void nrf24_read(uint8_t address, uint8_t *value, uint8_t data_length, uint8_t spi_state);
void nrf24_write(uint8_t address, uint8_t *value, uint8_t data_length, uint8_t spi_state);
void delay_function(uint32_t duration_ms);
void nrf24_crc_configuration(uint8_t crc_enable, uint8_t crc_encoding_scheme);
void nrf24_interrupt_mask(uint8_t rx_mask, uint8_t tx_mask, uint8_t max_rt_mask);
void nrf24_datapipe_enable(uint8_t number_of_datapipes);
void nrf24_prx_static_payload_width(uint8_t static_payload_width, uint8_t number_of_datapipes);
void nrf24_datapipe_address_configuration();
void nrf24_datapipe_ptx(uint8_t datapipe_number);
void nrf24_automatic_retransmit_setup(uint16_t delay_time, uint8_t retransmit_count);
void nrf24_auto_acknowledgment_setup(uint8_t datapipe);
void nrf24_dynamic_payload(uint8_t state, uint8_t datapipe);
void nrf24_device(uint8_t device_mode, uint8_t reset_state);
void nrf24_send_payload(uint8_t *payload, uint8_t payload_width);
uint8_t nrf24_receive(uint8_t *payload, uint8_t payload_width);
uint8_t nrf24_transmit(uint8_t *payload, uint8_t payload_width, uint8_t acknowledgement_state);
uint8_t nrf24_transmit_status();
void nrf24_dynamic_ack(uint8_t state);
uint8_t nrf24_flush(uint8_t fifo_select);

#endif


/*global variables related to this file*/
static uint8_t SPI_command;                                       /*1 byte spi command*/
static uint8_t register_current_value;                            /*in order to change some bits of internal registers or to check their content*/
static uint8_t register_new_value;                                /*used to write new value to nrf24l01+ registers*/
static uint8_t write_pointer;                                     /*used as an input for read and write functions (as a pointer)*/
static uint8_t current_address_width;                             /*current address width for receiver pipe addresses (up to 6 pipes), from 3 to 5 bytes*/
static uint8_t reset_flag = 0;                                    /*reset flag lets the software know if the nrf24l01+ has ever been reset or not*/
static uint8_t current_mode = DEVICE_NOT_INITIALIZED;             /*current mode of operation: DEVICE_NOT_INITIALIZED, PRX, PTX, STANDBYI, STANDBYII, POWER_DOWN*/
static uint8_t current_payload_width;                             /*payload width could be from 1 to 32 bytes, in either dynamic or static forms*/
static uint8_t current_acknowledgement_state = NO_ACK_MODE;       
static uint8_t dynamic_payload = DISABLE;

/*2 dimensional array of pipe addresses (5 byte address width) by default. you can change addresses using a new array later.
  Pipe 1 address could be anything. pipe 3 to 6 addresses share the first 4 bytes with pipe 2 and only differ in byte 5*/
uint8_t datapipe_address[MAXIMUM_NUMBER_OF_DATAPIPES][ADDRESS_WIDTH_DEFAULT] = {
  {0X20, 0XC3, 0XC2, 0XC1, 0XA0},
  {0X20, 0XC3, 0XC2, 0XC1, 0XA1},
  {0X20, 0XC3, 0XC2, 0XC1, 0XA2},
  {0X20, 0XC3, 0XC2, 0XC1, 0XA3},
  {0X20, 0XC3, 0XC2, 0XC1, 0XA4},
  {0X20, 0XC3, 0XC2, 0XC1, 0XA5}
};

/*function to enable or disable dynamic acknowledge. if enabled, you can disable acknowledge
   on a specific payload with W_TX_PAYLOAD_NOACK or enable acknowledge using W_TX_PAYLOAD commands.
   if disabled, you cannot disable acknowledging a payload. manipulates EN_DYN_ACK inside FEATURE*/
void nrf24_dynamic_ack(uint8_t state)
{
  if (state == ENABLE)
  {
    nrf24_read(FEATURE_ADDRESS, &register_current_value, 1, CLOSE);
    register_new_value = register_current_value | (1 << EN_DYN_ACK);
    nrf24_write(FEATURE_ADDRESS, &register_new_value, 1, CLOSE);
  }
  else
  {
    nrf24_read(FEATURE_ADDRESS, &register_current_value, 1, CLOSE);
    register_new_value = register_current_value & (~(1 << EN_DYN_ACK));
    nrf24_write(FEATURE_ADDRESS, &register_new_value, 1, CLOSE);
  }
}

/*function for PTX device to transmit 1 to 32 bytes of data, used for both dynamic payload length
   and static payload length methods. acknowledgemet state could be NO_ACK_MODE or ACK_MODE*/
uint8_t nrf24_transmit(uint8_t *payload, uint8_t payload_width, uint8_t acknowledgement_state)
{
  nrf24_read(STATUS_ADDRESS, &register_current_value, 1, CLOSE);         /*in order to check TX_FIFO status*/
  if ((!(register_current_value & (1 << TX_FULL))) && (current_mode == PTX))
  {
    current_acknowledgement_state = acknowledgement_state;      /*setting the acknowledgement state to either NO_ACK or ACK, based on input*/
    if (dynamic_payload == ENABLE)
      payload_width = current_payload_width;
    nrf24_send_payload(payload, payload_width);                 /*the actual function to send data*/
    return (TRANSMIT_BEGIN);                                     /*TX FIFO is not full and nrf24l01+ mode is standby ii or ptx*/
  }
  else
  {
    return (TRANSMIT_FAIL);            /*TX FIFO full or wrong mode*/
  }
}

/*used by nrf24_transmit function to send the actual data*/
void nrf24_send_payload(uint8_t *payload, uint8_t payload_width)
{
  nrf24_SPI(SPI_ON);
  if (current_acknowledgement_state == NO_ACK_MODE)
    SPI_command = W_TX_PAYLOAD_NOACK;
  else
    SPI_command = W_TX_PAYLOAD;
  SPI_send_command(SPI_command);
  for (; payload_width; payload_width--)
  {
    SPI_command = *payload;
    SPI_send_command(SPI_command);
    payload++;
  }
  nrf24_SPI(SPI_OFF);
}

/*reports back transmit status: TRANSMIT_DONE, TRANSMIT_FAILED (in case of reaching maximum number of retransmits in auto acknowledgement mode)
  and TRANSMIT_IN_PROGRESS, if neither flags are set. automatically resets the '1' flags.*/
uint8_t nrf24_transmit_status()
{
  nrf24_read(STATUS_ADDRESS, &register_current_value, 1, CLOSE);      /*status register is read to check TX_DS flag*/
  if (register_current_value & (1 << TX_DS))                          /*if the TX_DS == 1, */
  {
    nrf24_write(STATUS_ADDRESS, &register_current_value, 1, CLOSE);   /*reseting the TX_DS flag. as mentioned by datasheet, writing '1' to a flag resets that flag*/
    return TRANSMIT_DONE;
  }
  else if (register_current_value & (1 << MAX_RT))
  {
    nrf24_write(STATUS_ADDRESS, &register_current_value, 1, CLOSE);   /*reseting the MAX_RT flag. as mentioned by datasheet, writing '1' to a flag resets that flag*/
    return TRANSMIT_FAILED;
  }
  else
    return TRANSMIT_IN_PROGRESS;
}

/*the receive function output is used as a polling method to check the received data inside RX FIFOs. 
If there is any data available, it will be loaded inside payload array*/
uint8_t nrf24_receive(uint8_t *payload, uint8_t payload_width)
{
  if (current_mode == PRX)
  {
    nrf24_read(STATUS_ADDRESS, &register_current_value, 1, CLOSE);
    if (register_current_value & (1 << RX_DR))                         /*if received data is ready inside RX FIFO*/
    {
      if(dynamic_payload == DISABLE)                                    /*if dynamic payload width is disabled, use the static payload width and ignore the input*/
        payload_width = current_payload_width;
        
      nrf24_SPI(SPI_ON);                                                /*sending the read payload command to nrf24l01+*/                          
      SPI_command = R_RX_PAYLOAD;
      SPI_send_command(SPI_command);
       
      for (; payload_width; payload_width--)
      {
        SPI_command = NOP_CMD;
        *payload = SPI_send_command(SPI_command);
        payload++;
      }
      nrf24_SPI(SPI_OFF); 
      nrf24_read(FIFO_STATUS_ADDRESS, &register_current_value, 1, CLOSE);   /*in order to check the RX_EMPTY flag*/
      if(register_current_value & (1 << RX_EMPTY))                        /*if the RX FIFO is empty, reset the RX_DR flag inside STATUS register*/
      {
        nrf24_read(STATUS_ADDRESS, &register_current_value, 1, CLOSE);
        register_new_value = register_current_value | (1 << RX_DR);
        nrf24_write(STATUS_ADDRESS, &register_new_value, 1, CLOSE); 
      }      
      return OPERATION_DONE;
    }
    else
    {
      return RECEIVE_FIFO_EMPTY;
    }
  }
  else
    return OPERATION_ERROR;
}

/*function which uses TX_FLUSH or RX_FLUSH command to flush the fifo buffers. if successful, output is OPERATION_DONE.
   if not successful (wrong input or wrong mode of operation) output will be OPERATION_ERROR*/
uint8_t nrf24_flush(uint8_t fifo_select)
{
  switch (fifo_select)
  {
    case TX_BUFFER:
      if (current_mode == PTX)
      {
        nrf24_SPI(SPI_ON);
        SPI_command = FLUSH_TX;
        SPI_send_command(SPI_command);
        nrf24_SPI(SPI_OFF);
        return OPERATION_DONE;
      }
      else
        return OPERATION_ERROR;
    case RX_BUFFER:
      if (current_mode == PRX)
      {
        nrf24_SPI(SPI_ON);
        SPI_command = FLUSH_RX;
        SPI_send_command(SPI_command);
        nrf24_SPI(SPI_OFF);
        return OPERATION_DONE;
      }
      else
        return OPERATION_ERROR;
    default:
      return OPERATION_ERROR;
  }
}

/*must be called atleast once, which happens with calling nrf24_device function*/
void nrf24_reset()
{
  reset_flag = 1;
  nrf24_CE(CE_OFF);
  register_new_value = CONFIG_REGISTER_DEFAULT;
  nrf24_write(CONFIG_ADDRESS, &register_new_value, 1, CLOSE);
  register_new_value = EN_AA_REGISTER_DEFAULT;
  nrf24_write(EN_AA_ADDRESS, &register_new_value, 1, CLOSE);
  register_new_value = EN_RXADDR_REGISTER_DEFAULT;
  nrf24_write(EN_RXADDR_ADDRESS, &register_new_value, 1, CLOSE);
  register_new_value = SETUP_AW_REGISTER_DEFAULT;
  nrf24_write(SETUP_AW_ADDRESS, &register_new_value, 1, CLOSE);
  register_new_value = RF_CH_REGISTER_DEFAULT;
  nrf24_write(RF_CH_ADDRESS, &register_new_value, 1, CLOSE);
  register_new_value = RF_SETUP_REGISTER_DEFAULT;
  nrf24_write(RF_SETUP_ADDRESS, &register_new_value, 1, CLOSE);
  register_new_value = STATUS_REGISTER_DEFAULT;
  nrf24_write(STATUS_ADDRESS, &register_new_value, 1, CLOSE);

  nrf24_mode(PTX);
  nrf24_flush(TX_BUFFER);
  nrf24_mode(PRX);
  nrf24_flush(RX_BUFFER);

  nrf24_read(STATUS_ADDRESS, &register_current_value, 1, CLOSE);
  register_new_value = register_current_value | (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT);
  nrf24_write(STATUS_ADDRESS, &register_new_value, 1, CLOSE);
  
  nrf24_interrupt_mask(ENABLE, ENABLE, ENABLE);
  nrf24_crc_configuration(ENABLE, 1);
  nrf24_address_width(ADDRESS_WIDTH_DEFAULT);
  nrf24_rf_datarate(RF_DATARATE_DEFAULT);
  nrf24_rf_power(RF_PWR_DEFAULT);
  nrf24_rf_channel(RF_CHANNEL_DEFAULT);
  nrf24_datapipe_enable(NUMBER_OF_DP_DEFAULT);
  /*nrf24_datapipe_address_configuration();*/
  /*nrf24_datapipe_ptx(1);*/
  nrf24_prx_static_payload_width(STATIC_PAYLOAD_WIDTH_DEFAULT, NUMBER_OF_DP_DEFAULT);
  nrf24_automatic_retransmit_setup(RETRANSMIT_DELAY_DEFAULT, RETRANSMIT_COUNT_DEFAULT);
  nrf24_auto_acknowledgment_setup(NUMBER_OF_DP_DEFAULT);
  nrf24_dynamic_payload(DISABLE, NUMBER_OF_DP_DEFAULT);
  nrf24_dynamic_ack(ENABLE);
}

/*used by firmware to set the nrf24 mode in TRANSMITTER, RECEIVER, POWER_SAVING or TURN_OFF states, and reseting the device
  if it has not been done yet. This is the initializer, and everything starts by calling nrf24_device first.It has a higher
  level of abstraction than nrf24_mode and must be used by user*/
void nrf24_device(uint8_t device_mode, uint8_t reset_state)
{
  SPI_Initializer();
  pinout_Initializer();
  delay_function(STARTUP_DELAY);

  if ((reset_state == RESET) || (reset_flag == 0))
  {
    nrf24_reset();
  }

  switch (device_mode)
  {
    case TRANSMITTER:
      nrf24_mode(POWER_DOWN);
      nrf24_interrupt_mask(ENABLE, DISABLE, DISABLE);                /*disabling tx interrupt mask*/
      nrf24_mode(PTX);
      break;
    case RECEIVER:
      nrf24_mode(POWER_DOWN);
      nrf24_interrupt_mask(DISABLE, ENABLE, ENABLE);                /*disabling rx interrupt mask*/
      nrf24_mode(PRX);
      delay_function(PRX_MODE_DELAY);                              /*100ms for PRX mode*/
      break;
    case POWER_SAVING:
      nrf24_mode(POWER_DOWN);
      nrf24_interrupt_mask(ENABLE, ENABLE, ENABLE);
      nrf24_mode(STANDBYI);
      break;
    case TURN_OFF:
      nrf24_mode(POWER_DOWN);
      nrf24_interrupt_mask(ENABLE, ENABLE, ENABLE);
      break;
    default:
      nrf24_mode(POWER_DOWN);
      nrf24_interrupt_mask(ENABLE, ENABLE, ENABLE);
      break;
  }
}

/*setting automatic retransmit delay time and maximum number of retransmits*/
void nrf24_automatic_retransmit_setup(uint16_t delay_time, uint8_t retransmit_count)
{
  register_new_value = 0x00;
  for (; (delay_time > 250) && (register_new_value < 0X0F); delay_time -= 250)
    register_new_value++;
  register_new_value <<= ARD_0;
  if ((retransmit_count > 0) && (retransmit_count < 16))
    register_new_value |= retransmit_count;
  else
    register_new_value |= 0;
  nrf24_write(SETUP_RETR_ADDRESS, &register_new_value, 1, CLOSE);
}

/*setting auto acknoledgement on datapipes*/
void nrf24_auto_acknowledgment_setup(uint8_t datapipe)
{
  if (datapipe < 7)
    register_new_value = (1 << datapipe) - 1;
  nrf24_write(EN_AA_ADDRESS, &register_new_value, 1, CLOSE);
}

/*turns on or off the dynamic payload width capability*/
void nrf24_dynamic_payload(uint8_t state, uint8_t datapipe)
{
  nrf24_auto_acknowledgment_setup(datapipe);                        /*setting auto acknowledgment before setting dynamic payload*/
  nrf24_read(FEATURE_ADDRESS, &register_current_value, 1, CLOSE);
  if (state == ENABLE)
  {
    register_new_value = register_current_value | (1 << EN_DPL);    /*EN_DPL bit turns dynamic payload width on or off on all datapipes*/
    nrf24_write(FEATURE_ADDRESS, &register_new_value, 1, CLOSE);
    if (datapipe < 7)
      register_new_value = (1 << datapipe) - 1;                       /*turning on dynamic payload width on chosen datapipes, using DYNPD register*/
    nrf24_write(DYNPD_ADDRESS, &register_new_value, 1, CLOSE);
    dynamic_payload = ENABLE;
  }
  else
  {
    register_new_value = register_current_value & (~(1 << EN_DPL));
    nrf24_write(FEATURE_ADDRESS, &register_new_value, 1, CLOSE);
    dynamic_payload = DISABLE;
  }
}

/*on nrf24l01+ there is only one address for PTX device which must be the same as PRX data pipe address 0*/
void nrf24_datapipe_ptx(uint8_t datapipe_number)
{
  nrf24_write(TX_ADDR_ADDRESS, &datapipe_address[datapipe_number - 1][0], current_address_width, CLOSE);
}

/*setting the 6 datapipe addresses using the datapipe_address[][]*/
void nrf24_datapipe_address_configuration()
{
  uint8_t address = RX_ADDR_P0_ADDRESS;
  for (uint8_t counter = 0; counter < 6; counter++)
  {
    nrf24_write(address, &datapipe_address[counter][0], current_address_width, CLOSE);
    address++;
  }
}

/*function to change static payload width, from 1 to 32 bytes in each payload*/
void nrf24_prx_static_payload_width(uint8_t static_payload_width, uint8_t number_of_datapipes)
{
  for (uint8_t address = RX_PW_P0_ADDRESS; number_of_datapipes; number_of_datapipes--)
  {
    nrf24_write(address, &static_payload_width, 1, CLOSE);
    address++;
  }
  current_payload_width = static_payload_width;
}

/*datapipes are turned on and off using EN_RXADDR register, PRX datapipe addresses are located in RX_ADDR_Pn, TX address is located inside TX_ADDR*/
void nrf24_datapipe_enable(uint8_t number_of_datapipes)
{
  register_new_value = (1 << number_of_datapipes) - 1;
  nrf24_write(EN_RXADDR_ADDRESS, &register_new_value, 1, CLOSE);
}

/*function to set the nrf24l01+ address width, from 3 to 5 bytes*/
void nrf24_address_width(uint8_t address_width)
{
  if ((address_width <= 5) && (address_width >= 3))
  {
    write_pointer = address_width - 2;
  }
  else
  {
    write_pointer = 3;
  }
  nrf24_write(SETUP_AW_ADDRESS, &write_pointer, 1, CLOSE);                    /*5 bytes is the maximum address width available*/
  current_address_width = address_width;
}

/*datarate settings, you can choose between 2mbps, 1mbps, 250kbps*/
void nrf24_rf_datarate(uint8_t rf_datarate)
{
  nrf24_read(RF_SETUP_ADDRESS, &register_current_value, 1, CLOSE);
  register_current_value &= ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));
  switch (rf_datarate)
  {
    case 2000:
      register_new_value = register_current_value | (1 << RF_DR_HIGH);
      break;
    case 1000:
      register_new_value = register_current_value;
      break;
    case 250:
      register_new_value = register_current_value | (1 << RF_DR_LOW);
      break;
    default:
      register_new_value = register_current_value;
      break;
  }
  nrf24_write(RF_SETUP_ADDRESS, &register_new_value, 1, CLOSE);
}

/*nrf24l01+ RF power settings. 0dbm, -6dbm, -12dbm, -18dbm*/
void nrf24_rf_power(uint8_t rf_power)
{
  nrf24_read(RF_SETUP_ADDRESS, &register_current_value, 1, CLOSE);
  register_current_value &= ~((1 << RF_PWR_1) | (1 << RF_PWR_0));
  switch (rf_power)
  {
    case 0:
      register_new_value = register_current_value | ((1 << RF_PWR_1) | (1 << RF_PWR_0));
      break;
    case 6:
      register_new_value = register_current_value | (1 << RF_PWR_1);
      break;
    case 12:
      register_new_value = register_current_value | (1 << RF_PWR_0);
      break;
    case 18:
      register_new_value = register_current_value;
      break;
    default:
      register_new_value = register_current_value | (1 << RF_PWR_1);
      break;
  }
  nrf24_write(RF_SETUP_ADDRESS, &register_new_value, 1, CLOSE);
}

/*nrf24l01+ RF channel selection, from 1 to 125*/
void nrf24_rf_channel(uint8_t rf_channel)
{
  if ((rf_channel <= 125) && (rf_channel >= 1))
  {
    uint8_t write_pointer = rf_channel;
    nrf24_write(RF_CH_ADDRESS, &write_pointer, 1, CLOSE);
  }
  else
  {
    uint8_t write_pointer = 1;
    nrf24_write(RF_CH_ADDRESS, &write_pointer, 1, CLOSE);
  }
}

/*interrupt mask settings. 3 seperate masks for RX, TX, and RT (maximum numbers of retransmission reached*/
void nrf24_interrupt_mask(uint8_t rx_mask, uint8_t tx_mask, uint8_t max_rt_mask)
{
  nrf24_read(CONFIG_ADDRESS, &register_current_value, 1, CLOSE);
  if (rx_mask)
    register_new_value = (register_current_value) | (1 << MASK_RX_DR);
  else
    register_new_value &= (~(1 << MASK_RX_DR));
  if (tx_mask)
    register_new_value |= (1 << MASK_TX_DS);
  else
    register_new_value &= (~(1 << MASK_TX_DS));
  if (max_rt_mask)
    register_new_value |= (1 << MASK_MAX_RT);
  else
    register_new_value &= (~(1 << MASK_MAX_RT));

  nrf24_write(CONFIG_ADDRESS, &register_new_value, 1, CLOSE);
}

/*enabling or disabling crc in payload; setting crc encoding scheme between 1 or 2 bytes*/
void nrf24_crc_configuration(uint8_t crc_enable, uint8_t crc_encoding_scheme)
{
  nrf24_read(CONFIG_ADDRESS, &register_current_value, 1, CLOSE);
  if (crc_enable)
    register_new_value = (register_current_value) | (1 << EN_CRC);
  else
    register_new_value &= (~(1 << EN_CRC));
  if (crc_encoding_scheme == 2)
    register_new_value |= (1 << CRCO);
  else
    register_new_value &= (~(1 << CRCO));

  nrf24_write(CONFIG_ADDRESS, &register_new_value, 1, CLOSE);
}

/*mode selector: power down, standby i, standby ii, ptx, prx. used by nrf24_device function*/
void nrf24_mode(uint8_t mode)
{
  nrf24_read(CONFIG_ADDRESS, &register_current_value, 1, CLOSE);
  switch (mode)
  {
    case POWER_DOWN:
      nrf24_CE(CE_OFF);
      register_new_value = (register_current_value) & (~(1 << PWR_UP));
      delay_function(POWER_DOWN_DELAY);
      break;
    case STANDBYI:                                 /*standby I is defined by 'PWR_UP = 1' and 'CE pin LOW'*/
      nrf24_CE(CE_OFF);
      register_new_value = (register_current_value) | (1 << PWR_UP);
      delay_function(STANDBYI_DELAY);
      break;
    case STANDBYII:                                 /*standby ii is related to a ptx device*/
      nrf24_CE(CE_ON);
      register_new_value = ((register_current_value) | (1 << PWR_UP)) & (~(1 << PRIM_RX));
      delay_function(STANDBYI_DELAY);
      break;
    case PTX:
      nrf24_CE(CE_ON);
      register_new_value = ((register_current_value) | (1 << PWR_UP)) & (~(1 << PRIM_RX));
      delay_function(STANDBYI_DELAY);
      break;
    case PRX:
      nrf24_CE(CE_ON);
      register_new_value = (register_current_value) | (1 << PWR_UP) | (1 << PRIM_RX);
      delay_function(STANDBYI_DELAY);
      break;
    default:
      nrf24_CE(CE_OFF);
      register_new_value = (register_current_value) & (~(1 << PWR_UP));
      delay_function(POWER_DOWN_DELAY);
      break;
  }
  nrf24_write(CONFIG_ADDRESS, &register_new_value, 1, CLOSE);
  current_mode = mode;
}

/*reads the number of bytes (data_length) from the register in nrf24l01+ (address) and stores them inside an array (value),
  then closes the spi connection (spi_state = CLOSE) or leaves it open (spi_state = OPEN)*/
void nrf24_read(uint8_t address, uint8_t *value, uint8_t data_length, uint8_t spi_state)
{
  nrf24_SPI(SPI_ON);
  SPI_command = R_REGISTER | address;    /*in order to read CONFIG, then change one bit*/
  SPI_send_command(SPI_command);
  SPI_command = NOP_CMD;
  for (; data_length ; data_length--)
  {
    *value = SPI_send_command(SPI_command);
    value++;
  }
  if (spi_state == CLOSE)
    nrf24_SPI(SPI_OFF);
}

/*writes the number of bytes (data_length) from an array (value) inside registers in nrf24l01+ (address),
  then closes the spi connection (spi_state = CLOSE) or leaves it open (spi_state = OPEN)*/
void nrf24_write(uint8_t address, uint8_t *value, uint8_t data_length, uint8_t spi_state)
{
  nrf24_SPI(SPI_ON);
  SPI_command = W_REGISTER | address;    /*in order to read CONFIG, then change one bit*/
  SPI_send_command(SPI_command);
  for (; data_length ; data_length--)
  {
    SPI_command = *value;
    value++;
    SPI_send_command(SPI_command);
  }
  if (spi_state == CLOSE)
    nrf24_SPI(SPI_OFF);
}

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>


// Function prototypes
void pinout_Initializer(void);
void SPI_Initializer(void);
uint8_t SPI_send_command(uint8_t command);
void nrf24_SPI(uint8_t input);
void nrf24_CE(uint8_t input);
void delay_function(uint32_t duration_ms);

// Implementation of specific functions for the AVR microcontroller
void pinout_Initializer(void) {
    // Set up pins for NRF24L01 connection
    // CE -> PA7, CSN -> PC2, SCK -> PC0, MOSI -> PC1, MISO -> PC3
    
    // Set CE (PA7) as output
    PORTA.DIRSET = PIN7_bm;
    PORTA.OUTCLR = PIN7_bm; // Initially low
    
    // Set CSN (PC2) as output
    PORTC.DIRSET = PIN2_bm;
    PORTC.OUTSET = PIN2_bm; // Initially high (inactive)
    
    // Configure SPI pins
    PORTC.DIRSET = PIN0_bm | PIN1_bm; // SCK and MOSI as outputs
    PORTC.DIRCLR = PIN3_bm;           // MISO as input
}

void SPI_Initializer(void) {
    // Initialize SPI in master mode
    SPI0.CTRLA = SPI_MASTER_bm | SPI_ENABLE_bm;
    SPI0.CTRLB = SPI_SSD_bm;          // Manage slave select manually
    SPI0.INTCTRL = 0;                 // No interrupts
}

uint8_t SPI_send_command(uint8_t command) {
    SPI0.DATA = command;
    while (!(SPI0.INTFLAGS & SPI_IF_bm)); // Wait for transmission complete
    return SPI0.DATA;                     // Return received data
}

void nrf24_SPI(uint8_t input) {
    if (input == SPI_ON) {
        PORTC.OUTCLR = PIN2_bm; // CSN low (active)
        _delay_us(10);
    } else {
        _delay_us(10);
        PORTC.OUTSET = PIN2_bm; // CSN high (inactive)
    }
}

void nrf24_CE(uint8_t input) {
    if (input == CE_ON) {
        PORTA.OUTSET = PIN7_bm; // CE high (active)
    } else {
        PORTA.OUTCLR = PIN7_bm; // CE low (inactive)
    }
}

void delay_function(uint32_t duration_ms) {
    for (uint32_t i = 0; i < duration_ms; i++) {
        _delay_ms(1);
    }
}

int main(void) {
    uint8_t rx_data[5];  // Buffer for received data
    
    // Initialize NRF24L01
    nrf24_device(RECEIVER, RESET);
    
    // Configure address
    nrf24_address_width(5);  // 5-byte address
    
    // Set the same channel and data rate as the transmitter
    nrf24_rf_channel(76);       // Use channel 76 (2.476 GHz)
    nrf24_rf_datarate(1000);    // 1 Mbps data rate
    
    // Enable auto-acknowledgment and configure static payload width
    nrf24_auto_acknowledgment_setup(1);
    nrf24_prx_static_payload_width(5, 1);  // 5 bytes payload, pipe 0
    
    // Setup to receive on pipe 0
    nrf24_datapipe_enable(1);  // Enable pipe 0
    
    // Configure receive address (must match transmitter's address)
    nrf24_datapipe_address_configuration();
    
    // Configure LED for visual feedback (assuming PC5 is connected to an LED)
    PORTC.DIRSET = PIN5_bm;
    PORTC.OUTCLR = PIN5_bm;  // LED off initially
    
    // Main loop
    while (1) {
        // Check if data is available
        if (nrf24_receive(rx_data, 5) == OPERATION_DONE) {
            // Toggle LED to indicate data received
            PORTC.OUTTGL = PIN5_bm;
        }
        
        // Small delay
        _delay_ms(10);
    }
    
    return 0;
}