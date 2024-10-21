/* 
 * File:   SPI.h
 * Author: Ganji
 *  ____    _    _   _     _ ___ 
   / ___|  / \  | \ | |   | |_ _|
  | |  _  / _ \ |  \| |_  | || | 
  | |_| |/ ___ \| |\  | |_| || | 
   \____/_/   \_\_| \_|\___/|___| 
 *
 * Created on September 12, 2024, 7:30 AM
 */

#ifndef SPI_H
#define	SPI_H

#ifdef	__cplusplus
extern "C" {
#endif



#include <18F67J94.h>
#device ADC=16
#device ICD=TRUE

#use delay(CLOCK=16M, CRYSTAL=16M)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#use rs232(baud=9600, parity=N, xmit=PIN_E5, rcv=PIN_E4, bits=8, stream=EPS) //EPS DATA ACQUISITION
#use rs232(baud=9600, parity=N, xmit=PIN_C6, rcv=PIN_C7, bits=8, stream=EXT) //MAIN RAB Rear access board 
#use rs232(baud=57600, parity=N, xmit=PIN_D2, rcv=PIN_D3, bits=8, stream=COM, FORCE_SW) //MAIN COM Communication, send CW data 
#use rs232(baud=57600, parity=N, xmit=PIN_F7, rcv=PIN_F6, bits=8, stream=CAM, FORCE_SW) //MAIN CAM Communicationx
#use spi(MASTER, CLK=PIN_E1, DI=PIN_E0, DO=PIN_E6,  BAUD=10000, BITS=8, STREAM=MAIN_FM, MODE=0) //MAIN flash memory port
#use spi(MASTER, CLK=PIN_B2, DI=PIN_B5, DO=PIN_B4,  BAUD=10000, BITS=8, STREAM=COM_FM, MODE=0) //COM shared flash memory port
#use spi(MASTER, CLK=PIN_A3, DI=PIN_A0, DO=PIN_A1,  BAUD=10000, BITS=8, STREAM=ADCS_FM, MODE=0) //ADCS shared flash memory port, Camera module (ovcam,mvcam)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//SPI Stream alter name 
#define SPIPORT MAIN_FM
#define SPIPORT2 COM_FM
#define SPIPORT3 ADCS_FM    

//Flash memory chip select pins and mux control 
#define CS_PIN_1 PIN_E2 //OBC_FLASH_SELECT
#define CS_PIN_2 PIN_B3 //COM_CHIP_SELECT
#define CS_PIN_3 PIN_A2 //ADCS_CHIP_SELECT
#define MX_PIN_1 PIN_G2 //OVCAM_MUX_SELECT
#define MX_PIN_2 PIN_G3 //MVCAM_MUX_SELECT
#define MX_PIN_3 PIN_A5 //ADCS_MUX_SELECT
#define MX_PIN_4 PIN_C4 //COM_MUX_SELECT


//mt25q flash memory command assigment
#define READ_ID              0x9F
#define READ_STATUS_REG      0x05 
#define READ_DATA_BYTES      0x13  //0x03 for byte
#define ENABLE_WRITE         0x06
#define WRITE_PAGE           0x12  //0x02 for 3byte
#define ERASE_SECTOR         0xDC  //0xD8 for 3byte
#define ERASE_4KB_SUBSECTOR  0x21
#define ERASE_32KB_SUBSECTOR 0x5C
#define DIE_ERASE            0xC4
#define FAST_READ            0x0B

//memory maping     
#define SHUTDOWN_COUNT_ADDRESS 0x00100010

//digtal control pins 
#define EN_SUP_3V3_1 PIN_B0
#define EN_SUP_3V3_2 PIN_G1
#define EN_SUP_3V3_DAQ PIN_D0
#define EN_SUP_UNREG PIN_B1
#define EN_SUP_5V0 PIN_D1
#define KILL_SWITCH PIN_A4
#define MVCAM_PWR PIN_G0
#define OVCAM_PWR PIN_D7
    
    
void WRITE_ENABLE_OF(){
 output_low(CS_PIN_1);
 
 spi_xfer(SPIPORT,ENABLE_WRITE);                //Send 0x06
 output_high(CS_PIN_1);  
 return;
}

void WRITE_ENABLE_OF_COM(){
 output_low(CS_PIN_2);
 
 spi_xfer(SPIPORT,ENABLE_WRITE);                //Send 0x06
 output_high(CS_PIN_2);  
 return;
}

void WRITE_DATA_NBYTES(unsigned int32 ADDRESS, unsigned int8 data[], unsigned char data_number) {
    fprintf(EXT,"WRITE ADDRESS: 0x%08lx\n", ADDRESS);  // Print address as hex
    unsigned int8 adsress[4];
    // Byte extraction for a 32-bit address
    adsress[0]  = (unsigned int8)((ADDRESS >> 24) & 0xFF);
    adsress[1]  = (unsigned int8)((ADDRESS >> 16) & 0xFF);
    adsress[2]  = (unsigned int8)((ADDRESS >> 8) & 0xFF);
    adsress[3]  = (unsigned int8)(ADDRESS & 0xFF);
    WRITE_ENABLE_OF();  // Enable write operation

    // Lower CS to select the SPI device
    output_low(CS_PIN_1);
    delay_us(2);  // Small delay for stabilization
    // Send WRITE command and address
    spi_xfer(SPIPORT, WRITE_PAGE);
    spi_xfer(SPIPORT, adsress[0]);
    spi_xfer(SPIPORT, adsress[1]);
    spi_xfer(SPIPORT, adsress[2]);
    spi_xfer(SPIPORT, adsress[3]);
    // Write data bytes
    for (int i = 0; i < data_number; i++) {
        spi_xfer(SPIPORT, data[i]);  // Send data byte
        fprintf(EXT,"%02X ", data[i]);    // Print each byte as hex (optional)
    }
    
    output_high(CS_PIN_1);  // Deselect SPI device
    
    fprintf(EXT,"\n%d BYTES WRITTEN!\n", data_number);

}

void WRITE_DATA_NBYTES_COM(unsigned int32 ADDRESS, unsigned int8 data[], unsigned char data_number) {
    fprintf(EXT,"WRITE ADDRESS: 0x%08lx\n", ADDRESS);  // Print address as hex
    unsigned int8 adsress[4];
    // Byte extraction for a 32-bit address
    adsress[0]  = (unsigned int8)((ADDRESS >> 24) & 0xFF);
    adsress[1]  = (unsigned int8)((ADDRESS >> 16) & 0xFF);
    adsress[2]  = (unsigned int8)((ADDRESS >> 8) & 0xFF);
    adsress[3]  = (unsigned int8)(ADDRESS & 0xFF);
    WRITE_ENABLE_OF_COM();  // Enable write operation

    // Lower CS to select the SPI device
    output_low(CS_PIN_2);
    delay_us(2);  // Small delay for stabilization
    // Send WRITE command and address
    spi_xfer(SPIPORT, WRITE_PAGE);
    spi_xfer(SPIPORT, adsress[0]);
    spi_xfer(SPIPORT, adsress[1]);
    spi_xfer(SPIPORT, adsress[2]);
    spi_xfer(SPIPORT, adsress[3]);
    // Write data bytes
    for (int i = 0; i < data_number; i++) {
        spi_xfer(SPIPORT, data[i]);  // Send data byte
        fprintf(EXT,"%02X ", data[i]);    // Print each byte as hex (optional)
    }
    
    output_high(CS_PIN_2);  // Deselect SPI device5
    
    fprintf(EXT,"\n%d BYTES WRITTEN!\n", data_number);

}

 
void READ_DATA_NBYTES(unsigned int32 ADDRESS, unsigned char *Data_return, unsigned short data_number) {
    unsigned int8 adsress[4];
   
    // Byte extraction for a 32-bit address
    adsress[0]  = (unsigned int8)((ADDRESS >> 24) & 0xFF);
    adsress[1]  = (unsigned int8)((ADDRESS >> 16) & 0xFF);
    adsress[2]  = (unsigned int8)((ADDRESS >> 8) & 0xFF);
    adsress[3]  = (unsigned int8)(ADDRESS & 0xFF);

    output_low(CS_PIN_1);  // Select SPI device

    // Send READ DATA COMMAND (0x13 or appropriate for your flash chip)
    spi_xfer(SPIPORT, READ_DATA_BYTES);
    // Send address bytes
    spi_xfer(SPIPORT, adsress[0]);
    spi_xfer(SPIPORT, adsress[1]);
    spi_xfer(SPIPORT, adsress[2]);
    spi_xfer(SPIPORT, adsress[3]);
    // Read the requested number of bytes
    for (int i = 0; i < data_number; i++) {
        Data_return[i] = spi_xfer(SPIPORT, 0x00);  // Send dummy byte to receive data
        fprintf(EXT,"%c", Data_return[i]);           // Print each byte as hex
    }

    output_high(CS_PIN_1);  // Deselect SPI device after reading
    fprintf(EXT,"\n");
}


void READ_CHIP_ID_OF(int8 *chip_id) {
    output_low(CS_PIN_1);  // Lower the CS PIN
    spi_xfer(SPIPORT, READ_ID);  // READ ID COMMAND (0x9F)
    
    // Receive 8 bytes of chip ID
    for (int i = 0; i < 8; i++) {
        chip_id[i] = spi_xfer(SPIPORT, 0x00);  // Send dummy bytes to receive data
    }

    output_high(CS_PIN_1);  // Raise CS PIN back
}


void startup_freeze(){
    delay_ms(2000);
    fprintf(EXT, "POWER ON!\n");
}

void RTC_initialize(){
        setup_lcd(LCD_DISABLED);
    rtc_time_t read_clock;
    setup_rtc(RTC_ENABLE | RTC_CLOCK_SOSC | RTC_CLOCK_INT, 0);
    rtc_read(&read_clock);

}

void uart_repeater() {
    char received_data;

    while (TRUE) {
        // Check if data is available on the EPS stream
        if (kbhit(EPS)) {
            // Read one byte from the EPS stream
            received_data = fgetc(EPS);

            // Send the received byte to the EXT stream
            fputc(received_data, EXT);
        }
    }
}

//#define SHUTDOWN_COUNT_ADDRESS  0x00000500  // Address where shutdown count is stored

int8 update_shutdown_count(void) {
    fprintf(EXT, "Shutdown count started\n");

    unsigned int8 shutdown_count[1];
    READ_DATA_NBYTES(SHUTDOWN_COUNT_ADDRESS, shutdown_count, 1);
    delay_ms(10);

    fprintf(EXT, "Read shutdown count: %u\n", shutdown_count[0]);

    // Check if the shutdown count is uninitialized (0xFF)
    if (shutdown_count[0] == 0xFF) {
        shutdown_count[0] = 0;  // Initialize to 0 if uninitialized
        fprintf(EXT, "Shutdown count uninitialized, setting to 0\n");
    }

    shutdown_count[0] += 1;  // Increment the shutdown count
    WRITE_DATA_NBYTES(SHUTDOWN_COUNT_ADDRESS, shutdown_count, 1);
    delay_ms(10);

    // Read back to verify it was written correctly
    unsigned int8 verify_count[1];
    READ_DATA_NBYTES(SHUTDOWN_COUNT_ADDRESS, verify_count, 1);
    if (shutdown_count[0] == verify_count[0]) {
        fprintf(EXT, "Shutdown count successfully updated: %u\n", verify_count[0]);
    } else {
        fprintf(EXT, "Failed to update shutdown count. Read back: %u\n", verify_count[0]);
    }

    return shutdown_count[0];
}



void set_clock(rtc_time_t &date_time)
{

   date_time.tm_year=0000;
   date_time.tm_mon=00;
   date_time.tm_mday=00;
   date_time.tm_wday=00;
   date_time.tm_hour=00;
   date_time.tm_min=00;
   date_time.tm_sec=0; 

}
//hak thuah spilt on that thang enough 
// Main menu functions
void handle_flash_memories() {
    char flash_option;
    fprintf(EXT, "pressed option d: Check Flash Memories\n\n");
    fprintf(EXT, "Please choose which flash memory to work on (a, b, c):\n");
    fprintf(EXT, "press a: MAIN flash memory\n");
    fprintf(EXT, "press b: COM shared flash memory\n");
    fprintf(EXT, "press c: ADCS shared flash memory\n");

    flash_option = fgetc(EXT);

    switch (flash_option) {
        case 'a':
           // READ_DATA_NBYTES();
            break;
        case 'b':
            fprintf(EXT, "COM shared flash memory chosen\n");
            // Implement COM shared flash memory handling
            break;
        case 'c':
            fprintf(EXT, "ADCS shared flash memory chosen\n");
            // Implement ADCS shared flash memory handling
            break;
        default:
            fprintf(EXT, "Invalid flash memory option. Please try again.\n");
            break;
    }
}

void handle_main_flash_memory() {
    char main_flash_option;
    unsigned int32 address;
    unsigned char data[32];
    unsigned char data_length;

    fprintf(EXT, "MAIN flash memory chosen\n");
    fprintf(EXT, "press a: Read ID of the chip\n");
    fprintf(EXT, "press b: Write data set in specified address\n");
    fprintf(EXT, "press c: Read data set in specified address\n");
    fprintf(EXT, "press d: Return to MAIN MENU\n");

    main_flash_option = fgetc(EXT);

    switch (main_flash_option) {
        case 'a':
            fprintf(EXT, "Started reading chip ID of MAIN flash memory\n");
            int8 chip_id[8];
            READ_CHIP_ID_OF(chip_id);  // Replace with actual function
            break;
        case 'b':
            fprintf(EXT, "Write data set in specified address\n");
            fprintf(EXT, "Enter address (32-bit hexadecimal): ");
            scanf("%x", &address);
            fprintf(EXT, "Enter data length (1 to 32): ");
//            scanf("%d", &data_length);
            fprintf(EXT, "Enter data bytes (space-separated hexadecimal values): ");
            for (unsigned char i = 0; i < data_length; i++) {
                scanf("%x", &data[i]);
            }
            WRITE_DATA_NBYTES(address, data, data_length);
            fprintf(EXT, "Data written to specified address.\n");
            break;
        case 'c':
            fprintf(EXT, "Read data set in specified address\n");
            fprintf(EXT, "Enter your specified address: ");
            scanf("%x", &address);
            fprintf(EXT, "Enter your specified address length : ");
//            scanf("%d", &data_length);
//            READ_DATA_NBYTES(address,data_length);  // Replace with actual function
            break;
        case 'd':
            return;
        default:
            fprintf(EXT, "Invalid MAIN flash memory option. Please try again.\n");
            break;
    }
}

void handle_set_time() {
    char handle_set_time_option;
    fprintf(EXT, "Settings of RTC chosen\n");
    fprintf(EXT, "    press a: to reset the RTC /all current time will be set zero/\n");
    fprintf(EXT, "    press b: display current time\n");
    handle_set_time_option = fgetc(EXT);

    switch (handle_set_time_option) {
        case 'a':
    rtc_time_t write_clock, read_clock;
    rtc_read(&read_clock);
    fprintf(EXT, "Now time is\n");
    fprintf(EXT, "\r%02u/%02u/20%02u %02u:%02u:%02u", read_clock.tm_mon, read_clock.tm_mday, read_clock.tm_year, read_clock.tm_hour, read_clock.tm_min, read_clock.tm_sec);
    fprintf(EXT, "Time changing function activated\n");
    set_clock(write_clock);
    rtc_write(&write_clock);
    fprintf(EXT, "Time successfully changed. Current time is:\n");
    rtc_read(&read_clock);
    fprintf(EXT, "\r%02u/%02u/20%02u %02u:%02u:%02u", read_clock.tm_mon, read_clock.tm_mday, read_clock.tm_year, read_clock.tm_hour, read_clock.tm_min, read_clock.tm_sec);
    break;
        case 'b':
            rtc_read(&read_clock);
    fprintf(EXT, "\r%02u/%02u/20%02u %02u:%02u:%02u", read_clock.tm_mon, read_clock.tm_mday, read_clock.tm_year, read_clock.tm_hour, read_clock.tm_min, read_clock.tm_sec);
    break;
        case 'x':
            break;
            return;
        default:
            fprintf(EXT, "Invalid IO option. Please try again.\n");
    break;
            
}
}

void handle_io_control() {
    char io_option;
    int8 state_of_pin;

    fprintf(EXT, "IO control chosen\n");

    // Check and display the state of each pin before providing options
    state_of_pin = input_state(EN_SUP_3V3_1);
    fprintf(EXT, "    press a: Toggle EN_SUP_3V3_1 /is currently/");
    if(state_of_pin == 1 ){
        fprintf(EXT, "HIGH\n");
    }else if(state_of_pin == 0){
        fprintf(EXT, "LOW\n");
    }else {
        fprintf(EXT, "Invalid\n"); 
    }
    state_of_pin = input_state(EN_SUP_3V3_2);
    fprintf(EXT, "    press b: Toggle EN_SUP_3V3_2 /is currently/");
    if(state_of_pin == 1 ){
        fprintf(EXT, "HIGH\n");
    }else if(state_of_pin == 0){
        fprintf(EXT, "LOW\n");
    }else {
        fprintf(EXT, "Invalid\n"); 
    }
     state_of_pin = input_state(EN_SUP_3V3_DAQ);
    fprintf(EXT, "    press c: Toggle EN_SUP_3V3_DAQ /is currently/");
    if(state_of_pin == 1 ){
        fprintf(EXT, "HIGH\n");
    }else if(state_of_pin == 0){
        fprintf(EXT, "LOW\n");
    }else {
        fprintf(EXT, "Invalid\n"); 
    }
    state_of_pin = input_state(EN_SUP_UNREG);
    fprintf(EXT, "    press d: Toggle EN_SUP_UNREG /is currently/");
    if(state_of_pin == 1 ){
        fprintf(EXT, "HIGH\n");
    }else if(state_of_pin == 0){
        fprintf(EXT, "LOW\n");
    }else {
        fprintf(EXT, "Invalid\n"); 
    }
    state_of_pin = input_state(EN_SUP_5V0);
    fprintf(EXT, "    press e: Toggle EN_SUP_5V0 /is currently/");
    if(state_of_pin == 1 ){
        fprintf(EXT, "HIGH\n");
    }else if(state_of_pin == 0){
        fprintf(EXT, "LOW\n");
    }else {
        fprintf(EXT, "Invalid\n"); 
    }
    state_of_pin = input_state(KILL_SWITCH);
    fprintf(EXT, "    press f: Toggle KILL_SWITCH /is currently/");
    if(state_of_pin == 1 ){
        fprintf(EXT, "HIGH\n");
    }else if(state_of_pin == 0){
        fprintf(EXT, "LOW\n");
    }else {
        fprintf(EXT, "Invalid\n"); 
    }
        state_of_pin = input_state(MVCAM_PWR);
    fprintf(EXT, "    press g: Toggle MVCAM_PWR /is currently/");
    if(state_of_pin == 1 ){
        fprintf(EXT, "HIGH\n");
    }else if(state_of_pin == 0){
        fprintf(EXT, "LOW\n");
    }else {
        fprintf(EXT, "Invalid\n"); 
    }
        state_of_pin = input_state(OVCAM_PWR);
    fprintf(EXT, "    press h: Toggle OVCAM_PWR /is currently/");
    if(state_of_pin == 1 ){
        fprintf(EXT, "HIGH\n");
    }else if(state_of_pin == 0){
        fprintf(EXT, "LOW\n");
    }else {
        fprintf(EXT, "Invalid\n"); 
    }
    fprintf(EXT, "    press i: Toggle all Pins");
    

    io_option = fgetc(EXT);

    switch (io_option) {
        case 'a':
            output_toggle(EN_SUP_3V3_1);
            break;
        case 'b':
            output_toggle(EN_SUP_3V3_2);
            break;
        case 'c':
            output_toggle(EN_SUP_3V3_DAQ);
            break;
        case 'd':
            output_toggle(EN_SUP_UNREG);
            break;
        case 'e':
            output_toggle(EN_SUP_5V0);
            break;
        case 'f':
            output_toggle(KILL_SWITCH);
            break;
        case 'g':
            output_toggle(MVCAM_PWR);
            break;
        case 'h':
            output_toggle(OVCAM_PWR);        
            break;
        case 'i' :
            output_toggle(OVCAM_PWR);
            output_toggle(MVCAM_PWR);  
            output_toggle(KILL_SWITCH);
            output_toggle(EN_SUP_5V0);
            output_toggle(EN_SUP_UNREG);
            output_toggle(EN_SUP_3V3_DAQ);
            output_toggle(EN_SUP_3V3_2);
            output_toggle(EN_SUP_3V3_1);
            break;
        case 'x':
            break;
            return;
        default:
            fprintf(EXT, "Invalid IO option. Please try again.\n");
            break;
    }
}




void main_menu(void) {
    char option;
  fprintf(EXT, " __  __ _____ _   _ _   _   _____                 _   _             \n");
  fprintf(EXT, "|  \\/  | ____| \\ | | | | | |  ___|   _ _ __   ___| |_(_) ___  _ __  \n");
  fprintf(EXT, "| |\\/| |  _| |  \\| | | | | | |_ | | | | '_ \\ / __| __| |/ _ \\| '_ \\ \n");
  fprintf(EXT, "| |  | | |___| |\\  | |_| | |  _|| |_| | | | | (__| |_| | (_) | | | |\n");
  fprintf(EXT, "|_| _|_|_____|_| \\_|\\___/  |_|_  \\__,_|_| |_|\\___|\\__|_|\\___/|_| |_|\n");
  fprintf(EXT, "   / \\   ___| |_(_)_   ____ _| |_ ___  __| | |                      \n");
  fprintf(EXT, "  / _ \\ / __| __| \\ \\ / / _` | __/ _ \\/ _` | |                      \n");
  fprintf(EXT, " / ___ \\ (__| |_| |\\ V / (_| | ||  __/ (_| |_|                      \n");
  fprintf(EXT, "/_/   \\_\\___|\\__|_| \\_/ \\__,_|\\__\\___|\\__,_(_)                      \n");
    
    while (1) {
        // Display Main Menu
        fprintf(EXT, "\n-----------------Main Menu-----------------\n");
        fprintf(EXT, "    press a: Get House keeping data\n");
        fprintf(EXT, "    press b: EPS Power output control\n");
        fprintf(EXT, "    press c: House keeping data collection\n");
        fprintf(EXT, "    press d: Check Flash Memories\n");
        fprintf(EXT, "    press e: See satellite Log\n");
        fprintf(EXT, "    press f: Settings of RTC\n");
        fprintf(EXT, "    press g: Satellite log down-link command\n");
        fprintf(EXT, "    press h: IHC Mission start\n");
        fprintf(EXT, "    press i: SEL current Measurement\n");
        fprintf(EXT, "    press j: H8 COM Reset\n");
        fprintf(EXT, "    press k: IO control\n");
        fprintf(EXT, "    press i: UART repeater of EPS\n");
        fprintf(EXT, "    press x: Exit Main Menu\n");
        fprintf(EXT, "    DO NOT USE CAPITAL CHARACTERS TO WRITE!\n\n");

        // Read the user's choice
        option = fgetc(EXT);

        // Main menu switch
        switch (option) {
            case 'a':
                // Call a function to get housekeeping data
                // get_housekeeping_data();
                break;
            case 'b':
                // Call a function to control EPS power output
                // control_eps_power();
                break;
            case 'c':
                // Call a function to collect housekeeping data
                // collect_housekeeping_data();
                break;
            case 'd':
                handle_flash_memories();
                break;
            case 'e':
                // Call a function to see satellite log
                // see_satellite_log();
                break;
            case 'f':
                handle_set_time();
                break;
            case 'g':
                // Call a function for satellite log downlink command
                // satellite_log_downlink_command();
                break;
            case 'h':
                // Call a function for IHC mission start
                // ihc_mission_start();
                break;
            case 'i':
                // Call a function for SEL current measurement
                // sel_current_measurement();
                break;
            case 'j':
                // Call a function for H8 COM reset
                // h8_com_reset();
                break;
            case 'k':
                handle_io_control();
                break;
            case 'l':
                fprintf(EXT, "UART Repeater Initialized.\n");
                uart_repeater();
                break;
            case 'x':
                return;
            default:
                fprintf(EXT, "Invalid option. Please try again.\n");
                break;
        }
    }
}


// Implement the helper functions as required, e.g., READ_DATA_FROM_ADDRESS, WRITE_DATA_NBYTES, etc.



#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

