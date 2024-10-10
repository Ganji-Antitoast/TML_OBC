/* 
 * File:   SPI.h
 * Author: Ganji
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

#define SPIPORT MAIN_FM
#use delay(CLOCK=16M, CRYSTAL=16M)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#use rs232(baud=9600, parity=N, xmit=PIN_E5, rcv=PIN_E4, bits=8, stream=EXT, FORCE_SW) //MAIN External debuging port
#use rs232(baud=9600, parity=N, xmit=PIN_C6, rcv=PIN_C7, bits=8, stream=REAR, FORCE_SW) //MAIN DAQ Rear access board 
#use rs232(baud=9600, parity=N, xmit=PIN_D2, rcv=PIN_D3, bits=8, stream=COM, FORCE_SW) //MAIN COM Communication, send CW data 
#use rs232(baud=9600, parity=N, xmit=PIN_F7, rcv=PIN_F6, bits=8, stream=CAM, FORCE_SW) //MAIN CAM Communication
#use spi(MASTER, CLK=PIN_E1, DI=PIN_E0, DO=PIN_E6,  BAUD=10000, BITS=8, STREAM=MAIN_FM, MODE=0) //MAIN flash memory port
#use spi(MASTER, CLK=PIN_B2, DI=PIN_B5, DO=PIN_B4,  BAUD=10000, BITS=8, STREAM=COM_FM, MODE=0) //COM shared flash memory port
#use spi(MASTER, CLK=PIN_A3, DI=PIN_A0, DO=PIN_A1,  BAUD=10000, BITS=8, STREAM=ADCS_FM, MODE=0) //ADCS shared flash memory port, CAM (ovcam,mvcam)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
#define CS_PIN_1 PIN_E2 //OBC_MUX_SELECT
#define CS_PIN_2 PIN_B3 //COM_MUX_SELECT
#define CS_PIN_3 PIN_A2 //ADCS_MUX_SELECT
#define CS_PIN_4 PIN_G2 //OVCAM_MUX_SELECT
#define CS_PIN_5 PIN_G3 //MVCAM_MUX_SELECT



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
    printf("WRITE ADDRESS: 0x%08lx\n", ADDRESS);  // Print address as hex
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
        printf("%02X ", data[i]);    // Print each byte as hex (optional)
    }
    
    output_high(CS_PIN_1);  // Deselect SPI device
    
    printf("\n%d BYTES WRITTEN!\n", data_number);

}

void WRITE_DATA_NBYTES_COM(unsigned int32 ADDRESS, unsigned int8 data[], unsigned char data_number) {
    printf("WRITE ADDRESS: 0x%08lx\n", ADDRESS);  // Print address as hex
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
        printf("%02X ", data[i]);    // Print each byte as hex (optional)
    }
    
    output_high(CS_PIN_2);  // Deselect SPI device
    
    printf("\n%d BYTES WRITTEN!\n", data_number);

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
        printf("%02X ", Data_return[i]);           // Print each byte as hex
    }

    output_high(CS_PIN_1);  // Deselect SPI device after reading
    printf("\n");
}


int8 READ_CHIP_ID_OF()
{
 output_low(CS_PIN_1);           //lower the CS PIN
 
 ////////////////////////////////////////////////////////////////
 int8 chip_id;
 spi_xfer(SPIPORT,READ_ID);    //READ ID COMAND   (0x9F)
 chip_id = spi_xfer(SPIPORT);
 ////////////////////////////////////////////////////////////////
 
 output_high(CS_PIN_1);         //take CS PIN higher back
 
 return chip_id;
}


//#define SHUTDOWN_COUNT_ADDRESS  0x00000500  // Address where shutdown count is stored
//
//
//int8 update_shutdown_count(void) {
//    printf("Shutdown count started\n");
//
//    unsigned int8 shutdown_count[1];  
//    READ_DATA_NBYTES(SHUTDOWN_COUNT_ADDRESS, shutdown_count, 1);
//    delay_ms(10);
//
//    // Check if the shutdown count is uninitialized
//    if (shutdown_count[0] == 0xFF) {
//        shutdown_count[0] = 0;  // Initialize to 0 if uninitialized
//        printf("Shutdown count uninitialized, setting to 0\n");
//    }
//
//    shutdown_count[0] += 1;  // Increment the shutdown count
//    WRITE_DATA_NBYTES(SHUTDOWN_COUNT_ADDRESS, shutdown_count, 1);
//    delay_ms(10);
//
//    // Print the updated shutdown count
//    printf("Shutdown count: %u\n", shutdown_count[0]);
//    return shutdown_count[0];
//}


void set_clock(rtc_time_t &date_time)
{

   date_time.tm_year=00;
   date_time.tm_mon=00;
   date_time.tm_mday=00;
   date_time.tm_wday=00;
   date_time.tm_hour=00;
   date_time.tm_min=00;
   date_time.tm_sec=0; 

}
//hak thuah spilt on that thang enough 
// Main menu function
void main_menu(void) {
    char option;
    char flash_option;           // Variable to capture flash memory option
    char main_flash_option;      // Variable to capture MAIN flash memory option
    char com_flash_option;       // Variable to capture com flash memory option
    char adcs_flash_option;      // Variable to capture adcs flash memory option
     //Variable for option d
    unsigned int32 address;
    unsigned char data[32]; // Maximum data length
    unsigned char data_length;

    printf("/n");
    printf("-----------------Main_menu-----------------\n");
    printf("    press a: Get House keeping data\n");
    printf("    press b: EPS Power output control\n");
    printf("    press c: Houskeeping data collection\n");
    printf("    press d: Check Flash Memories\n");
    printf("    press e: See satellite Log\n");
    printf("    press f: Set time of RTC\n");
    printf("    press g: Satellite log downlink command\n");
    printf("    press h: IHC Mission start\n");
    printf("    press i: SEL current Measurement\n\n");
    printf("    press j: H8 COM Reset\n");
    printf("    press k: IO control\n");
    printf("    press x: Exit Main_menu\n");
    printf("    DO NOT USE CAPITAL CHARACTERS TO WRITE!\n\n\n");

    while (1) {
        
        option = getc();

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
            //g
                printf("    pressed option d: Check Flash Memories\n\n");
                printf("    Please choose which flash memory to work on (a, b, c, ...):\n\n");
                printf("    press a: Choice MAIN flash memory\n");
                printf("    press b: Choice COM shared flash memory\n");
                printf("    press c: Choice ADCS shared flash memory\n");
                flash_option = getc();

                switch (flash_option) {
                    case 'a':
                        printf("MAIN flash memory chosen\n");
                        printf("    press a: Read id of the chip\n");
                        printf("    press b: Write data set in specified address\n");
                        printf("    press c: Read data set in specified address\n");
                        printf("    press d: Return to MAIN MENU\n");
                        main_flash_option = getc();

                        switch (main_flash_option) {
                            case 'a':
                                printf("    pressed option a: Read id of the chip\n");
                                printf("Started reading chip ID of MAIN flash memory\n");
                                READ_CHIP_ID_OF();  // Implement this function or replace with actual code
                                break;
                            case 'b':
                                printf("    pressed option b: \n");
                                printf("    Write data set in specified address\n");
                                printf("Enter address (32-bit hexadecimal): ");
                                scanf("%lx", &address);
                                printf("Enter data length (1 to 32): ");
                                scanf("%d", &data_length);
                                printf("Enter data bytes (space-separated hexadecimal values, e.g., 'AA BB CC'): ");
                                for (unsigned char i = 0; i < data_length; i++) {
                                    scanf("%x", &data[i]);
                                }
                                WRITE_DATA_NBYTES(address, data, data_length);
                                printf("Data written to specified address.\n");
                                break;
                            case 'c':
                                printf("    pressed option c: \n");
                                printf("    Read data set in specified address\n");
                                printf("    Enter your specified address: ");
                                scanf("%lx", &address);
                                
                                break;
                            case 'd':
                                //main_menu();
                                break;
                            default:
                                printf("Invalid MAIN flash memory option. Please try again.\n");
                                break;
                        }
                        break;
                    case 'b':
                        // Implement COM shared flash memory handling
                        break;
                    case 'c':
                        // Implement ADCS shared flash memory handling
                        break;
                    case 'x':
                        return;
                    default:
                        printf("Invalid flash memory option. Please try again.\n");
                        break;
                }
                break;
            case 'e':
                // Call a function to see satellite log
                // see_satellite_log();
                break;
            case 'f':
                // Call a function to reset all time to 0
                rtc_time_t write_clock, read_clock;
                rtc_read(&read_clock);        //reads clock value from RTCC
                printf("Now time is\n");
                printf("\r%02u/%02u/20%02u %02u:%02u:%02u",read_clock.tm_mon,read_clock.tm_mday,read_clock.tm_year,read_clock.tm_hour,read_clock.tm_min,read_clock.tm_sec);
                printf("Time changing function activated\n");
                set_clock(write_clock);
                rtc_write(&write_clock); 
                printf("Time successfully changed current time is\n");
                rtc_read(&read_clock);        //reads clock value from RTCC
                printf("\r%02u/%02u/20%02u %02u:%02u:%02u",read_clock.tm_mon,read_clock.tm_mday,read_clock.tm_year,read_clock.tm_hour,read_clock.tm_min,read_clock.tm_sec);
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
            case 'x':
                return;
case 'k':
    printf("IO control chosen\n");
    printf("    press a: Toggle CPLD_P14 (currently ");
    if (input_state(PIN_A0) == TRUE) {
        printf("HIGH");
    } else if (input_state(PIN_A0) == FALSE){ 
        printf("LOW");
    }else{
        printf("error handeling this request");
    } 
    printf(")\n");
    printf("    press b: Toggle CPLD_P13\n");

            printf("    press c: Toggle CPLD_P12\n");
            printf("    press d: Toggle CPLD_P15\n");
            printf("    press e: Toggle KILL_SW\n");
            printf("    press f: Toggle DIO_OBC0\n");
            printf("    press g: Toggle COM_MISO\n");
            printf("    press h: Toggle COM_MOSI\n");
            printf("    press i: Toggle EN_3V3_DAQ\n");
            printf("    press j: Toggle EN_SUP_5V0\n");
            printf("    press k: Toggle DIO_OBC1\n");
            printf("    press l: Toggle DIO_OBC3\n");
            printf("    press m: Toggle CPLD_P18\n");
            printf("    press n: Toggle CPLD_P17\n");
            printf("    press o: Toggle CPLD_P16\n");
            printf("    press p: Toggle CPLD_P10\n");
            printf("    press q: Toggle CPID_P11\n");
            printf("    press r: Toggle CPLD_P09\n");

            char io_option = getc();
            
            switch (io_option) {
                
        case 'a':
//            unsigned int pinstate;
//            pinstate = input_state(CPLD_P14);
//            if (pinstate == 1) {
//                printf("output of the CPLD_P14 is HIGH\n");
//            } else {
//                printf("output of the CPLD_P14 is LOW\n");
//            }
//            output_toggle(CPLD_P14);

            break;
                case 'b':
                    // Toggle CPLD_P13
                    break;
                case 'c':
                    // Toggle CPLD_P12
                    break;
                case 'd':
                    // Toggle CPLD_P15
                    break;
                case 'e':
                    // Toggle KILL_SW
                    break;
                case 'f':
                    // Toggle DIO_OBC0
                    break;
                case 'g':
                    // Toggle COM_MISO
                    break;
                case 'h':
                    // Toggle COM_MOSI
                    break;
                case 'i':
                    // Toggle EN_3V3_DAQ
                    break;
                case 'j':
                    // Toggle EN_SUP_5V0
                    break;
                case 'k':
                    // Toggle DIO_OBC1
                    break;
                case 'l':
                    // Toggle DIO_OBC3
                    break;
                case 'm':
                    // Toggle CPLD_P18
                    break;
                case 'n':
                    // Toggle CPLD_P17
                    break;
                case 'o':
                    // Toggle CPLD_P16
                    break;
                case 'p':
                    // Toggle CPLD_P10
                    break;
                case 'q':
                    // Toggle CPID_P11
                    break;
                case 'r':
                    // Toggle CPLD_P09
                    break;
                case 's':
                    // restart 
                   // main_menu_exit();
                case 'x':
                return;
                    
                default:
                    printf("Invalid IO option. Please try again.\n");
                    break;
            }
            break;

        default:
            printf("Invalid option. Please try again.\n");
            break;
    }
    }
}


#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

