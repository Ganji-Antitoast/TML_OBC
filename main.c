/* 
 * File:   main.c
 * Author: Ganji
 *
 * Created on September 12, 2024, 7:31 AM
 */

#include <spi.h>


/*
 * 
 */


#fuses NOWDT, STVREN, NOXINST, FRC, SOSC, PR, NOCLOCKOUT, HS
#use delay(clock=16MHz, crystal)
#use delay(crystal=16MHz)
//int8 setup_oscillator(OSC_PRIMARY|OSC_SOSC_ENABLED);
char bichig[21] = "test data update with";
char read_data[80];

void main() {
    fprintf(EPS, "start_writing\n");
    fprintf(EXT, "POWER ON!\n");
    //--------------------------RTC-------------------------
    setup_lcd(LCD_DISABLED);
    rtc_time_t write_clock, read_clock;
    setup_rtc(RTC_ENABLE | RTC_CLOCK_SOSC | RTC_CLOCK_INT, 0);
    RTC_CLOCK_INT;
    rtc_read(&read_clock);

    //------------------------Start_Indicator-----------------------
    delay_ms(1000);
    fprintf(EXT, "POWER ON!\n");
    //------------------------restart_indicator-----------------------
//     #define SHUTDOWN_COUNT_ADDRESS  0x00000500
      update_shutdown_count();
//       #define SHUTDOWN_COUNT_ADDRESS  0x00000500  // Address where shutdown count is stored
//         fprintf(EXT, "Shutdown count started\n");
//        unsigned int8 shutdown_count[1];  
//        READ_DATA_NBYTES(SHUTDOWN_COUNT_ADDRESS, shutdown_count, 1);
//        delay_ms(10);
//    
//        // Check if the shutdown count is uninitialized
//        if (shutdown_count[0] == 0xFF) {
//            shutdown_count[0] = 0;  // Initialize to 0 if uninitialized
//             fprintf(EXT, "Shutdown count uninitialized, setting to 0\n");
//        }
//    
//        shutdown_count[0] += 1;  // Increment the shutdown count
//        WRITE_DATA_NBYTES(SHUTDOWN_COUNT_ADDRESS, shutdown_count, 1);
//        delay_ms(10);
    
        // Print the updated shutdown count
//         fprintf(EXT, "Shutdown count: %u\n", shutdown_count[0]);
        fprintf(EXT, "Reading chip ID\n");
    unsigned char chip_id[8];
    READ_CHIP_ID_OF();
    for (int i = 0; i < 8; i++) {
        fprintf(EXT, "%c", chip_id[i]);
    }
    fprintf(EXT, "\n");
    delay_ms(1000);
    fprintf(EXT, "Done reading chip ID\n");
    //  fprintf(EXT, "Starting to write data\n");
    //WRITE_DATA_NBYTES(0x00000160,bichig,29);
    //   WRITE_DATA_NBYTES(0x00000250,bichig,40);
    WRITE_DATA_NBYTES(0x00000300, bichig, 20);
    fprintf(EXT, "Byte saved\n");
    fprintf(EXT, "Reading desired address\n");
    fprintf(EXT, "Reading... \n");
    READ_DATA_NBYTES(0x00000300, read_data, 20);
    for (int i = 0; i < 16; i++) {
        fprintf(EXT, "%c", read_data[i]);
    }
    fprintf(EXT, "\n");
    while (1) {
        if (kbhit(EXT)) {
            main_menu();
            fprintf(EXT, "exiting main menu function");
        }
    }
}

