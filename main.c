/* 
 * File:   main.c
 * Author: Ganji
 *  ____    _    _   _     _ ___ 
   / ___|  / \  | \ | |   | |_ _|
  | |  _  / _ \ |  \| |_  | || | 
  | |_| |/ ___ \| |\  | |_| || | 
   \____/_/   \_\_| \_|\___/|___| 
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
    //--------------------------RTC-------------------------
RTC_initialize();
    //------------------------Start_Indicator-----------------------
startup_freeze();
    //------------------------restart_indicator-----------------------
    update_shutdown_count();
    
    fprintf(EXT, "Reading chip ID\n");
int8 chip_id[8];
READ_CHIP_ID_OF(chip_id);  // Pass the array to be filled by the function

// Print the received chip ID bytes
for (int i = 0; i < 8; i++) {
    fprintf(EXT, "%02X ", chip_id[i]);
}
fprintf(EXT, "\n");

    delay_ms(1000);
    fprintf(EXT, "Done reading chip ID\n");
    //  fprintf(EXT, "Starting to write data\n");
    //WRITE_DATA_NBYTES(0x00000160,bichig,29);
    //   WRITE_DATA_NBYTES(0x00000250,bichig,40);
//    WRITE_DATA_NBYTES(0x00000300, bichig, 20);
//    fprintf(EXT, "Byte saved\n");
//    fprintf(EXT, "Reading desired address\n");
//    fprintf(EXT, "Reading... \n");
    READ_DATA_NBYTES(0x00000300, read_data, 20);
//    for (int i = 0; i < 16; i++) {
//        fprintf(EXT, "%x", read_data[i]);
//    }
    fprintf(EXT, "\n");
    while (TRUE) {
        if (kbhit(EXT)) {
            main_menu();
            fprintf(EXT, "exiting main menu function");
        }
    }
}

