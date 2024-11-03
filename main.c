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

#include <main.h>


/*
 * 
 */

//int8 setup_oscillator(OSC_PRIMARY|OSC_SOSC_ENABLED);
char bichig[25] = "test data update of MAIN";
char bichigcom[24] = "test data update of COM";
char bichigadcs[25] = "test data update of ADCS";
unsigned char *read_data[80];
unsigned char buffer[40];
char read_data_com[80];
char read_data_adcs[80];

void main() {
    //------------------------Start_Indicator-------------------------
startup_freeze();
    //--------------------------RTC-----------------------------------
RTC_initialize();
    //------------------------restart_indicator-----------------------
    update_shutdown_count();
    //------------------------read_chip_ID----------------------------
    fprintf(EXT, "Reading chip ID of main\n");
    READ_CHIP_ID_OF();  // Pass the array to be filled by the function
    fprintf(EXT, "Reading chip ID of COM\n");
    READ_CHIP_ID_OF_COM();
    fprintf(EXT, "Reading chip ID of ADCS\n");
    READ_CHIP_ID_OF_ADCS();
    fprintf(EXT, "Done reading chip ID\n");
    delay_ms(1000);
//------------------------write_flash_memory--------------------------
    //  fprintf(EXT, "Starting to write data\n");
    WRITE_DATA_NBYTES(0x00005000,bichig,29);
    delay_ms(1000);
    READ_DATA_NBYTES(0x00005000,read_data, 29);
    delay_ms(1000);
    for (int i = 0; i < buffer[i]; i++) {
        fprintf(EXT, "%c", buffer[i]);
    }
    fprintf(EXT, "\n");
    WRITE_DATA_NBYTES_COM(0x00005000,bichigcom,29);
    delay_ms(1000);
    READ_DATA_NBYTES_COM(0x00005000,read_data_com, 29);
    delay_ms(1000);
    WRITE_DATA_NBYTES_ADCS(0x00005000,bichigadcs,29);
    delay_ms(1000);
    READ_DATA_NBYTES_ADCS(0x00005000,read_data_adcs, 29);
    delay_ms(1000);
    
    //   WRITE_DATA_NBYTES(0x00000250,bichig,40);
//    WRITE_DATA_NBYTES(0x00000300, bichig, 20);
//    fprintf(EXT, "Byte saved\n");
//    fprintf(EXT, "Reading desired address\n");
//    fprintf(EXT, "Reading... \n");
//    char baba;
     READ_DATA_NBYTES(0x00000300, read_data, 20);
    fprintf(EXT, "%c", read_data);
    for (int i = 0; i < 19; i++) {
        fprintf(EXT, "%c", read_data[i]);
    }
    fprintf(EXT, "\n");
    //------------------------MAIN_MENU-------------------------------
    while (TRUE) {
        if (kbhit(EXT)) {
            main_menu();
            fprintf(EXT, "exiting main menu function");
        }
    }
}

