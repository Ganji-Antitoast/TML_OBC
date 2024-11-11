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

//int8 setup_oscillator(OSC_PRIMARY|OSC_SOSC_ENABLED); //prototype of crystal ignore
char bichig[25] = "test data update of MAIN"; //test data for testing 
char bichigcom[24] = "test data update of COM"; //test data for testing 
char bichigadcs[25] = "test data update of ADCS"; //test data for testing 
char *read_data; //MAIN flash received data will be stored in here 
unsigned char buffer[40]; //secondary buffer 
char *read_data_com; //COM flash received data will be stored in here 
char *read_data_adcs; //ADCS flash received data will be stored in here 

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
      //write and read from MAIN flash memory 
    fprintf(EXT, "Starting to write data in MAIN flash memory\n");
    WRITE_DATA_NBYTES(0x00005000,bichig,29); //write functions 
    delay_ms(1000);
    
    read_data = READ_DATA_NBYTES(0x00005000, 29);
    delay_ms(1000);
    for (int i = 1; i < read_data[i]; i++) {
        fprintf(EXT, "%c", read_data[i]);
        delay_ms(2);
    }
    fprintf(EXT, "\n"); 

       
        //write and read from COM flash memory
    fprintf(EXT, "Starting to write data in COM flash memory\n");
    WRITE_DATA_NBYTES_COM(0x00005000,bichigcom,29);
    delay_ms(1000);
    
    read_data_com = READ_DATA_NBYTES_COM(0x00005000, 29);
    delay_ms(1000);
    for (int i = 1; i < read_data_com[i]; i++) {
        fprintf(EXT, "%c", read_data_com[i]);
        delay_ms(2);
    }
    fprintf(EXT, "\n"); 
    
    
  
    //write and read from ADCS flash memory
    fprintf(EXT, "Starting to write data in ADCS flash memory\n");
    WRITE_DATA_NBYTES_ADCS(0x00005000,bichigadcs,29);
    delay_ms(1000);
    
    read_data_adcs = READ_DATA_NBYTES_ADCS(0x00005000, 29);
    delay_ms(1000);
    for (int i = 1; i < read_data_adcs[i]; i++) {
        fprintf(EXT, "%c", read_data_adcs[i]);
        delay_ms(2);
    }
    fprintf(EXT, "\n"); 
    fprintf(EXT, "TEST IS FINISHED!\n");
    //------------------------MAIN_MENU-------------------------------
    while (TRUE) {
        if (kbhit(EXT)) {
            main_menu();
            fprintf(EXT, "exiting main menu function");
        }
    }
}

