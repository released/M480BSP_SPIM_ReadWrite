# M480BSP_SPIM_ReadWrite
 M480BSP_SPIM_ReadWrite

update @ 2021/05/24

1. use SPIM in M487 ETM , with PA0 : MOSI ,PA1 : MISO,PA2 : CLK,PA3 : SS

2. under data_creation , enable data dump for SPIM data to write in (2K)

![image](https://github.com/released/M480BSP_SPIM_ReadWrite/blob/main/log_SPIM_dump_data.jpg)

3. emulate data buffer size , by enable define as below 

DATA_BUFF_SIZE_512

DATA_BUFF_SIZE_2K

DATA_BUFF_SIZE_8K

4. below is screen capture , 

data with 512 bytes (erase , write , read count timing)

![image](https://github.com/released/M480BSP_SPIM_ReadWrite/blob/main/log_SPIM_erase_write_read_512.jpg)

data with 2K bytes (erase , write , read count timing)

![image](https://github.com/released/M480BSP_SPIM_ReadWrite/blob/main/log_SPIM_erase_write_read_2K.jpg)

data with 8K bytes (erase , write , read count timing)

![image](https://github.com/released/M480BSP_SPIM_ReadWrite/blob/main/log_SPIM_erase_write_read_8K.jpg)


