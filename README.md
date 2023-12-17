# README.md
## My I2C Wrappers  


> Add the parth to the toolbox component in your main CMakeList.txt  
set(EXTRA_COMPONENT_DIRS "/Users/devuser/ESP_Workspaces/my-common-components/toolbox")  

And include:  
#include <toolbox.h>  
to your sources.  
  
```c
#define REG_ADDR      0x01  
uint8_t i2c_address = 0x58;  
uint8_t wr_buffer[1] = { REG_ADDR };  
uint8_t bytes = 1;  
esp_err_t err = i2c_write(i2c_address, wr_buffer, bytes);   
``` 


```c
As a response read 4 bytes and store them in recv_buffer.  
uint8_t recv_buffer[4] = { 0 };  
esp_err_t err +=  i2c_read(i2c_address, recv_buffer, 4);  
```