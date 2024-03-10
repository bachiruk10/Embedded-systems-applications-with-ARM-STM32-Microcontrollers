#Embedded-systems-applications-with-ARM-STM32-Microcontrollers


# Application: ADC_DMA_MultiChannel_UART_ interface with Python

	/***************************** Application tags ******************************
	 ADC triggered by timer
	 UART data communication to PC
	 Python for control STM32 and reading/ plotting data
	 **************************************************************************/

	/* *************************************************************************
	* Application: ADC_DMA_MultiChannel_UART_ interface with Python
	*

    Form a project with the following specification:

    Configure STM32F4 to read data from 3 potentiometers. ADC data is triggered using
    appropriate timer at a frquency of 10 Hz (10 samples/second). Data is transfered in real time to Python
    for plotting. Also configure Python to send character "g" via UART to
    for the ADC to start acquiring data, or "r" to stop the ADC. Use animated plot on Python to plot
    updated data frames of 50 digitised ADC samples.
    Green led will be toggled every time a sample is taken by the ADC
    Red led will be turned on when ADC is started by Python and turned off when ADC stops
	***************************************************************************/
 Results: You can access results of this application from subfolder Results under project folder   

# Application: ADC_DMA_MultiChannel_UART_ interface with Python

 	/***************************** Application tags ******************************
	 ADC triggered by timer
	 UART data communication to PC
	 Python for control STM32 and reading/ plotting data
	 **************************************************************************/

	/* *************************************************************************
	* Application: ADC_DMA_MultiChannel_UART_ interface with Python
	*

    Form a project with the following specification:

    Configure STM32F4 to read data from 3 potentiometers. ADC data is triggered using
    appropriate timer at a frquency of 10 Hz (10 samples/second). Data is transfered in real time to Python
    for plotting. Also configure Python to send character "g" via UART to
    for the ADC to start acquiring data, or "r" to stop the ADC. Use animated plot on Python to plot
    updated data frames of 50 digitised ADC samples.
    Green led will be toggled every time a sample is taken by the ADC
    Red led will be turned on when ADC is started by Python and turned off when ADC stops
	***************************************************************************/
 Results: You can access results of this application from subfolder Results under project folder

 #  Application: SPI_Interface_LCD MIDAS DISPLAYS-MC21605H6W-SPTLY-V2

  	/***************************** Application tags ******************************
	 LCD MIDAS DISPLAYS-MC21605H6W-SPTLY-V2 driver
	 SPI peripheral
         Shift register
	 **************************************************************************/

	/* *************************************************************************
	* Application: SPI_Interface_LCD MIDAS DISPLAYS-MC21605H6W-SPTLY-V2
	*

    In this project we are going to display a string on an LCD screen using a shift register and the SPI module of STM32F401RE Nucleo board

    4.1 LCD display and Shift Register 
	The LCD (MIDAS DISPLAYS-MC21605H6W-SPTLY-V2) has a 2 line x 16 character display. It can be configured, and the data can be written via SPI interface. To be able to use SPI with the LCD, you will have to use a 74HC595N shift register. 
	You can find out more about shift registers, consult the datasheet for 74HC595N. 
	You will be using the LCD in 4-bit mode. You will need to write a driver for the LCD onj STM32 including initialisation of the LCD 
        by following the instructions on the datasheet LCD.
	***************************************************************************/
 



