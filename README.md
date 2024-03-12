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

       Further information about hardware setup can be found on the RE_Module_Lab_Hardware.docx which can be found under project directory.
	***************************************************************************/
## Application: ADC_DMA_MultiChannel_UART_ interface with Python - FreeRTOS
		/***************************** Tutorial Tags ******************************
	 ADC triggered by Timer
	 UART data communication to PC
	 Python for control STM32 and reading/ plotting data
	 FreeRTOS implmentation:
	 Tasks (priorities, states),
	 Events,
	 Queues,
	 Semaphores,
	 Memory pools
	 **************************************************************************/

	/* *************************************************************************
	* Application: ADC_DMA_MultiChannel_UART_ interface with Python - FreeRTOS
	*

    We repeat implementation of the previous project "ADC_DMA_MultiChannel_UART_ interface with Python"
    with FreeRTOS implementation.

    Configure three tasks (threads): one task with High priority is triggered with an interrupt
    sent from Python. This task start ADC and Timer for sampling three potentiometers. The second task
    set with Above normal priority convert the ADC data into float, and calculacte the average of 10
    measurements for each of the three ADC channels. The averaged data is sent in a memory queue to Task
    3, which set in normal priority that is resposnsible to send the average data via UART to PC, where
    Python code process and plot the data.

    Python is able to start or stop ADC acquisition via flags sent to MCU. (more information can be found from
    previous project with bare metal implementation)

    Demonstration of this project is shown with figures in the subfolder results of this project

    Python code can be found in the subfolder  "Python client"

    ****************************************************************************
    Calculation of specs used by this application:
    ADC freq  = APB2/4 = 90 MHz/4 = 22.5 MHz
    Max Sample rate for ADC = 22.5 MHz / (15 cycles) = 1.5 MS/s
    Max allowed freq timer for ADC trig = 1.5 M / N_Channels scanned
    Freq_Timer = 100 Hz.
    So for 1 scanned Channel: DMA for 10 measurements is 10 Hz
    for 3 scanned Channels: DMA for 30 measurements is 10 Hz
    because channels are sampled sequentially at ADC sampling rate. :-)
	***************************************************************************/


 



