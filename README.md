# Embedded_Systems_Final_Project

_This project was done in December 2024_

### Project Summary: Scoreboard Embedded System  

In this project, we developed an advanced scoreboard embedded system using the STM32F091RC microcontroller, leveraging C programming, FreeRTOS, and assembly code. The system integrates multiple functionalities and components to deliver a versatile and interactive scoreboard.  

#### Key Features:  

1. **LED Sequences and Indicators**:  
   - **Default Sequence**: The board displays an initial LED blinking sequence (e.g., R-R-G-G). This default sequence is triggered whenever a correct button is pressed. 
   - **Invalid Button Press**: A red LED blinks (R-R) to indicate an invalid input. The watchdog timer ensures system stability during these scenarios.  

2. **Interrupt Implementation**:  
   - Onboard LED blinking was implemented using a timer and interrupts. The LED blinking code was developed in assembly language.  

3. **Score Input and Display**:  
   - Scores are input via a keypad.  
   - Updated scores are displayed on connected LCDs for real-time feedback.
   - The correct buttons were '1' for increasing the score, '3' for decreasing the score, '5' to reset the score, and '0' for a self rest mode.

4. **Bluetooth Integration**:  
   - A Bluetooth module allows users to wirelessly update the scoreboard using their phones, providing convenience and modern interaction.  

5. **Obstacle Detection**:  
   - An ultrasonic sensor detects when a hand or obstacle is in front of it, triggering a buzzer to alert that someone has entered the stadium.  

6. **Light Control with ADC**:  
   - The system reads potentiometer values using ADC (Analog-to-Digital Conversion). When the value exceeds a certain threshold, an LED turns on to provide light control functionality.  

7. **Self-Test Mode**:  
   - A "Self-Test Mode" is activated by pressing button 0. This mode systematically calls and tests all functions in the code, demonstrating the full range of features.  

8. **Real-Time Task Scheduling with FreeRTOS**:  
   - FreeRTOS enabled efficient scheduling, allowing all processes to run concurrently without conflict, ensuring smooth and reliable system performance.  

#### Technical Achievements:  

- **Assembly Code**: Timer-based LED blinking was implemented directly in assembly, showcasing low-level programming expertise.  
- **Interrupts and Timers**: Precise timing and responsiveness were achieved through the use of interrupts and timers.  
- **Watchdog Timer**: Enhanced system reliability by resetting the system during unresponsive states.  
- **Bluetooth and Ultrasonic Integration**: Added modern connectivity and functionality to improve user interaction and safety.  

This project demonstrates a comprehensive understanding of embedded systems design and implementation, blending real-time operating systems, low-level programming, and hardware-software integration for a robust and feature-rich application.
