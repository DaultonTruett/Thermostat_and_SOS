# SNHU CS-350: Emerging Systems Architectures and Technologies

## *Thermostat*
  ### Summarize the project and what problem it was solving.
      This project interfaces with a microcontroller to simulate an embedded system in a smart thermostat. It features a task scheduler to check button interrupts every 200ms, read the room temperature every 500ms, turn the heat on or off and send data of the current state via UART2 every 1000ms. A timer is used to execute the task scheduler at a particular time interval (the GCF of all the tasks). Two buttons on the microcontroller are configured as interrupts via GPIO to increase/decrease the target temperature. I2C is used to read the room temperature via an integrated circuit on the board. GPIO is used to turn a red LED on/off indicating if the heat is on or off which is based on the current temperature and target temperature. UART2 is used to simulate sending the current temperature, target temperature, state of the heater, and the time the thermostat has been running. 
  
  ### What did you do particularly well?
      What I did well on this project was meeting the software requirements by writing clean code to operate the embedded system as a thermostat. The architecture I implemented limited memory use which is important in an embedded system. I also effectively implemented a task scheduler to track the programs state and execute tasks at their arranged times.
      
  ### Where could you improve?
      I could improve by practicing embedded programming much more frequently as it has noticeable differences from web and application programming. This class was my first exposure to C, and even though I have experience with C++, it took some time to get used to some of the nuanced syntax and rules of C. Another area I could improve is becoming more familiar with the various libraries used in embedded programming because they have much more functionality beyond what I used them for in this project.
  
  ### What tools and/or resources are you adding to your support network?
      A tool I will be adding to my support network is an Arduino microcontroller. This is considered one of the best tools for embedded systems programming as it has a vast community and resources available. The resources I will add to my support network is the official documentation for each of the peripherals embedded system use to communicate with external devices (UART, PWM, GPIO, I2C, etc.).
     
  ### What skills from this project will be particularly transferable to other projects and/or course work?
      The skills I gained from this project that will be transferrable is the use of a task scheduler and tracking state of application variables. 

  
  ### How did you make this project maintainable, readable, and adaptable?
      I made this project maintainable, readable, and adaptable by following best practices, in-line commenting, implementing 'separation of concerns' with job specific functions, and organizing my code thoroughly.
  

## *SOS*
  ### Summarize the project and what problem it was solving.
  ### What did you do particularly well?
  ### Where could you improve?
  ### What tools and/or resources are you adding to your support network?
  ### What skills from this project will be particularly transferable to other projects and/or course work?
  ### How did you make this project maintainable, readable, and adaptable?

