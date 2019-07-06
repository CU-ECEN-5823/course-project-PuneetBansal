# INFRASTRUCTURE OPERATIONAL EFFICIENCY IMPROVEMENT USING BLUETOOTH MESH
                         Contributors  : Puneet Bansal , Nachiket Kelkar, Tanmay Chaturvedi 
                         Professor     : Daniel Walkes
## OVERVIEW:
The project aims at improving efficiency of an infrastructure by monitoring and improving 3 major factors:
- Human Comfort (Implemented on low power node 1) : Achieved by monitoring humidity and air quality Index values.
- Effective Space Utilization (Implemented on friend node) : Achieved by monitoring the noise level and the number of people in an area.
- Energy efficiency ( Implemented on low power node 2): Achieved by using artificial lights based on the amount of luminous intensity.

**This repository contains code for Low Power Node 1 described above , implemented by Puneet Bansal**

## FEATURES:
Features: 
- Configuration of the nodes as LPN and successful friendship establishment
- Integration of CCS811 gas sensor and on board Si7021 humidity sensor.
- Ability to send valid sensor data using level model to the friend node.
- Ability to send type of sensor data using on_off model to the friend node.
- Use of persistent data.
- Use of state machines for the sensor interfacing.

## BLOCK DIAGRAM
![Block Diagram](https://github.com/CU-ECEN-5823/course-project-PuneetBansal/blob/master/Images/Block%20Diagram.jpg)

## HARDWARE COMPONENTS
- Silicon Labs EFR32BG13
- Low Power Node 1 
  - Air Quality Index Sensor
  - Humidity Sensor
- Low Power Node 2
  - Ambient Light Sensor
  - Fire Sensor ,MQ2
- Friend Node
  - Motion Sensor 
  - Sound Sensor

## DOCUMENTATION :
- **Puneet's Project Report** can be found [here](https://docs.google.com/document/d/1DEPz8JNS0c0bjY_M5uN1TsHUUEU2Ww4W-2euO1UKoJ0/edit)
- **Puneet's Command Table** can be found [here](https://docs.google.com/spreadsheets/d/1ZElzzUw0Mz11OLMvnqzG13fEOBEhh-M-a5BcmVSSPeY/edit#gid=108495522)
- **Group Report** can be found [here](https://docs.google.com/document/d/1RYUYPIxFx2UlZDfMEpF_KqUhhn8t8Qh3gLkbjv7GPYw/edit)
- **Project Verification Plan** can be found [here](https://docs.google.com/spreadsheets/d/1lZt5A3WWoxcO_D8hX-FW8FySXG71lKcEpaOZikxLJfs/edit#gid=732473264)
- **Nachiket Kelkar's contribution** can be found [here](https://github.com/CU-ECEN-5823/course-project-NachiketKelkar)
- **Tanmay Chaturvedi's contribution** can be found [here]( https://github.com/CU-ECEN-5823/course-project-TanmayChaturvedi1 
)



### ECEN 5823 Bluetooth Mesh Skeleton Project

This project contains skeleton code used for coursework in University of Colorado [ECEN 5823 IoT Embedded Firmware](https://sites.google.com/colorado.edu/ecen5823/home).

Below is an overview of the sequence used to generate this repository:
* The project was generated starting with the new project Wizard built into [Simplicity Studio 4](https://www.silabs.com/products/development-tools/software/simplicity-studio).  
* The AppBuilder project was used with application type "Bluetooth Mesh SDK" with version 2.11.2.0 and Bluetooth SDK 2.9.3.0, application stack "Bluetooth Mesh SDK 1.4.1.0"
* The SOC- BT Mesh Empty project application was used.
* Board and part were configured for BRD4104A Rev 00 and EFR32BG13P632F512GM48 respectively
* Configurations were setup to target GNU ARM 7.2.1.
* Simplicity project Workspace paths were setup to pull in emlib functions needed for display support, including middleware/glib directory and glib/glib glib/dmd directories
* Relevant emlib project files were copied from SiliconLabs\SimplicityStudio\v4\developer\sdks\gecko_sdk_suite\v2.5\platform as needed and added into the respective directories at the root.
* The main.c file in the root folder was renamed [gecko_main.c](gecko_main.c).  Contents of the main while loop were moved into functions and the main() function was #ifdef'd out.
* The [src](src) subfolder was added to contain code specific to the ECEN 5823 course and source files were added to support ECEN 5823 and the simplicity studio exercise assignment.




