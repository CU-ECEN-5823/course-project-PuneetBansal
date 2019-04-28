# ECEN 5823 Bluetooth Mesh Skeleton Project

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




## INFRASTRUCTURE OPERATIONAL EFFICIENCY IMPROVEMENT USING BLUETOOTH MESH

The following has been implemented in the project as of 28th April 2019:

* Configuration of the nodes as LPN and successfull friendship establishment

* Integration of CCS811 gas sensor and on board Si7021 humidity sensor.

* Ability to send valid sensor data using level model to the friend node.

* Ability to send type of sensor data using on_off model to the friend node.

* Addition of persistent data

* Addition of state machines for the sensors

* All the proposed elements are functional.

* FinalReport Puneet : https://drive.google.com/drive/u/1/folders/1kCcFiodJQJ5zUSyfy8F3fvhO_kHt09sV

* Final Group Update : https://drive.google.com/drive/u/1/folders/19pP2BnPFkOunKWceiNoMzz06ycwfbmVY
