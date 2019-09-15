# RTK-comm-parsing
 this code is derived from 2019 International Student Car Competition : Auto-Driving Team D-Ace, DGIST(Deagu Gyeongbuk institude of Science & Technology) based on c++

# Introduction of CPP files & Functions
## CPP files
#### 1. CSerialPort.cpp, CSerialPort.h
 * This cpp, header files are for serial port communication and are required for the RTK_Comm function, which will be explatined later.
 * User do not need to modify this code.

 #### 2. RTK.cpp 
  * This is containing the main functions(RTK_Comm, ValueTrans etc).
  * User can modify this file as needed. 
  
## Functions
 #### 1. vector <double>UTM(double lat, double lng)
  * Function that takes longitude and latitude as input, converts it into x,y of UTM coordinate system and stores it in vector
  * This is used in the ValueTrans to convert raw data of RTK into UTM coordinate system.
  * User do not need to modify this code
 
 #### 2. void HeroNeverDies()
  * Protection code for when communication with RTK isn't connected
  * User do not need to modify this code
 
 #### 3. void RTK_Comm()
  * Function that saves the raw data of RTK obtained through serial communication to a txt file, using a buffer.
  * Befroe using this function, you need to make sure that **#define RTKComm** is defeined to a specific value.  ex) #define RTKComm L"COM3"
  * If communication is successed in real time and values are saved, **"done"** is printed.
  * User can modify buffer's size in this code as needed.  ex) BYTE * pByte = new BYTE[2028]; => BYTE * pByte = new BYTE[512];

#### 4. void ValueTrans(int lat_deg, int lng_deg)
 * Function to take deg vealues of longitude and latitude as input, convert them to x,y of UTM coordinate system and save then in a txt file using **vector<double>UTM(double lat, lng) function**
 * Before using this function, set the path of the txtfie where raw data is stroed **"ifstream gpsfile(".txt");"** also the input of this fuction should be set to degree of the longitude and latitude of the current position
 * If value conversion is completed, **"Raw data to UTM complete"** is printed.
 * User can modify this code as needed. 
 
 # Points to note
 1. The overall code used NMEA protocol GNRMC.
 2. When converting to the UTM corrdinate system, the writer used the longitude and latitude form **ddmm.mmmm**, so the user needs to check whether raw data is saved in the abouve form **ddmm.mmmm**, when saving the raw data 
 3. When executing a function in **int main()** only one of **RTK_Comm** and **ValueTrans** can be executed and not simulataneously.
 4. User can use the codes above to implement a function that allows communication and value transformation at the same time.
