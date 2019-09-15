#include "CSerialPort.h"
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <iomanip>

// RTK Port Number 
// ex) L"COM3"
#define RTKComm L""


//for UTM transformation
#define PI 3.14159265358979323846
#define radiRatio 1 / 298.257223563
#define lamda 129.0*PI/180.0
#define Eoff 500000
#define k0 0.9996
#define radi 6378137

CSerialPort _rtk;

vector <double>UTM(double lat, double lng) {
	double lat_rad = lat * PI / 180.0;
	double lng_rad = lng * PI / 180.0;

	double n = radiRatio / (2 - radiRatio);
	double A = radi * (1 - n + 5.0 / 4.0 * (pow(n, 2) - pow(n, 3)) + 81.0 / 64.0 * (pow(n, 4) - pow(n, 5)));
	double B = 3.0 / 2.0 * radi*(n - pow(n, 2) + 7.0 / 8.0 * (pow(n, 3) - pow(n, 4)) + 55.0 / 64.0* pow(n, 5));
	double C = 15.0 / 16.0 * radi*(pow(n, 2) - pow(n, 3) + 3.0 / 4.0 * (pow(n, 4) - pow(n, 5)));
	double D = 35.0 / 48.0 *radi*(pow(n, 3) - pow(n, 4) + 11.0 / 16.0 * pow(n, 5));
	double Ed = 315.0 / 512.0 * radi*(pow(n, 4) - pow(n, 5));
	double S = A * lat_rad - B * sin(2 * lat_rad) + C * sin(4 * lat_rad) - D * sin(6 * lat_rad) + Ed * sin(8 * lat_rad);
	double esq = radiRatio * (2 - radiRatio);
	double v = radi / sqrt(1 - esq * pow(sin(lat_rad), 2));
	double esqd = radiRatio * (2 - radiRatio) / pow((1 - radiRatio), 2);

	double T1 = k0 * S;
	double T2 = v * sin(lat_rad)*cos(lat_rad)*k0 / 2.0;
	double T3 = v * sin(lat_rad)*pow(cos(lat_rad), 3)*k0 / 24.0 * (5.0 - pow(tan(lat_rad), 2) + 9.0 * esqd*pow(cos(lat_rad), 2) + 4.0 * pow(esqd, 2)*pow(cos(lat_rad), 4));
	double T4 = v * sin(lat_rad)*pow(cos(lat_rad), 5)*k0 / 720.0 * (61.0 - 58.0 * pow(tan(lat_rad), 2) + pow(tan(lat_rad), 4) + 270.0 * esqd*pow(cos(lat_rad), 2) - 330.0 * esqd*pow(tan(lat_rad)*cos(lat_rad), 2)
		+ 445.0 * pow(esqd, 2)*pow(cos(lat_rad), 4) + 324.0 * pow(esqd, 3)*pow(cos(lat_rad), 6) - 680.0 *pow(tan(lat_rad), 2)* pow(esqd, 2)*pow(cos(lat_rad), 4) + 88.0 * pow(esqd, 4)*pow(cos(lat_rad), 8) - 600.0 * pow(esqd, 3)*pow(tan(lat_rad), 2)*pow(cos(lat_rad), 6)
		- 192.0 * pow(esqd, 4)*pow(tan(lat_rad), 2)*pow(cos(lat_rad), 8));
	double T5 = v * sin(lat_rad)*pow(cos(lat_rad), 7)*k0 / 40320.0 * (1385.0 - 3111.0 * pow(tan(lat_rad), 2) + 543.0 * pow(tan(lat_rad), 4) - pow(tan(lat_rad), 6));
	double T6 = v * cos(lat_rad)*k0;
	double T7 = v * pow(cos(lat_rad), 3)*k0 / 6.0 * (1.0 - pow(tan(lat_rad), 2) + esqd * pow(cos(lat_rad), 2));
	double T8 = v * pow(cos(lat_rad), 5)*k0 / 120.0 * (5.0 - 18.0 * pow(tan(lat_rad), 2) + pow(tan(lat_rad), 4) + 14.0 * esqd*pow(cos(lat_rad), 2) - 58.0 * esqd*pow(tan(lat_rad)*cos(lat_rad), 2) + 13.0 * pow(esqd, 2)*pow(cos(lat_rad), 4) + 4.0 * pow(esqd, 3)*pow(cos(lat_rad), 6)
		- 64.0 * pow(esqd*tan(lat_rad), 2)*pow(cos(lat_rad), 4) - 24.0 * pow(tan(lat_rad), 2)*pow(esqd, 3)*pow(cos(lat_rad), 6));
	double T9 = v * pow(cos(lat_rad), 7)*k0 / 5040.0 * (61.0 - 479.0 * pow(tan(lat_rad), 2) + 179.0 * pow(tan(lat_rad), 4) - pow(tan(lat_rad), 6));

	double dellng = lng_rad - lamda;
	double N = T1 + pow(dellng, 2)*T2 + pow(dellng, 4)*T3 + pow(dellng, 6)*T4 + pow(dellng, 8)*T5;
	double E = Eoff + dellng * T6 + pow(dellng, 3)*T7 + pow(dellng, 5)*T8 + pow(dellng, 7)*T9;

	vector<double> utm;
	utm.push_back(E);
	utm.push_back(N);
	return utm;
}

//reviving communication when communication is closed
void HeroNeverDies() {
	_rtk.ClosePort();
	if (_rtk.OpenPort(RTKComm)) {
		_rtk.ConfigurePortA(CBR_115200, 8, FALSE, NOPARITY, ONESTOPBIT);
		_rtk.SetCommunicationTimeouts(0, 0, 0, 0, 0);
	}
}


void RTK_Comm() {
	_rtk.OpenPort(RTKComm);
	_rtk.ClosePort();
	ofstream in("RTK_Comm_test.txt");
	if (_rtk.OpenPort(RTKComm)) {
		_rtk.ConfigurePortA(CBR_115200, 8, FALSE, NOPARITY, ONESTOPBIT);
		_rtk.SetCommunicationTimeouts(0, 0, 0, 0, 0);
		if (in.is_open()) {
			while (1) {
				BYTE * pByte = new BYTE[2028];

				if (_rtk.ReadByte(pByte, 2028)) {
					pByte[2027] = '\0';

					const char * p = (const char*)pByte;
					std::cout << "done" << endl;
					in << p;
				}
				else {
					HeroNeverDies();
				}
			}
		}
		in.close();
	}
}

void ValueTrans(int lat_deg, int lng_deg) {
	double heading;
	double _lat, _lng;
	double lat, lng;

	cout << setprecision(16);

	ifstream gpsfile(".txt");  // you should set the path of txt file inlcuding raw data you wanna change to UTM
	ofstream ofile("ValueTrans_test.txt");
	ofile << setprecision(16);

	char line[200];
	string tap;
	vector<string> vec;

	if (gpsfile.is_open()) {
		while (gpsfile.getline(line, sizeof(line), '\n')) {

			stringstream str(line);

			while (getline(str, tap, ',')) {

				vec.push_back(tap);
			}

			if (size(vec) > 8) {

				if (vec[0] == "$GNRMC" && vec[2] == "A") { // you should check the RTK NEAM protocol; ex. $GNRMC, $ GPRMC, $GNGGA etc.

					_lat = ((atof(vec[3].c_str()) - lat_deg * 100) / 60) + lat_deg; 
					_lng = ((atof(vec[5].c_str()) - lng_deg * 100) / 60) + lng_deg; 
					heading = atof(vec[8].c_str());

					vector<double >utm = UTM(_lat, _lng);
					lat = utm[0];
					lng = utm[1];
					ofile << lat << ',' << lng << ',' << heading << endl;
				}
			}
			vec.clear();
		}
	}

	gpsfile.close();
	ofile.close();
	cout << "Raw data to UTM complete" << endl;
}


int main() {

	//  you should change the number of lat_deg, lng_deg below to degree of current positoin
	int lat_deg = 37;
	int lng_deg = 126;
	

	//RTK_Comm();
	//ValueTrans(lat_deg, lng_deg);
}