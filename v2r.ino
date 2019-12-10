#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <DS1307.h>
#include <ArduinoJson.h>


//IPAddress ModbusDeviceIP(192, 168, 3, 41);

#include "Mudbus.h"
#include <ModbusTCP.h>
#define WIZNET_W5100 1
ModbusTCP node(2);
Mudbus Mb;

static const unsigned char table[512] = {                                                        // this table is used to calculate the CRC16
	0x00, 0x00, 0xC0, 0xC1, 0xC1, 0x81, 0x01, 0x40, 0xC3, 0x01, 0x03, 0xC0, 0x02, 0x80, 0xC2, 0x41,
	0xC6, 0x01, 0x06, 0xC0, 0x07, 0x80, 0xC7, 0x41, 0x05, 0x00, 0xC5, 0xC1, 0xC4, 0x81, 0x04, 0x40,
	0xCC, 0x01, 0x0C, 0xC0, 0x0D, 0x80, 0xCD, 0x41, 0x0F, 0x00, 0xCF, 0xC1, 0xCE, 0x81, 0x0E, 0x40,
	0x0A, 0x00, 0xCA, 0xC1, 0xCB, 0x81, 0x0B, 0x40, 0xC9, 0x01, 0x09, 0xC0, 0x08, 0x80, 0xC8, 0x41,
	0xD8, 0x01, 0x18, 0xC0, 0x19, 0x80, 0xD9, 0x41, 0x1B, 0x00, 0xDB, 0xC1, 0xDA, 0x81, 0x1A, 0x40,
	0x1E, 0x00, 0xDE, 0xC1, 0xDF, 0x81, 0x1F, 0x40, 0xDD, 0x01, 0x1D, 0xC0, 0x1C, 0x80, 0xDC, 0x41,
	0x14, 0x00, 0xD4, 0xC1, 0xD5, 0x81, 0x15, 0x40, 0xD7, 0x01, 0x17, 0xC0, 0x16, 0x80, 0xD6, 0x41,
	0xD2, 0x01, 0x12, 0xC0, 0x13, 0x80, 0xD3, 0x41, 0x11, 0x00, 0xD1, 0xC1, 0xD0, 0x81, 0x10, 0x40,
	0xF0, 0x01, 0x30, 0xC0, 0x31, 0x80, 0xF1, 0x41, 0x33, 0x00, 0xF3, 0xC1, 0xF2, 0x81, 0x32, 0x40,
	0x36, 0x00, 0xF6, 0xC1, 0xF7, 0x81, 0x37, 0x40, 0xF5, 0x01, 0x35, 0xC0, 0x34, 0x80, 0xF4, 0x41,
	0x3C, 0x00, 0xFC, 0xC1, 0xFD, 0x81, 0x3D, 0x40, 0xFF, 0x01, 0x3F, 0xC0, 0x3E, 0x80, 0xFE, 0x41,
	0xFA, 0x01, 0x3A, 0xC0, 0x3B, 0x80, 0xFB, 0x41, 0x39, 0x00, 0xF9, 0xC1, 0xF8, 0x81, 0x38, 0x40,
	0x28, 0x00, 0xE8, 0xC1, 0xE9, 0x81, 0x29, 0x40, 0xEB, 0x01, 0x2B, 0xC0, 0x2A, 0x80, 0xEA, 0x41,
	0xEE, 0x01, 0x2E, 0xC0, 0x2F, 0x80, 0xEF, 0x41, 0x2D, 0x00, 0xED, 0xC1, 0xEC, 0x81, 0x2C, 0x40,
	0xE4, 0x01, 0x24, 0xC0, 0x25, 0x80, 0xE5, 0x41, 0x27, 0x00, 0xE7, 0xC1, 0xE6, 0x81, 0x26, 0x40,
	0x22, 0x00, 0xE2, 0xC1, 0xE3, 0x81, 0x23, 0x40, 0xE1, 0x01, 0x21, 0xC0, 0x20, 0x80, 0xE0, 0x41,
	0xA0, 0x01, 0x60, 0xC0, 0x61, 0x80, 0xA1, 0x41, 0x63, 0x00, 0xA3, 0xC1, 0xA2, 0x81, 0x62, 0x40,
	0x66, 0x00, 0xA6, 0xC1, 0xA7, 0x81, 0x67, 0x40, 0xA5, 0x01, 0x65, 0xC0, 0x64, 0x80, 0xA4, 0x41,
	0x6C, 0x00, 0xAC, 0xC1, 0xAD, 0x81, 0x6D, 0x40, 0xAF, 0x01, 0x6F, 0xC0, 0x6E, 0x80, 0xAE, 0x41,
	0xAA, 0x01, 0x6A, 0xC0, 0x6B, 0x80, 0xAB, 0x41, 0x69, 0x00, 0xA9, 0xC1, 0xA8, 0x81, 0x68, 0x40,
	0x78, 0x00, 0xB8, 0xC1, 0xB9, 0x81, 0x79, 0x40, 0xBB, 0x01, 0x7B, 0xC0, 0x7A, 0x80, 0xBA, 0x41,
	0xBE, 0x01, 0x7E, 0xC0, 0x7F, 0x80, 0xBF, 0x41, 0x7D, 0x00, 0xBD, 0xC1, 0xBC, 0x81, 0x7C, 0x40,
	0xB4, 0x01, 0x74, 0xC0, 0x75, 0x80, 0xB5, 0x41, 0x77, 0x00, 0xB7, 0xC1, 0xB6, 0x81, 0x76, 0x40,
	0x72, 0x00, 0xB2, 0xC1, 0xB3, 0x81, 0x73, 0x40, 0xB1, 0x01, 0x71, 0xC0, 0x70, 0x80, 0xB0, 0x41,
	0x50, 0x00, 0x90, 0xC1, 0x91, 0x81, 0x51, 0x40, 0x93, 0x01, 0x53, 0xC0, 0x52, 0x80, 0x92, 0x41,
	0x96, 0x01, 0x56, 0xC0, 0x57, 0x80, 0x97, 0x41, 0x55, 0x00, 0x95, 0xC1, 0x94, 0x81, 0x54, 0x40,
	0x9C, 0x01, 0x5C, 0xC0, 0x5D, 0x80, 0x9D, 0x41, 0x5F, 0x00, 0x9F, 0xC1, 0x9E, 0x81, 0x5E, 0x40,
	0x5A, 0x00, 0x9A, 0xC1, 0x9B, 0x81, 0x5B, 0x40, 0x99, 0x01, 0x59, 0xC0, 0x58, 0x80, 0x98, 0x41,
	0x88, 0x01, 0x48, 0xC0, 0x49, 0x80, 0x89, 0x41, 0x4B, 0x00, 0x8B, 0xC1, 0x8A, 0x81, 0x4A, 0x40,
	0x4E, 0x00, 0x8E, 0xC1, 0x8F, 0x81, 0x4F, 0x40, 0x8D, 0x01, 0x4D, 0xC0, 0x4C, 0x80, 0x8C, 0x41,
	0x44, 0x00, 0x84, 0xC1, 0x85, 0x81, 0x45, 0x40, 0x87, 0x01, 0x47, 0xC0, 0x46, 0x80, 0x86, 0x41,
	0x82, 0x01, 0x42, 0xC0, 0x43, 0x80, 0x83, 0x41, 0x41, 0x00, 0x81, 0xC1, 0x80, 0x81, 0x40, 0x40
};

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
#define btnSELECTlong 6
int adc_key_in = 0;

const unsigned long arduino_id = 160825001;  //aa mm dd ccc - 9 digitos

char* menu[] = { "Arduino ID", "Ethernet IP", "RS485", "Publisher", "Server" };
int indiceMenu = 0;
int indiceMenuOld = indiceMenu;
int limiteMenu = 5;
int numModes = 0;
int actualMode = 0;
int _register = 0;
boolean haveIP = false;

byte mac[] = {
	0x66, 0x69, 0x69, 0x68, 0xDE, 0x01
};
EthernetClient client;
EthernetServer server(80);

char c;
String msg;
String data;

//common variables
boolean dataAvailable;
int dataType = 0; //0-none, 1-chart
int deviation;
int start;
//chart variables
int datalength;
int period;
//discrete variables
int controllable;
int offCons;
int onCons;
String APIUrl, APIPath, APIOn = "", APIOff = "", APIJson = "";
int APIDelay = 1000;
double realCons = 0;
boolean onState = true;
long int lastUpdate = 0;
class Mode {
public:
	int start;
	int onCons;
};

Mode modes[10];

const int chipSelect = 22;

byte block[8] = {
	B11111,
	B11111,
	B11111,
	B11111,
	B11111,
	B11111,
	B11111,
};

byte bulbOFF[8] = {
	B01110,
	B10001,
	B10001,
	B10001,
	B10001,
	B01110,
	B01110,
};

byte bulbON[8] = {
	B01110,
	B11111,
	B11111,
	B11111,
	B11111,
	B01110,
	B01110,
};

byte bulbONl[8] = {
	B00000,
	B00001,
	B00000,
	B00001,
	B00000,
	B00000,
	B00000,
};

byte bulbONr[8] = {
	B00000,
	B10000,
	B00000,
	B10000,
	B00000,
	B00000,
	B00000,
};

byte chart[8] = {
	B00000,
	B00000,
	B00101,
	B01010,
	B10000,
	B00000,
	B00000,
};

byte discrete[8] = {
	B00000,
	B01110,
	B10101,
	B10101,
	B10101,
	B01110,
	B00000,
};

byte contextual[8] = {
	B11000,
	B11100,
	B11110,
	B11111,
	B11110,
	B11100,
	B11000,
};
byte mirror[8] = {
	B00100,
	B10101,
	B00100,
	B01110,
	B00100,
	B10101,
	B00100,
};

byte etherERROR[8] = {
	B00100,
	B00100,
	B01110,
	B00000,
	B01110,
	B00100,
	B00100,
};

byte RSworks[8] = {
	B10111,
	B10101,
	B11101,
	B00000,
	B11101,
	B10110,
	B11111,
};

byte Arrows[8] = {
	B00100,
	B01110,
	B11111,
	B00000,
	B11111,
	B01110,
	B00100,
};

DS1307 rtc(A15, A14);

String nextValuesChange = "";
String lastValuesUpdated = "";

int valueV_O = 0;
double valueA_O = 0;
int valueC_O = 0;
int valueV_RT = 0;
double valueA_RT = 0;
int valueC_RT = 0;

int indexLineSD = 0;
boolean midniht = false;

int pressedKey;

int DBipServer = 116;
boolean publishInWeb = false;
int DBserverPort = 8080;
char DBpageName[] = "/uGIM/saver/saviour.php";
int contServerSend = 0;
const int contServerSend_trigger = 10; //number of seconds
int valueV_RT_server[contServerSend_trigger];
double valueA_RT_server[contServerSend_trigger];
int valueC_RT_server[contServerSend_trigger];

unsigned int contRSbytes = 1;
int sizeRS = 35;
byte bytesRS[35];
int pin_control_1_rs = 23;
int RSaddress = 7;
boolean rsIcon = false;
bool sended = false;

typedef union {
	byte array[4];
	float value;
}
ByteToFloat;

void setup() {
	// set up the LCD's number of columns and rows:
	Serial.begin(9600);

	Serial2.begin(9600);


	pinMode(24, OUTPUT);
	digitalWrite(24, LOW);
	pinMode(pin_control_1_rs, OUTPUT);
	digitalWrite(pin_control_1_rs, LOW);

	rtc.halt(false);

	/*rtc.setDOW(THURSDAY);
	rtc.setTime(14, 07, 00);
	rtc.setDate(25, 11, 2016);
	Serial.println("Clock is seted!");
	return;*/

	rtc.setSQWRate(SQW_RATE_1);
	rtc.enableSQW(true);

	lcd.begin(16, 2);
	lcd.createChar(2, bulbONl);
	lcd.createChar(3, bulbON);
	lcd.createChar(4, bulbONr);
	lcd.createChar(5, bulbOFF);
	lcd.createChar(6, etherERROR);
	lcd.createChar(7, RSworks);
	lcd.createChar(8, Arrows);
	lcd.setCursor(0, 1);

	startEthernet();
	delay(2000);
	dataAvailable = updateMainData();

	lcd.clear();
	if (dataType == 1 && dataAvailable) {
		lcd.setCursor(2, 0);
		lcd.write("LOADING DATA");

		if (start == 0)
			indexLineSD += 1;

		recalculateNextValuesChange();
		lcd.clear();
		publishResults();
	}
	else if (dataType == 2) {
		onState = false;
		if (start == 1)
			onState = true;
		updateDiscreteLoad();
	}
	else if (dataType == 4) {
		onState = false;
		if (start == 1)
			onState = true;
		updateVariableLoad();
	}
	else if (dataType == 3) {

		//Contextual
		lcd.setCursor(2, 0);
		lcd.write("LOADING DATA");

		if (start == 0)
			indexLineSD += 1;

		recalculateNextValuesChange();
		lcd.clear();
		publishResults();
	}
	else if (dataType == 5) {
		recalculateNextValuesChange();

		lcd.clear();
		updateMirrorLoad();
	}
	else if (dataType == 6) {
		recalculateNextValuesChange();

		lcd.clear();
		updateModbusLoad();
	}
	else if (dataType == 7) {
		lcd.clear();
		updateOpalLoad();
	}
	else {
		lcd.setCursor(1, 0);
		lcd.write("NOT CONFIGURED");
	}
}

int countChars(char* s, char c)
{
	return *s == '\0'
		? 0
		: countChars(s + 1, c) + (*s == c);
}

String retJson(JsonObject& root, int max, int i) {
	Serial.println();
	Serial.print(i);
	Serial.print(" Key: ");
	Serial.println(split(APIPath, "/", i));
	Serial.println();
	return i >= max ? root[split(APIPath, "/", i)] : retJson(root[split(APIPath, "/", i)], max, i + 1);
}
bool fstate = true;
int tid = 1;
void ModbusUpdate() {
	lastUpdate = millis()+ APIDelay;

	uint8_t result;

	node.setTransactionID(tid++);           // Not necessary; but good to distinguish each frame individually.
	result = node.readHoldingRegisters(_register, 1);    // Read Holding Registers
												   // Serial.println(result, HEX);
	if (result != 0)
	{
		Serial.println("TimeOut");
	}

	int len = node.getResponseBufferLength();
	//Serial.println("Response Length: " + String(len));// See the length of data packet received.
	for (byte j = 0; j < len; j++)
	{
		realCons = node.getResponseBuffer(j);
		onState = node.getResponseBuffer(j) > 0;
		Serial.print(node.getResponseBuffer(j));       // Inspect the data.
		Serial.print(" ");
	}
	Serial.println();
	node.clearResponseBuffer();

}
int newCons = 0;
void loop() {

	if (dataType == 7)
	{
		Mb.Run();
		newCons = Mb.R[187];
		if (newCons != realCons) realCons = newCons;
		if (lastUpdate < millis()) {
			updateOpalLoad();
		}
	}
	if (dataType == 6 && lastUpdate < millis()) {
		ModbusUpdate();
		updateModbusLoad();
	}





	if ((dataType == 1 || dataType == 3) && dataAvailable) {
		if (!midniht && strcmp(rtc.getTimeStr(), nextValuesChange.c_str()) >= 0) { //-1 still early, 1 already late
			recalculateNextValuesChange();
			publishResults();
		}
		else {
			if (strcmp(rtc.getTimeStr(), nextValuesChange.c_str()) == 0)
				midniht = false;
		}
	}
	if (sended && dataType == 5) {
		String json;
		char endOfHeaders[] = "\r\n\r\n";
		client.find(endOfHeaders);
		while (client.available()) {
			char c = client.read();
			Serial.print(c);
			json += c;
		}

		json.replace("\n", "");
		json.replace("\r", "");
		StaticJsonBuffer<500> jsonBuffer;

		JsonObject& root = jsonBuffer.parseObject(json);
		Serial.println(json);

		//char endOfHeaders[] = "\r\n\r\n";
		//client.find(endOfHeaders);
		//// Allocate JsonBuffer
		//const size_t capacity = JSON_ARRAY_SIZE(3)
		//	+ 8 * JSON_OBJECT_SIZE(1)
		//	+ 4 * JSON_OBJECT_SIZE(4)
		//	+ 300;
		//StaticJsonBuffer<capacity> jsonBuffer;
		//// Parse response
		//JsonObject& root = jsonBuffer.parseObject(client);

		if (!root.success()) {
			Serial.println("parseObject() failed");
			recalculateNextValuesChange();
			//onState = false;
			//sended = false;
		}
		else {
			char path[100];
			APIPath.toCharArray(path, 100);
			int count = countChars(path, '/');


			//auto val = root[split(APIPath, "/", 1)];
			//Serial.print("1: ");
			//Serial.println(split(APIPath, "/", 1));
			//for (int i = 1; i < count-1; i++) {
			//	val = val[split(APIPath, "/", i + 1)];
			//	Serial.print(i+1);
			//	Serial.print(": ");
			//	Serial.println(split(APIPath, "/", i+1));
			//}

			//String current_power = val[split(APIPath, "/", count+1)].as<char*>();

			String var = retJson(root, count + 1, 1) + " W";

			Serial.print(count + 1);
			Serial.print(": ");
			Serial.println(split(APIPath, "/", count + 1));
			Serial.print("Var: ");

			Serial.println(var);
			realCons = var.toDouble();
			realCons = realCons > 999 ? 999 : realCons;
			if (fstate) {
				if (realCons > 0) onState = true;
				else onState = false;
				fstate = false;
			}
			updateMirrorLoad();
			sended = false;
		}
	}

	if (dataType==5 && lastUpdate < millis()) { //-1 still early, 1 already late
		recalculateNextValuesChange();
	}
	pressedKey = read_LCD_buttons();
	if (pressedKey == btnSELECTlong) {
		menuOptions();
	}
	else if (pressedKey == btnSELECT) {
		if (dataType == 2) {
			onState = !onState;
			updateDiscreteLoad();
			unpressButton();
		}
		if (dataType == 4) {
			onState = !onState;
			updateVariableLoad();
			unpressButton();
		}
		if (dataType == 3) {
			onState = !onState;
			updateContextualLoad();
			if (!onState) indexLineSD = 1;
			unpressButton();
		}
		if (dataType == 5) {
			onState = !onState;
			unpressButton();

			char json[100];
			APIJson.toCharArray(json, 100);

			if (onState)
				sendPOST(APIOn, json);
			else
				sendPOST(APIOff, json);
			updateMirrorLoad();
		}
	}
	else if (pressedKey == btnUP && dataType == 4) {
		actualMode++;
		if (actualMode > numModes - 1) actualMode = numModes - 1;
		updateVariableLoad();
		unpressButton();
	}
	else if (pressedKey == btnDOWN && dataType == 4) {
		actualMode--;
		if (actualMode < 0) actualMode = 0;
		updateVariableLoad();
		unpressButton();
	}

	if (dataAvailable && deviation > 0 && dataType != 7) {
		if ((String)rtc.getTimeStr() != lastValuesUpdated) {

			double rand = random(deviation * -1, deviation);
			rand = (rand / 100) + 1;
			valueV_RT = valueV_O * ((rand / 2) + 0.5);

			if (valueC_O > 0) {
				valueC_RT = valueC_O * rand;
				valueA_RT = (double)valueC_RT / (double)valueV_RT;

				if (valueA_RT < 0 || valueC_RT < 0) {
					valueA_RT = 0;
					valueC_RT = 0;
				}
			}

			lastValuesUpdated = rtc.getTimeStr();
			publishResults();
		}
	}

	//Ethernet
	if (haveIP) {
		EthernetClient clients = server.available();

		if (clients) {
			Serial.println("novo cliente");
			boolean clientTrustworthy = false;
			boolean emptyString = false;
			boolean startRec = true;
			File dataFile;
			int loadingSize = 0;
			int loadingIndex = 0;
			int loadingPrinted = 0;

			while (clients) {
				msg = "";
				while (clients.available() > 0) {
					c = clients.read();
					msg += c;
				}

				if (clientTrustworthy) {
					if (startRec) {
						if (msg.charAt(0) == 'c') {
							Serial.println("Order received");
							if (dataType == 2) {
								onState = msg.charAt(1) - 48;
								updateDiscreteLoad();
								Serial.println("onState: ");
								Serial.println(onState);
							}
							if (dataType == 4) {
								onState = msg.charAt(1) - 48;
								updateVariableLoad();
								Serial.println("onState: ");
								Serial.println(onState);
							}
							publishInWeb = msg.charAt(2) - 48;
							Serial.println("publishInWeb: ");
							Serial.println(publishInWeb);
							clients.stop();
							lcd.clear();
							publishResults();
						}
						else {
							lcd.createChar(1, block);
							startRec = false;

							SD.begin(chipSelect);
							if (SD.exists("simdata.txt")) {
								Serial.println("eliminou o ficheiro antigo");
								SD.remove("simdata.txt");
							}
							dataFile = SD.open("simdata.txt", FILE_WRITE);

							lcd.clear();
							lcd.setCursor(3, 0);
							lcd.write("Receiving!");
							lcd.setCursor(0, 1);

							String loadAux = msg.substring(0, msg.lastIndexOf(","));
							loadingSize = loadAux.substring(loadAux.lastIndexOf(",") + 1).toInt();
							loadingIndex = 0;

							delay(25);
							clients.print(arduino_id);
							delay(25);
						}

					}
					else {
						loadingIndex++;
						for (int i = 0; i < map(loadingIndex, 0, loadingSize, 0, 16) - loadingPrinted; i++) {
							lcd.write(byte(1));
							loadingPrinted++;
						}

						delay(25);
						clients.print("OKI");
						delay(25);
					}

					if (msg == "OFF") {
						dataFile.println(msg);
						//delay(10);
						//Serial.println("msg: " + msg);
						//delay(10);
						emptyString = false;
					}
					else if (msg != "" && msg != "OFF") {
						dataFile.println(msg);
						//delay(10);
						//Serial.println("msg: " + msg);
						//delay(10);
						emptyString = false;
					}
					else if (emptyString || msg == "OFF") {
						dataFile.close();
						clients.stop();
						lcd.clear();

						dataAvailable = updateMainData();
						indexLineSD = 0;
						if (dataType == 1 && dataAvailable) {
							lcd.setCursor(2, 0);
							lcd.write("LOADING DATA");
							if (start == 0)
								indexLineSD += 1;
							recalculateNextValuesChange();
							lcd.clear();
							publishResults();
						}
						else if (dataType == 2) {
							onState = start;
							updateDiscreteLoad();
						}
						else if (dataType == 4) {
							onState = start;
							updateVariableLoad();
						}
						else {
							dataAvailable = updateMainData();
							if ((dataType == 1 || dataType == 3) && dataAvailable) {
								lcd.clear();
								lcd.setCursor(2, 0);
								lcd.write("LOADING DATA");
								if (start == 0)
									indexLineSD += 1;
								recalculateNextValuesChange();
								lcd.clear();
								publishResults();
							}
							else if (dataType == 2) {
								lcd.clear();
								onState = start;
								updateDiscreteLoad();
							}
							else if (dataType == 4) {
								lcd.clear();
								onState = start;
								updateVariableLoad();
							}
							else if (dataType == 5) {
								onState = true;
								lcd.clear();
							}
							else if (dataType == 6) {
								onState = true;
								lcd.clear();
							}
							else if (dataType == 7) {
								onState = true;
								lcd.clear();
							}
							else {
								lcd.setCursor(1, 0);
								lcd.write("BAD  RECEPTION");
								lcd.setCursor(0, 1);
								lcd.write("try upload again");
							}
						}
					}
					else if (!emptyString && msg == "") {
						emptyString = true;
						delay(100);
					}
				}
				else if (msg == "startCom") {
					clientTrustworthy = true;
					delay(25);
					clients.print("OKI");
					delay(25);
				}
			}
		}
	}

	//RS485
	if (Serial2.available()) {
		if (Serial2.read() != RSaddress) {
			while (Serial2.read() != -1)
				if (!Serial2.available())
					delay(2);
		}
		else {
			bytesRS[0] = RSaddress;
			if (!Serial2.available())
				delay(2);
			while (Serial2.available()) {
				if (contRSbytes < sizeRS)
					bytesRS[contRSbytes] = Serial2.read();
				contRSbytes++;
				if (!Serial2.available())
					delay(2);
			}
			if (contRSbytes >= 7)
				rsResponse();
			contRSbytes = 1;
		}
	}
}

void rsResponse() {
	if (bytesRS[1] == 3 && bytesRS[2] == 0 && bytesRS[4] == 0) {
		byte response[(5 + bytesRS[5])];
		response[0] = bytesRS[0];
		response[1] = bytesRS[1];
		response[2] = bytesRS[5];
		ByteToFloat converter;

		if (bytesRS[3] == 1) {
			if (bytesRS[5] >= 0x04) {
				//voltagem
				converter.value = valueV_RT;
				response[3] = converter.array[3];
				response[4] = converter.array[2];
				response[5] = converter.array[1];
				response[6] = converter.array[0];
			}
			if (bytesRS[5] >= 0x08) {
				//amperagem
				converter.value = valueA_RT;
				response[7] = converter.array[3];
				response[8] = converter.array[2];
				response[9] = converter.array[1];
				response[10] = converter.array[0];
			}
			if (bytesRS[5] == 0x0C) {
				//consumo
				converter.value = valueC_RT;
				response[11] = converter.array[3];
				response[12] = converter.array[2];
				response[13] = converter.array[1];
				response[14] = converter.array[0];
			}
		}
		else if (bytesRS[3] == 5) {
			if (bytesRS[5] >= 0x04) {
				//amperagem
				converter.value = valueA_RT;
				response[3] = converter.array[3];
				response[4] = converter.array[2];
				response[5] = converter.array[1];
				response[6] = converter.array[0];
			}
			if (bytesRS[5] == 0x08) {
				//consumo
				converter.value = valueC_RT;
				response[7] = converter.array[3];
				response[8] = converter.array[2];
				response[9] = converter.array[1];
				response[10] = converter.array[0];
			}
		}
		else if (bytesRS[3] == 9) {
			if (bytesRS[5] == 0x08) {
				//consumo
				converter.value = valueC_RT;
				response[3] = converter.array[3];
				response[4] = converter.array[2];
				response[5] = converter.array[1];
				response[6] = converter.array[0];
			}
		}
		calculate_CRC(response, (3 + bytesRS[5]));
		delay(40);
		digitalWrite(pin_control_1_rs, HIGH);
		for (int i = 0; i < (5 + bytesRS[5]); i++)
		{
			Serial2.write(response[i]);
		}
		Serial2.flush();
		digitalWrite(pin_control_1_rs, LOW);
		lcd.setCursor(2, 1);
		if (rsIcon)
			lcd.write(" ");
		else
			lcd.write(byte(7));
		rsIcon = !rsIcon;

		/*for (int i = 0; i < (5 + bytesRS[5]); i++)
		{
		Serial.print(" ");
		Serial.print(response[i], HEX);
		}
		Serial.println();*/
	}
}

void calculate_CRC(unsigned char *message, int length)
{
	unsigned char CRCHi, CRCLo, TempHi, TempLo;
	CRCHi = 0xff;
	CRCLo = 0xff;

	while (length)
	{
		TempHi = CRCHi;
		TempLo = CRCLo;
		CRCHi = table[2 * (*message ^ TempLo)];
		CRCLo = TempHi ^ table[(2 * (*message ^ TempLo)) + 1];
		message++;
		length--;
	};
	*message = CRCLo;
	message++;
	*message = CRCHi;

	return;
}

void updateDiscreteLoad() {

	if (onState) {
		valueV_O = 230;
		valueC_O = onCons;
		valueA_O = (double)valueC_O / (double)valueV_O;
	}
	else {
		valueV_O = 230;
		if (offCons != 0) {
			valueC_O = offCons;
			valueA_O = (double)valueC_O / (double)valueV_O;
		}
		else {
			valueC_O = 0;
			valueA_O = 0;
		}
	}

	valueV_RT = valueV_O;
	valueC_RT = valueC_O;
	valueA_RT = valueA_O;
	publishResults();
}



void updateMirrorLoad() {

	if (onState) {
		valueV_O = 230;
		valueC_O = realCons;
		valueA_O = (double)valueC_O / (double)valueV_O;
	}
	else {
		valueV_O = 230;
		valueC_O = 0;
		valueA_O = 0;
	}

	valueV_RT = valueV_O;
	valueC_RT = valueC_O;
	valueA_RT = valueA_O;
	publishResults();
}

void updateModbusLoad() {

	if (onState) {
		valueV_O = 230;
		valueC_O = realCons;
		valueA_O = (double)valueC_O / (double)valueV_O;
	}
	else {
		valueV_O = 230;
		valueC_O = 0;
		valueA_O = 0;
	}

	valueV_RT = valueV_O;
	valueC_RT = valueC_O;
	valueA_RT = valueA_O;
	publishResults();
}
void updateOpalLoad() {
	lastUpdate = millis() + 1000;
	double rand = random(deviation * -1, deviation);
	rand = (rand / 100) + 1;
	valueV_O = 230;
	valueC_O = realCons * rand;
	valueA_O = (double)valueC_O / (double)valueV_O;

	Mb.R[186] = valueC_O;
	valueV_RT = valueV_O * ((rand / 2) + 0.5);
	valueC_RT = valueC_O;
	valueA_RT = valueA_O;
	publishResults();
}

void updateVariableLoad() {

	if (onState) {
		valueV_O = 230;
		valueC_O = modes[actualMode].onCons;
		valueA_O = (double)valueC_O / (double)valueV_O;
	}
	else {
		valueV_O = 230;
		
		Serial.println("is zero ");
		valueC_O = 0;
		valueA_O = 0;
	}

	valueV_RT = valueV_O;
	valueC_RT = valueC_O;
	valueA_RT = valueA_O;
	publishResults();
}


void updateContextualLoad() {

	if (onState) {
		valueV_O = 230;
		valueC_O = onCons;
		valueA_O = (double)valueC_O / (double)valueV_O;
	}
	else {
		valueV_O = 230;
		if (offCons != 0) {
			valueC_O = offCons;
			valueA_O = (double)valueC_O / (double)valueV_O;
		}
		else {
			valueC_O = 0;
			valueA_O = 0;
		}
	}

	valueV_RT = valueV_O;
	valueC_RT = valueC_O;
	valueA_RT = valueA_O;

	publishResults();
}

String split(String ent, String del, int pos) {
	String ret;
	for (int i = 0; i < pos; i++) {
		ret = ent.substring(0, ent.indexOf(del));
		ent = ent.substring(ent.indexOf(del) + 1);
	}
	return ret;
}

void sendGET(String url, bool em = false) {
	String ipport = split(url, "/", 3);

	char _ip[50];
	split(ipport, ":", 1).toCharArray(_ip, 50);
	String _port = split(ipport, ":", 2);


	if (client.connect(_ip, _port.toInt())) {
		Serial.println("Connected");
		client.print("GET ");
		client.println(url);
		client.println("Connection: close");
		client.println();
		if (em) sended = true;
	}
}

void sendPOST(String url, char PostData[]) {
	Serial.print("Sending post to ");
	Serial.println(url);
	Serial.print("With body ");
	Serial.println(PostData);
	String ipport = split(url, "/", 3);

	char _ip[50];
	split(ipport, ":", 1).toCharArray(_ip, 50);
	String _port = split(ipport, ":", 2);


	if (client.connect(_ip, _port.toInt())) {

		client.print("POST ");
		client.print(url);
		client.println(" HTTP/1.1");
		client.println("Connection: close");
		client.print("Content-Length: ");
		client.println(strlen(PostData));// number of bytes in the payload
		client.println();// important need an empty line here 
		client.println(PostData);// the payload


	}
}

void recalculateNextValuesChange() {

	if (dataType == 5) {
		lastUpdate = millis() + APIDelay;

		//http://192.168.2.68:8123/api/states/switch.tplink_frigo
		//http://192.168.2.5:8520/building/energy
		sendGET(APIUrl, true);
		return;
	}

	if (onState && dataType == 3) {
		Serial.println("Returned by end");
		return;
	}
	Serial.println("-----------  ");
	Serial.println(rtc.getTimeStr());

	String dadosOld = "";

	if (indexLineSD == 0 && start == 1) {
		File dataFile;
		SD.begin(chipSelect);
		dataFile = SD.open("simdata.txt");
		if (dataFile.available()) {
			dataFile.readStringUntil('\n');
			indexLineSD++;
			while (true) {
				String dados = dataFile.readStringUntil('\n');
				String time = dados.substring(0, dados.indexOf(","));
				if (time.length() < 8)
					time = "0" + time;

				if (strcmp(rtc.getTimeStr(), time.c_str()) < 0) {
					dataFile.close();

					nextValuesChange = dados.substring(0, dados.indexOf(","));
					dadosOld = dadosOld.substring(dados.indexOf(",") + 1);
					valueV_O = dadosOld.substring(0, dadosOld.indexOf(",")).toInt();
					dadosOld = dadosOld.substring(dadosOld.indexOf(",") + 1);
					valueA_O = dadosOld.substring(0, dadosOld.indexOf(",")).toFloat();
					dadosOld = dadosOld.substring(dadosOld.indexOf(",") + 1);
					valueC_O = dadosOld.substring(0, dadosOld.indexOf(",")).toInt();

					valueV_RT = valueV_O;
					valueC_RT = valueC_O;
					valueA_RT = valueA_O;

					Serial.print("nextValuesChange: ");
					Serial.println(nextValuesChange);
					Serial.print("indexLineSD: ");
					Serial.println(indexLineSD);

					return;
				}
				indexLineSD++;
				dadosOld = dados;
			}
			indexLineSD--;
		}
	}
	else {
		indexLineSD++;

		if (indexLineSD > datalength) {
			Serial.println("midniht");
			indexLineSD = 1;
			midniht = true;
			if (dataType == 3) {
				onState = 1;
				Serial.println("END 3");
				updateContextualLoad();
				return;
			}
		}
		File dataFile;
		SD.begin(chipSelect);
		dataFile = SD.open("simdata.txt");
		if (dataFile.available()) {
			String dados;
			for (int i = 0; i < indexLineSD - 1;) {
				if (dataFile.read() == '\n')
					i++;
			}

			dadosOld = dataFile.readStringUntil('\n');
			dados = dataFile.readStringUntil('\n');

			if (indexLineSD == 1) {
				for (int i = 0; i < datalength - 2;) {
					if (dataFile.read() == '\n')
						i++;
				}
				dadosOld = dataFile.readStringUntil('\n');
			}

			dataFile.close();

			nextValuesChange = dados.substring(0, dados.indexOf(","));
			if (nextValuesChange.length() < 8)
				nextValuesChange = "0" + nextValuesChange;
			dadosOld = dadosOld.substring(dadosOld.indexOf(",") + 1);
			valueV_O = dadosOld.substring(0, dadosOld.indexOf(",")).toInt();
			dadosOld = dadosOld.substring(dadosOld.indexOf(",") + 1);
			valueA_O = dadosOld.substring(0, dadosOld.indexOf(",")).toFloat();
			dadosOld = dadosOld.substring(dadosOld.indexOf(",") + 1);
			valueC_O = dadosOld.substring(0, dadosOld.indexOf(",")).toInt();

			valueV_RT = valueV_O;
			valueC_RT = valueC_O;
			valueA_RT = valueA_O;

			if (start == 0) {
				nextValuesChange = addPeriod(rtc.getTimeStr());
			}

			Serial.print("nextValuesChange: ");
			Serial.println(nextValuesChange);
			Serial.print("indexLineSD: ");
			Serial.println(indexLineSD);
			Serial.print("value: ");
			Serial.println(valueC_O);

			return;
		}
	}
}

String addPeriod(String time) {
	long seconds = 0;
	seconds += time.substring(0, time.indexOf(":")).toInt() * 3600;
	time = time.substring(time.indexOf(":") + 1);
	seconds += time.substring(0, time.indexOf(":")).toInt() * 60;
	time = time.substring(time.indexOf(":") + 1);
	seconds += time.toInt();

	seconds += period;

	if (seconds > 86399)
		seconds -= 86400;

	String result = "";
	int hours = (int)(seconds / 60 / 60);
	if (hours < 10)
		result += "0";
	result += String(hours) + ":";
	seconds = seconds - ((long)hours * 3600);

	int minu = (int)(seconds / 60);
	if (minu < 10)
		result += "0";
	result += String(minu) + ":";
	seconds = seconds - ((long)minu * 60);

	if (seconds < 10)
		result += "0";
	result += String(seconds);

	Serial.println("*************");
	Serial.println(result);

	return result;
}

void publishResults() {
	if (publishInWeb) {
		valueV_RT_server[contServerSend] = valueV_RT;
		valueA_RT_server[contServerSend] = valueA_RT;
		valueC_RT_server[contServerSend] = valueC_RT;
		contServerSend++;
		if (contServerSend >= contServerSend_trigger) {
			contServerSend = 0;

			char params[15 + (contServerSend_trigger * 25)];
			String sf(valueA_RT_server[0], 2);
			String volts = String(valueV_RT_server[0]);
			String amps = sf;
			String watts = String(valueC_RT_server[0]);

			for (int i = 1; i < contServerSend_trigger; i++) {
				String sf(valueA_RT_server[i], 2);
				volts = volts + ";" + valueV_RT_server[i];
				amps = amps + ";" + sf;
				watts = watts + ";" + valueC_RT_server[i];
			}
			sprintf(params, "id=%s&v=[%s]&a=[%s]&w=[%s]", String(arduino_id).c_str(), volts.c_str(), amps.c_str(), watts.c_str());

			//Serial.println(params);

			int auxSize;
			if (DBipServer > 99)
				auxSize = 14;
			else if (DBipServer > 9)
				auxSize = 13;
			else
				auxSize = 12;

			char DBserverName[auxSize];
			sprintf(DBserverName, "192.168.2.%s\0", String(DBipServer).c_str());

			if (!postPage(DBserverName, DBserverPort, DBpageName, params)) {
				Serial.print("ERRO IN DB");
				lcd.setCursor(1, 1);
				lcd.write(byte(6));
			}
			else {
				lcd.setCursor(1, 1);
				lcd.write(' ');
			}
		}
	}

	//load type
	lcd.setCursor(0, 1);
	lcd.write(byte(1));

	//voltage
	lcd.setCursor(1, 0);
	lcd.print("    ");
	if (valueV_RT < 10) {
		lcd.setCursor(4, 0);
	}
	else if (valueV_RT < 100) {
		lcd.setCursor(3, 0);
	}
	else if (valueV_RT < 1000) {
		lcd.setCursor(2, 0);
	}
	else {
		lcd.setCursor(1, 0);
	}
	lcd.print(valueV_RT);
	lcd.print(" V");

	//watts
	lcd.setCursor(8, 0);
	lcd.print("    ");
	if (valueC_RT < 10) {
		lcd.setCursor(11, 0);
	}
	else if (valueC_RT < 100) {
		lcd.setCursor(10, 0);
	}
	else if (valueC_RT < 1000) {
		lcd.setCursor(9, 0);
	}
	else {
		lcd.setCursor(8, 0);
	}
	lcd.print(valueC_RT);
	lcd.print(" W");

	//lamp
	lcd.setCursor(3, 1);
	if (onState) {
		lcd.write(byte(2));
		lcd.write(byte(3));
		lcd.write(byte(4));
	}
	else {
		lcd.print(" ");
		lcd.write(byte(5));
		lcd.print(" ");
	}

	//amperes
	lcd.setCursor(6, 1);
	lcd.print("   ");
	if (valueA_RT < 10) {
		lcd.setCursor(8, 1);
	}
	else if (valueA_RT < 100) {
		lcd.setCursor(7, 1);
	}
	else {
		lcd.setCursor(6, 1);
	}
	lcd.print(valueA_RT);
	lcd.print(" A");


	if (dataType == 4) {
		lcd.setCursor(1, 1);
		lcd.print(actualMode + 1);
	}
}

boolean updateMainData() {
	////////// force values
	/*dataType = 2;
	deviation = 2;
	start = 0;
	controllable = 0;
	onCons = 1500;
	offCons = 0;
	lcd.createChar(1, discrete);
	return true;*/
	////////// force values

	File dataFile;

	SD.begin(chipSelect);
	dataFile = SD.open("simdata.txt");
	delay(50);
	if (dataFile.available()) {
		String dados = dataFile.readStringUntil('\n');
		Serial.println(dados);
		dataType = dados.substring(0, dados.indexOf(",")).toInt();
		Serial.println(dataType);
		dados = dados.substring(dados.indexOf(",") + 1);

		if (dataType == 1) {

			deviation = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			start = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			datalength = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			period = dados.substring(0, dados.indexOf(",")).toInt();
			lcd.createChar(1, chart);

		}
		else if (dataType == 2) {

			deviation = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			start = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			controllable = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			offCons = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			onCons = dados.substring(0, dados.indexOf(",")).toInt();
			lcd.createChar(1, discrete);
		}
		else if (dataType == 3) {

			deviation = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			start = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			datalength = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			period = dados.substring(0, dados.indexOf(",")).toInt();
			lcd.createChar(1, contextual);
		}
		else if (dataType == 4) {

			/*deviation = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			start = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			controllable = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			offCons = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			onCons = dados.substring(0, dados.indexOf(",")).toInt();*/

			deviation = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			controllable = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			numModes = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			int fmode = 0;
			for (int i = 0; i < numModes; i++) {
				Mode mode;
				mode.start = dados.substring(0, dados.indexOf(",")).toInt();

				if (mode.start == 1 && fmode == 0)
					fmode = i;

				dados = dados.substring(dados.indexOf(",") + 1);
				mode.onCons = dados.substring(0, dados.indexOf(",")).toInt();
				dados = dados.substring(dados.indexOf(",") + 1);
				modes[i] = mode;
			}

			Serial.print("Starting in ");
			Serial.println(fmode);
			actualMode = fmode;
			start = modes[fmode].start;
			onCons = modes[0].onCons;

			Serial.println(" ------------------------- ");
			for (int i = 0; i < numModes; i++) {
				Serial.print("Dev: ");
				Serial.println(deviation);

				Serial.print("Start: ");
				Serial.println(modes[i].start);

				Serial.print("Cont: ");
				Serial.println(controllable);

				Serial.print("On: ");
				Serial.println(modes[i].onCons);

				Serial.println(" ----------- ");

			}

			lcd.createChar(1, Arrows);
		}

		//   dataType,controllable,APIUrl,APIPath,APIDelay,APIOn,APIOff
		else if (dataType == 5) {
			controllable = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			APIUrl = dados.substring(0, dados.indexOf(","));
			dados = dados.substring(dados.indexOf(",") + 1);
			APIPath = dados.substring(0, dados.indexOf(","));
			dados = dados.substring(dados.indexOf(",") + 1);
			APIDelay = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			APIOn = dados.substring(0, dados.indexOf(","));
			dados = dados.substring(dados.indexOf(",") + 1);
			APIOff = dados.substring(0, dados.indexOf(","));
			dados = dados.substring(dados.indexOf(",") + 1);
			APIJson = dados.substring(0, dados.indexOf(","));
			dados = dados.substring(dados.indexOf(",") + 1);

			onState = true;

			Serial.println("API Configurations:");
			Serial.print("URL: ");
			Serial.println(APIUrl);
			Serial.print("PATH: ");
			Serial.println(APIPath);
			lcd.createChar(1, mirror);
		}
		else if (dataType == 6) {
			int ip1 = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			int ip2 = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			int ip3 = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			int ip4 = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			APIDelay = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);
			_register = dados.substring(0, dados.indexOf(",")).toInt();
			dados = dados.substring(dados.indexOf(",") + 1);

			IPAddress ModbusDeviceIP(ip1, ip2, ip3, ip4);
			node.setServerIPAddress(ModbusDeviceIP);

			Serial.println(APIPath);
			lcd.createChar(1, mirror);
		}
		else if (dataType == 7) {
			//OPAL
			deviation = 10;
			lcd.createChar(1, mirror);
		}

		dataFile.close();
		if (dataType >= 1 && dataType <= 7)
			return true;
		else
			return false;
	}
	dataFile.close();

	return false;
}

void menuOptions() {
	lcd.clear();
	lcd.setCursor(6, 0);
	lcd.print("MENU");
	lcd.setCursor(0, 1);
	lcd.print(" >               ");
	lcd.setCursor(3, 1);
	lcd.write(menu[indiceMenu]);
	unpressButton();

	boolean stayInMenu = true;
	long inactivity = millis();
	boolean insideMenu = false;

	while (stayInMenu) {

		if ((millis() - inactivity) > 15000)
			stayInMenu = false;

		if (indiceMenu != indiceMenuOld) {
			lcd.setCursor(0, 1);
			lcd.print(" >               ");
			lcd.setCursor(3, 1);
			lcd.write(menu[indiceMenu]);
			unpressButton();
			indiceMenuOld = indiceMenu;
		}

		switch (read_LCD_buttons()) {
		case btnUP:
			inactivity = millis();
			if (insideMenu) {
				unpressButton();
				switch (indiceMenu) {
				case 2:
					RSaddress++;
					if (RSaddress > 99)
						RSaddress = 0;
					lcd.setCursor(0, 1);
					lcd.print(RSaddress);
					lcd.print(" ");
					break;
				case 3:
					publishInWeb = !publishInWeb;
					lcd.setCursor(0, 1);
					if (publishInWeb)
						lcd.print("online");
					else
						lcd.print("offline");
					lcd.print(" ");
					break;
				case 4:
					DBipServer++;
					if (DBipServer > 254)
						DBipServer = 1;
					lcd.setCursor(10, 1);
					lcd.print(DBipServer);
					lcd.print("  ");
					break;
				}
			}
			else {
				indiceMenu++;
				if (indiceMenu >= limiteMenu)
					indiceMenu = 0;
			}
			break;
		case btnDOWN:
			inactivity = millis();
			if (insideMenu) {
				unpressButton();
				switch (indiceMenu) {
				case 2:
					RSaddress--;
					if (RSaddress < 0)
						RSaddress = 99;
					lcd.setCursor(0, 1);
					lcd.print(RSaddress);
					lcd.print(" ");
					break;
				case 3:
					publishInWeb = !publishInWeb;
					lcd.setCursor(0, 1);
					if (publishInWeb)
						lcd.print("online");
					else
						lcd.print("offline");
					lcd.print(" ");
					break;
				case 4:
					DBipServer--;
					if (DBipServer < 1)
						DBipServer = 254;
					lcd.setCursor(10, 1);
					lcd.print(DBipServer);
					lcd.print("  ");
					break;
				}
			}
			else {
				indiceMenu--;
				if (indiceMenu < 0)
					indiceMenu = limiteMenu - 1;
			}
			break;
		case btnRIGHT:
			inactivity = millis();
			switch (indiceMenu) {
			case 0:
				lcd.setCursor(0, 1);
				lcd.print("                ");
				lcd.setCursor(0, 1);
				lcd.print(arduino_id);
				Serial.println(arduino_id);
				break;
			case 1:
				if (haveIP) {
					lcd.setCursor(0, 1);
					lcd.print("                ");
					lcd.setCursor(0, 1);
					lcd.print(Ethernet.localIP());
					Serial.println(Ethernet.localIP());
				}
				else {
					startEthernet();
				}
				break;
			case 2:
				insideMenu = true;
				lcd.setCursor(0, 1);
				lcd.print("                ");
				lcd.setCursor(0, 1);
				lcd.print(RSaddress);
				lcd.setCursor(3, 1);
				lcd.write(byte(8));
				break;
			case 3:
				insideMenu = true;
				lcd.setCursor(0, 1);
				lcd.print("                ");
				lcd.setCursor(0, 1);
				if (publishInWeb)
					lcd.print("online");
				else
					lcd.print("offline");
				lcd.print(" ");
				lcd.setCursor(9, 1);
				lcd.write(byte(8));
				break;
			case 4:
				insideMenu = true;
				lcd.setCursor(0, 1);
				lcd.print("                ");
				lcd.setCursor(0, 1);
				lcd.print("192.168.2.");
				lcd.print(DBipServer);
				lcd.setCursor(15, 1);
				lcd.write(byte(8));
				break;
			}
			break;
		case btnLEFT:
			insideMenu = false;
			lcd.clear();
			lcd.createChar(2, bulbONl);
			lcd.createChar(3, bulbON);
			lcd.createChar(4, bulbONr);
			lcd.createChar(5, bulbOFF);
			lcd.createChar(6, etherERROR);
			lcd.createChar(7, RSworks);
			publishResults();
			return;
			break;
		}
	}

	lcd.clear();
	if (dataType == 0) {
		lcd.setCursor(1, 0);
		lcd.write("NOT CONFIGURED");
		return;
	}
	lcd.createChar(2, bulbONl);
	lcd.createChar(3, bulbON);
	lcd.createChar(4, bulbONr);
	lcd.createChar(5, bulbOFF);
	lcd.createChar(6, etherERROR);
	lcd.createChar(7, RSworks);
	publishResults();
}

void startEthernet() {
	kit();
	lcd.setCursor(0, 1);
	lcd.print("  ");

	if (Ethernet.begin(mac) == 0) {
		lcd.setCursor(0, 0);
		lcd.write("Failed Ethernet");

	}
	else {
		haveIP = true;
		lcd.print(Ethernet.localIP());
		Serial.print("IP: ");
		Serial.println(Ethernet.localIP());
	}
	server.begin();
}

void kit() {
	int delayTime = 100;

	lcd.setCursor(3, 0);
	lcd.write("Connecting");

	lcd.setCursor(0, 1);
	lcd.write('.');
	for (int i = 1; i <= 15; i++) {
		if (i >= 2) {
			lcd.setCursor(i - 2, 1);
			lcd.write(' ');
		}
		lcd.setCursor(i, 1);
		lcd.write('.');
		delay(delayTime);
	}
	for (int i = 14; i >= 0; i--) {
		lcd.setCursor(i, 1);
		lcd.write('.');
		lcd.setCursor(i + 2, 1);
		lcd.write(' ');
		delay(delayTime);
	}
}

int read_LCD_buttons()
{
	adc_key_in = analogRead(0);
	delay(5);
	int k = (analogRead(0) - adc_key_in);

	if (5 < abs(k)) return btnNONE;
	if (adc_key_in > 1000) return btnNONE;
	if (adc_key_in < 50)   return btnRIGHT;
	if (adc_key_in < 250)  return btnUP;
	if (adc_key_in < 450)  return btnDOWN;
	if (adc_key_in < 650)  return btnLEFT;
	if (adc_key_in < 850) {
		long startPressing = millis();
		int testSelect = analogRead(0);
		while (testSelect > 650 && testSelect < 850) {
			testSelect = analogRead(0);
			if ((millis() - startPressing) > 1000)
				return btnSELECTlong;
		}
		return btnSELECT;
	};
	return btnNONE;
}

void unpressButton() {
	while (analogRead(0) < 1000) delay(50);
}

byte postPage(char* domainBuffer, int thisPort, char* page, char* thisData)
{
	int inChar;
	char outBuf[150];
	client.setTimeout(600);
	if (client.connect(domainBuffer, thisPort) == 1)
	{
		// send the header
		sprintf(outBuf, "POST %s HTTP/1.1", page);
		client.println(outBuf);
		sprintf(outBuf, "Host: %s", domainBuffer);
		client.println(outBuf);
		client.println(F("Connection: close\r\nContent-Type: application/x-www-form-urlencoded"));
		sprintf(outBuf, "Content-Length: %u\r\n", strlen(thisData));
		client.println(outBuf);

		// send the body (variables)
		client.print(thisData);
	}
	else
	{
		return 0;
	}

	int connectLoop = 0;
	int res = 1;

	while (client.connected())
	{
		while (client.available())
		{
			inChar = client.read();
			connectLoop = 0;
			if (inChar == 'b') {
				if (client.available()) {
					inChar = client.read();
					if (inChar == 'r') {
						if (client.available()) {
							inChar = client.read();
							if (inChar == 'e') {
								if (client.available()) {
									inChar = client.read();
									if (inChar == 'a') {
										if (client.available()) {
											inChar = client.read();
											if (inChar == 'k') {
												res = 0;
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		delay(1);
		connectLoop++;
		if (connectLoop > 300)
		{
			client.stop();
		}
	}

	client.stop();
	return res;
}