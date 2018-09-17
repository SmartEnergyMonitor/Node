//#include <MAX1278ch.h>
#include <EmonLibI2C.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>

//---------------------------------------------------------------------
// Defeni��o de constantes
//---------------------------------------------------------------------
#define SENSOR_MAX 10				  //N�mero m�ximo de sensores
#define I2CBASEADDR 0x50			//Endere�o I2C base do MAX127 01010000
#define READ_INTERVAL 5000		//Intervalo de tempo entre as leituras	

//voltage = tens�o (V)
//current = amperagem (I)
#define VOLTAGE 0
#define CURRENT 1
#define ANALOG 0
#define I2C 1
//---------------------------------------------------------------------


//---------------------------------------------------------------------
//Topicos
//---------------------------------------------------------------------
//Topico Ready
char topicReady[] = "ready";

//Topico de chegada para receber o ID
char incomingTopic[25];

//Topico de chegada para receber as configura��es
char configTopic[50];

//Topico de envio de leituras
char outboundTopic[] = "readings";

//Topico de EOT
char EOTTopic[25];
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Vari�veis auxiliares
//---------------------------------------------------------------------
uint8_t mac[6];					      //buffer para converter o mac address para String
bool statusReady = false;		  //flag de status, pronto a ler sensores
bool ignoreFirstCycle = true;	//ignora o primeiro ciclo. Permite estabilizar os valores de leitura ap�s intanciar os obejctos EnergyMonitor.
static int sensorNumber = 0;	//N�mero de sensores inst�nciados
static int readCounter = 0;		//Contador do n�mero de leituras a efectuar entre cada transmi��o
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Vari�veis de tempo
//---------------------------------------------------------------------
unsigned long lastMillis = 0;	//tempo decorrido desde a ultima leitura
unsigned long timeSinceReset = 0;
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Declara��o de objectos de rede
//---------------------------------------------------------------------
IPAddress server(192, 168, 4, 1);	//IP do broker MQTT
WiFiClient net;						        //Cliente WiFi
PubSubClient client(net);			    //Cliente MQTT
//---------------------------------------------------------------------

//---------------------------------------------------------------------
//Sensor
//---------------------------------------------------------------------
class Sensor {
public:
	Sensor() {}

	//set and get
	void setID(int _ID) { ID = _ID; }
	int getID() { return ID; }

	void setAmp(int _Amp) { Amp = _Amp; }
	int getAmp() { return Amp; }

	void setType(int _type) { type = _type; }
	int getType() { return type; }

	void setConn(int _conn) { conn = _conn; }
	int getConn() { return conn; }

	byte setI2C(byte _i2c) { i2c = _i2c; }
	int getI2C() { return i2c; }

	void setChannel(int _ch) { ch = _ch; }
	int getChannel() { return ch; }

	void setReading(int index, double _reading) { reading[index] = _reading; }
	double getReading(int index) { return reading[index]; }

	double readingAVR() {
		double sum = 0;
		double average = 0;
		size_t n = sizeof reading / sizeof *reading;
		for (int i = 0; i < n ; i++) {
			sum += reading[i];
		}
		average = sum / (float)n;
		return average;
	}

protected:
private:
	int ID;				//ID do sensor na base de dados
	int Amp;			//Valor da Amperagem do sensor. Utilizado como constante de calibra��o dos SCT pela biblioteca Emonlib.h
	int type;			//Tipo de leitura, tens�o ou corrente. N�o utilizado de momento, �til para modifica��o futura
	int conn;			//Tipo de connec��o, anal�gica ou I2C
	byte i2c;			//Originalmente o endere�o I2C da placa do sensor. Agora apenas os 3 bits referentes ao n�mero da placa. Utilizado com um "or" na constante I2CBASEADDR
	int ch;				//Canal I2C
	double reading[6];	//Array com leituras
};
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Arrays de sensores
//---------------------------------------------------------------------
Sensor sensorArray[SENSOR_MAX];
EnergyMonitor emonArray[SENSOR_MAX];	//O objecto Emonlib t�m de ser instanciado � parte dos sensores, uma vez que necessita de p�rametros recebidos por callback.
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Inicializa os objectos EnergyMonitor. Chamada ap�s chegada de mensagem EOT
//---------------------------------------------------------------------
void initEmon() {
	for (int i = 0; i < SENSOR_MAX; i++) {
		int address = I2CBASEADDR | (sensorArray[i].getI2C() << 1);
		emonArray[i].currentI2C(address >> 1, sensorArray[i].getChannel(), sensorArray[i].getAmp());
	}
	Serial.println("\nInit ok");
}
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Retorna o MAC Address em forma de string
//---------------------------------------------------------------------
String macToStr(const uint8_t* mac) {
	String result;
	for (int i = 0; i < 6; ++i)
	{
		result += String(mac[i], 16);
		if (i < 5)
			result += ':';
	}
	return result;
}
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Connec��o ao broker MQTT.
// Inicia a liga��o WiFi. Inicia a connec��o ao broker. Subscreve o t�pico de chegada do ID (incomingTopic)
// id�ntico ao MAC Address. Publica o MAC Address para o topicReady .
//---------------------------------------------------------------------
void connect() {
	Serial.print("checking wifi...");
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(1000);
	}

	Serial.print("\nconnecting...");
	while (!client.connect("arduinoClient")) {
		Serial.print(".");
		delay(1000);
	}
	WiFi.macAddress(mac);
	Serial.println("\nconnected!");

	String macAddressStr = macToStr(mac);
	macAddressStr.toCharArray(incomingTopic, macAddressStr.length() + 1);
	client.subscribe(incomingTopic);
	Serial.print("Sub Topic:");
	Serial.println(incomingTopic);
	delay(1000);
	client.publish(topicReady, (char*)macAddressStr.c_str());
	Serial.print("Pub Topic:");
	Serial.println(topicReady);

}
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Restabelece a liga��o
//---------------------------------------------------------------------
void reconnect() {
	Serial.print("checking wifi...");
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(1000);
	}

	Serial.print("\nconnecting...");
	while (!client.connect("arduinoClient")) {
		Serial.print(".");
		delay(1000);
	}
}
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Callback de chegada de mensagem.
//---------------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
	String topicStr = String(topic);
	// T�pico de chegada do ID
	if (topicStr.equals(incomingTopic)) {
		String topicStr = String(topic);
		Serial.print("Topico: ");
		Serial.println(topicStr);
		String payloadStr = "";
		String configTopicStr = "";
		String EOTTopicStr = "";
		for (int i = 0; i < length; i++) {
			payloadStr += (char)payload[i];
			configTopicStr += (char)payload[i];
			EOTTopicStr += (char)payload[i];
		}
		configTopicStr += "/#";		//Determina o t�pico de chegada das configura��es ("$ID/#")
		EOTTopicStr += "/EOT";		//Determina o t�pico de fim da transmi��o ("$ID/EOT")
		configTopicStr.toCharArray(configTopic, configTopicStr.length() + 1);
		EOTTopicStr.toCharArray(EOTTopic, EOTTopicStr.length() + 1);
		// Subscreve o t�pico de chegada das configura��es (inclui o t�pico EOT)
		client.subscribe(configTopic);
		Serial.print("Sub Topic:");
		Serial.println(configTopic);
		Serial.print("Recepcao de mensagem-> ID: ");
		Serial.println(payloadStr);
		// Publica o ID para o Broker
		client.publish("NodeID", (char*)payloadStr.c_str());
	}
	// T�pico EOT. Indica o fim da transmiss�o de configura��es
	else if (topicStr.equals(EOTTopic)) {
		delay(5000);
		initEmon();
		statusReady = true;
	}
	// Outro t�pico ("$ID/#", excepto "$ID/EOT") - Recebe e processa os par�metros de configura��o
	else {
		String topicStr = String(topic);
		int index1 = topicStr.indexOf('S');
		int index2 = topicStr.indexOf('/', index1 + 1);
		String sensor = topicStr.substring(index1 + 1, index2);
		int sensorIndex = sensor.toInt();
		if (sensorNumber < sensorIndex) { sensorNumber = sensorIndex; }
		String parameter = topicStr.substring(index2 + 1);
		String payloadStr = "";
		for (int i = 0; i < length; i++) {
			payloadStr += (char)payload[i];
		}
		if (parameter.equals("ID")) { sensorArray[sensorIndex].setID(payloadStr.toInt()); }				//T�pico:"$ID/ID"
		if (parameter.equals("AMP")) {																	//T�pico:"$ID/AMP"
			if (payloadStr.toInt() == 5) { sensorArray[sensorIndex].setAmp(5); }
			else if (payloadStr.toInt() == 20) { sensorArray[sensorIndex].setAmp(15); }
		}			
		if (parameter.equals("Type")) { sensorArray[sensorIndex].setType(payloadStr.toInt()); }			//T�pico:"$ID/Type"
		if (parameter.equals("Conn")) { sensorArray[sensorIndex].setConn(payloadStr.toInt()); }			//T�pico:"$ID/Conn"
		if (parameter.equals("I2C")) { sensorArray[sensorIndex].setI2C(payloadStr.toInt()); }			//T�pico:"$ID/I2C"
		if (parameter.equals("Channel")) { sensorArray[sensorIndex].setChannel(payloadStr.toInt()); }	//T�pico:"$ID/Channel"
	}
}
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Setup
//---------------------------------------------------------------------
void setup() {
	Serial.begin(115200);

	WiFiManager wifiManager;
	//wifiManager.resetSettings();		//faz reset �s credenciais de rede guardadas em EPROM.
	wifiManager.setTimeout(180);

	if (!wifiManager.autoConnect("Energy Monitor Node")) {
		Serial.println("failed to connect and hit timeout");
		delay(3000);
		ESP.restart();
		delay(5000);
	}

	client.setServer(server, 1883);
	client.setCallback(callback);

	connect();
}
//---------------------------------------------------------------------

//---------------------------------------------------------------------
// Main loop
//---------------------------------------------------------------------
void loop() {

	client.loop();

	if (!client.connected()) {
		reconnect();
	}

	if (statusReady) {

		if (millis() - lastMillis > READ_INTERVAL) {
			lastMillis = millis();
			Serial.println("-------------------------");
			for (int i = 0; i <= sensorNumber; i++) {
				Serial.print("ID:"); Serial.println(sensorArray[i].getID());
				Serial.print("AMP"); Serial.println(sensorArray[i].getAmp());
				if (sensorArray[i].getConn() == I2C) {
					double Irms = emonArray[i].calcIrmsI2C(1480);
					Serial.print("Corrente: ");
					Serial.println(Irms);
					sensorArray[i].setReading(readCounter, Irms);
					if (readCounter == 5) {	
						if (ignoreFirstCycle) {
							ignoreFirstCycle = false;
							break;
						}
						double watth = ((sensorArray[i].readingAVR())*230)/1000;
						String payload;
						payload += sensorArray[i].getID();
						payload += ";";
						payload += watth;
						Serial.println(payload);
						while (!client.connected()) {
							reconnect();
						}
						client.publish(outboundTopic, (char*)payload.c_str());
					}
				}
				Serial.println("-------------------------");
				delay(100);
			}
			readCounter++;
			if (readCounter == 6) { readCounter = 0; }
		}
		//Necess�rio para estabilizar as leituras de Corrente ap�s inicializa��o do objecto EnergyMonitor
		else {
			for (int i = 0; i <= sensorNumber; i++) {
				if (sensorArray[i].getConn() == I2C) {
					emonArray[i].calcIrmsI2C(1480);
					delay(1000);
				}
			}
		}
	}
}
//---------------------------------------------------------------------
