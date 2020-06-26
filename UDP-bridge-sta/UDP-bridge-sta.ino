// ESP8266 WiFi (UDP) <-> UART Bridge
// by Daniel McArthur

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Configure Serial Comms
#define BAUD_UART 921600
#define TIMEOUT_UART 500 // us (if delay in UART data > timeout, send UDP packet)
#define BUF_SIZE 8192    // Data receipt buffer size

// Set up Station
IPAddress static_ip(192,168,1,2);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
const char *ssid = "ESPBridge";  // WiFi SSID
const char *pw = "bridge123";    // WiFi PASSWORD
const int port_udp = 8888;

// Set up built-in LED for user feedback
int pin_led = D0;        // LED indicates when UDP data is received
unsigned long t_udp = 0; // Timer since last UDP data
unsigned long t_led = 0; // Timer to throttle LED changes
//////////////////////////////////////////////////////////////////////////

WiFiUDP udp;
IPAddress ip_AP(192, 168, 1, 1);

// Prepare data buffers
uint8_t buf_udp[BUF_SIZE];
uint8_t buf_uart[BUF_SIZE];
uint16_t i_uart=0;

void setup() {
  
  // Start UART Serial connection
  Serial.begin(BAUD_UART);

  // Enable led
  pinMode(pin_led, OUTPUT);
  digitalWrite(pin_led, HIGH); // ON/OFF are reversed for this LED

  // Start up the Station
  WiFi.mode(WIFI_STA);
  WiFi.config(static_ip, gateway, subnet);
  WiFi.begin(ssid, pw);
  Serial.print("Attempting connection to "); Serial.println(ssid); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(100); // Wait until Station is connected to AP
    //Serial.print(".");
  }
  //Serial.println("");
  
  // Start UDP Server
  Serial.print("\nStation forwarding UDP to UART: "); Serial.print(static_ip);
  Serial.print(":"); Serial.print(port_udp); Serial.println(" --> UART0");
  Serial.print("Station forwarding UART to UDP: "); Serial.print("UART0 --> ");
  Serial.print(ip_AP); Serial.print(":"); Serial.println(port_udp);

  udp.begin(port_udp);
}


void loop() {

  // If there’s UDP data available, forward to UART
  int packet_size = udp.parsePacket();
  if(packet_size > 0) {
    //remoteIp = udp.remoteIP(); // store the ip of the remote device
    udp.read(buf_udp, BUF_SIZE);
    
    // Forward data from UDP on to UART
    Serial.write(buf_udp, packet_size);

    t_udp = millis(); // Update time of last UDP data
  }

  // If there's UART Serial data, forward to UDP
  if(Serial.available()) {

    // Read the data until there's a pause of TIMEOUT_UART or longer
    while(1) {
      if(Serial.available()) {
        buf_uart[i_uart] = (char)Serial.read(); // read char from UART
        if(i_uart<BUF_SIZE-1) {
          i_uart++;
        }
      } 
      else {
        // Wait TIMEOUT_UART microseconds, then check for more data
        delayMicroseconds(TIMEOUT_UART);
        //Serial.println("dl");
        //delay(TIMEOUT_UART);
        if(!Serial.available()) {
          //Serial.println("bk");
          break;
        }
      }
    }

    // Forward UART Serial data to UDP port 
    udp.beginPacket(ip_AP, port_udp); // remote IP and port
    udp.write(buf_uart, i_uart);
    udp.endPacket();
    i_uart = 0;
  }

  // Automatically reconnect if lose WiFi
  if(WiFi.status() != WL_CONNECTED) {WiFi.begin(ssid, pw);}
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
    //Serial.print(".");
  }

  // Indicate connection status via built-in LED
  if(millis() - t_led >= 300 ) {  // Update LED status every 300 ms
    if(millis() - t_udp < 1000) { // If UDP data received in last second
      digitalWrite(pin_led, LOW);
    } else {
      digitalWrite(pin_led, HIGH);
    }
    t_led = millis();
  }
 }
