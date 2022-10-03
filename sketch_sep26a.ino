#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


const char* ssid = "Sophie";          // tên wifi của bạn sẽ sử dụng
const char* password = "aaaaaaaaaa"; // mật khẩu của wifi
String command = "";

#define MQTT_SERVER "192.168.1.17"    // Địa chỉ ip của máy tính của bạn ( đây sẽ là MQTT sever)
#define MQTT_PORT 1884                // PORT mở trên node red
#define MQTT_USER "originallyus"     // user name bạn sử dụng trên node red
#define MQTT_PASSWORD "originallyus" // password trên node red MQTT

#define MQTT_TEMPERATURE_TOPIC "nodeWiFi32/dht11/temperature" // TOPIC để Node red đọc giá trị mà ESP32 gửi lên ( Ở đây là nhiệt độ)
#define MQTT_HUMIDITY_TOPIC "nodeWiFi32/dht11/humidity"       // TOPIC để Node red đọc giá trị mà ESP32 gửi lên( Ở đây là độ ẩm)
#define MQTT_LED_TOPIC "nodeWiFi32/led"

#define LEDPIN 27  //khai báo chân led sẽ sử dụng làm OUTPUT
#define LEDPIN1 26 //khai báo chân led sẽ sử dụng làm OUTPUT
#define DHTPIN 17  //khai báo chân IO sẽ sử dụng làm chân đọc tín hiệu DHT11 (INPUT)
DHT dht;

unsigned long previousMillis = 0;  // khai báo biến
unsigned long currentMillis = 0;   // khai báo biến
const long delay_time = 5000;     // thời gian tính bằng mili second
int current_ledState = LOW;       // khởi tạo trạng thái OUTPUT
int last_ledState = LOW;          // khởi tạo trạng thái OUTPUT

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup_wifi() { // hàm setup wifi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password); // tên và mật khẩu đã được khai báo ở trên

  while (WiFi.status() != WL_CONNECTED) { // Kiểm tra xem có kết nối với wifi chưa. Nếu chưa thì sẽ tiếp tục cố kết nối ( Nên kiểm tra lại ssid và password)
    delay(500);
    Serial.print(".");
  }
  
//  randomSeed(micros());
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void connect_to_broker() {            //Kiểm tra xem có kết nối được với sever MQTT không
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "nodeWiFi32";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      client.subscribe(MQTT_LED_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}


void setupMQTT()
{
  client.setServer(MQTT_SERVER, MQTT_PORT ); //Setup sever MQTT
  client.setCallback(callback);
  connect_to_broker();
}

void handleCommand(String cmd) // Hàm được gọi khi hàm callback được gọi, ở đây chúng ta làm một việc đơn giản là so sánh giá trị của dữ liệu truyền vào và lưu lại trạng thái đèn mong muốn.
{
  if (cmd == "on") current_ledState = HIGH;
  if (cmd == "off") current_ledState = LOW;  
}

 

void callback(char* topic, byte *payload, unsigned int length) { // hàm callback là hàm nhận giá trị mà MQTT trên node red trả về. Ở đây data nhận được là dạng byte nghĩa là theo từng kí tự. Vì vậy nếu muốn truyền 1 chuỗi dữ liệu thì chúng ta phải tạo ra 1 chuỗi khác để lưu giá trị.
  Serial.println("-------new message from broker-----");
  Serial.print("topic: ");
  Serial.println(topic);
  Serial.print("message: ");
  Serial.write(payload, length);
  Serial.println();
  
  for(int i=0; i < length; i++)
  {
    char c = (char)payload[i];
    command += c;
  }
  
  Serial.println("command: " + String(command));
  
  handleCommand(command); // Khi lưu được giá trị của chuỗi data truyền xuống thì chúng ta gọi một hàm mới để thực hiện những việc chúng ta muốn ( NÊN GỌI HÀM MỚI VÌ KHÔNG CHỈ ĐƠN GIẢN LÀ LÀM MỘT VIỆC )
  command = "";
}



void setupOtaWifi() { // Hàm setup OAT wifi
  Serial.println("Booting");
    WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  ArduinoOTA.setHostname("ESP");
  ArduinoOTA.setPassword("admin");//Cần ở lần đầu tiên nạp code OTA cho chip
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  //Báo lỗi khi không nạp code qua OTA được
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
  ArduinoOTA.begin(); //Khởi tạo OTA
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  }

void loopOtaWifi()
{
    ArduinoOTA.handle();
}

bool is = false;
ICACHE_RAM_ATTR void  isr()
{
  is = true;
}



void setup() {
  Serial.begin(115200); // Khởi tạo Serial

  pinMode(25,INPUT);
  attachInterrupt(digitalPinToInterrupt(25),isr,RISING);

  pinMode(LEDPIN,OUTPUT);
  pinMode(LEDPIN1,OUTPUT);
  pinMode(DHTPIN,INPUT);

  setup_wifi(); // gọi hàm setup Wifi
  setupOtaWifi(); // gọi hàm setup OTA
  
  setupMQTT();
  
  dht.setup(DHTPIN); // Khởi tạo chân 17 là chân đọc tín hiệu của DHT11
  
}


void loop() 
{
  client.loop();
  
  loopOtaWifi();//Gọi hàm loopOAT wifi
  
  if (!client.connected()) 
  {
    connect_to_broker();
  }
  if (last_ledState != current_ledState) // Kiểm tra trạng thái mới có giống trạng thái đèn cũ không
  {
    last_ledState = current_ledState; // Lưu trạng thái led hiện tại vào 1 biến tạm và dùng để so sánh
    digitalWrite(LEDPIN, current_ledState); //Thực hiện bật/tắt trạng thái đèn theo MQTT.
    digitalWrite(LEDPIN1, current_ledState);//Thực hiện bật/tắt trạng thái đèn theo MQTT.
    Serial.print("LED state is ");
    Serial.println(current_ledState);
  }
  
  currentMillis = millis(); // hàm millis có nhiệm vụ trả về một số - là thời gian (tính theo mili giây) kể từ lúc mạch Arduino bắt đầu chương trình của bạn.
  if (currentMillis - previousMillis > delay_time)  // sau 5s gui 1 lan ( delay_time = 5000 )
  {
    float h = dht.getHumidity();    // Hàm trong thư viện DHT11 giúp đọc giá trị độ ẩm
    float t = dht.getTemperature();// Hàm trong thư viện DHT11 giúp đọc giá trị nhiệt độ
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t))   // Khi ESP32 không đọc được giá trị của cảm biến ( Nên kiểm tra chân khai báo sau đó là phần cứng xem có lỗi gì không)
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      client.publish(MQTT_TEMPERATURE_TOPIC, "Failed to read from DHT sensor!");
      client.publish(MQTT_HUMIDITY_TOPIC, "Failed to read from DHT sensor!");
    } 
    else // Gửi lên sever MQTT theo topic
    {  
      client.publish(MQTT_TEMPERATURE_TOPIC, String(t).c_str()); // c_str() là hàm để chuyển từ String về char* Gửi lên sever MQTT về giá trị nhiệt độ bằng topic "nodeWiFi32/dht11/temperature"
      client.publish(MQTT_HUMIDITY_TOPIC, String(h).c_str()); // Gửi lên sever MQTT về giá trị độ ẩm bằng topic "nodeWiFi32/dht11/humidity"
    }
    previousMillis = millis();
  }

  if (is)
  {
    digitalWrite(LEDPIN1,!digitalRead(LEDPIN1));
    digitalWrite(LEDPIN,!digitalRead(LEDPIN));
    is = false;
  }
   
  
}
