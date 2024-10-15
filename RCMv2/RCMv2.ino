//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information see this page: https://github.com/RCMgames

/**
uncomment one of the following lines depending on which hardware you have
Remember to also choose the "environment" for your microcontroller in PlatformIO
*/
// #define RCM_HARDWARE_VERSION RCM_ORIGINAL // versions 1, 2, 3, and 3.1 of the original RCM hardware // https://github.com/RCMgames/RCM_hardware_documentation_and_user_guide
// #define RCM_HARDWARE_VERSION RCM_4_V1 // version 1 of the RCM 4 // https://github.com/RCMgames/RCM-Hardware-V4
#define RCM_HARDWARE_VERSION RCM_BYTE_V2 // version 2 of the RCM BYTE // https://github.com/RCMgames/RCM-Hardware-BYTE
// #define RCM_HARDWARE_VERSION RCM_NIBBLE_V1 // version 1 of the RCM Nibble // https://github.com/RCMgames/RCM-Hardware-Nibble
// #define RCM_HARDWARE_VERSION RCM_D1_V1 // version 1 of the RCM D1 // https://github.com/RCMgames/RCM-Hardware-D1

/**
uncomment one of the following lines depending on which communication method you want to use
*/
#define RCM_COMM_METHOD RCM_COMM_EWD // use the normal communication method for RCM robots
// #define RCM_COMM_METHOD RCM_COMM_ROS // use the ROS communication method

#define WIFI_MODE_JOIN 0        //join existing network
#define WIFI_MODE_CREATE 1      //create new access point

#define _STRINGIZE(x) #x
#define STRINGIZE(x) _STRINGIZE(x)

#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "ElegantOTA.h"
//#include <string>

#include "rcm.h" //defines pins

// set up motors and anything else you need here
// See this page for how to set up servos and motors for each type of RCM board:
// https://github.com/RCMgames/useful-code/tree/main/boards
// See this page for information about how to set up a robot's drivetrain using the JMotor library
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain

String AP_NAME = STRINGIZE(WIFI_NAME);
String AP_PASSWORD = STRINGIZE(WIFI_PASSWORD);

JMotorDriverEsp32Servo servo1Driver = JMotorDriverEsp32Servo(port1);
JServoController sC = JServoController(servo1Driver, false, INFINITY, INFINITY, INFINITY );
float servo1Val = 0;


//const char* AP_NAME = "HAL"; const char* AP_PASSWORD = "booboo42";'

AsyncWebServer server(80);

unsigned long ota_progress_millis = 0;

void onOTAStart() {
    // Log when OTA has started
    Serial.println("OTA update started!");
    // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
    // Log every 1 second
    if (millis() - ota_progress_millis > 1000) {
        ota_progress_millis = millis();
        Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    }
}

void onOTAEnd(bool success) {
    // Log when OTA has finished
    if (success) {
        Serial.println("OTA update finished successfully!");
    } else {
        Serial.println("There was an error during OTA update!");
    }
    // <Add your own code here>
}

void Enabled()
{
    // code to run while enabled, put your main code here
    if (servo1Val > 180) servo1Val = -180;
    else servo1Val += 15;
    sC.setAngleImmediate(servo1Val);
    //servo1Driver.enable();
    
}

void Enable()
{
    // turn on outputs
    sC.enable();
    sC.setAngleImmediate(0);
}

void Disable()
{
    // turn off outputs
    sC.disable();

}

void PowerOn()
{
    Serial.println();
    delay(2000);
    // runs once on robot startup, set pin modes and use begin() if applicable here
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you shouldn't also call leftMotorController.run())
    sC.run();
    /*#ifdef WIFI_MODE
    Serial.printf("WIFI_MODE: %d\n", WIFI_MODE);
    #else
    Serial.printf("WIFI_MODE UNDEFINED");
    #endif*/
    //Serial.printf("WIFI NAME: %s\n", EWD::routerName);
    //Serial.printf("WIFI PW: %s\n", EWD::routerPassword);
    ElegantOTA.loop();
    delay(100);
}

#if RCM_COMM_METHOD == RCM_COMM_EWD
void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)

}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    EWD::sendFl(servo1Val);
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)

}

void configWifi()
{
    //esp_netif_init();
    #ifdef WIFI_MODE
    Serial.printf("WIFI_MODE: %d\n", WIFI_MODE);
    #if WIFI_MODE == WIFI_MODE_JOIN
    Serial.printf("JOINING WIFI NETWORK\n");
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = AP_NAME.c_str();
    EWD::routerPassword = AP_PASSWORD.c_str();
    EWD::routerPort = 25210;
    #elif WIFI_MODE == WIFI_MODE_CREATE
    Serial.printf("CREATING WIFI NETWORK\n");
    EWD::mode = EWD::Mode::createAP;
    EWD::APName = "rcm0";
    EWD::APPassword = "rcmPassword";
    EWD::APPort = 25210;
    AP_NAME = "rcm0";
    AP_PASSWORD = "rcmPassword";
    #else
    Serial.printf("WIFI_MODE UNDEFINED");
    #endif
    #endif

    Serial.printf("WIFI NAME: %s\n", EWD::routerName);
    Serial.printf("WIFI PW: %s\n", EWD::routerPassword);
}
void setupOTA()
{
    while (WiFi.status() != WL_CONNECTED) {
        delay(550);
        Serial.print(".");
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo. Number: 624");
    });
    ElegantOTA.begin(&server);    // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    server.begin();
    Serial.println("HTTP server started");
}
#elif RCM_COMM_METHOD == RCM_COMM_ROS ////////////// ignore everything below this line unless you're using ROS mode/////////////////////////////////////////////
void ROSWifiSettings()
{
    // SSID, password, IP, port (on a computer run: sudo docker run -it --rm --net=host microros/micro-ros-agent:iron udp4 --port 8888 )
    set_microros_wifi_transports("router", "password", "10.25.21.1", 8888); // doesn't complete until it connects to the wifi network
    nodeName = "rcm_robot";
    // numSubscribers = 10; //change max number of subscribers
}

#include <example_interfaces/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
// and lots of other message types are available (see file available_ros2_types)
// #include <geometry_msgs/msg/twist.h>

// declare publishers
declarePub(battery, std_msgs__msg__Float32);

// // declare subscribers and write callback functions
// declareSubAndCallback(cmd_vel, geometry_msgs__msg__Twist);
// velCmd.x = cmd_velMsg->linear.x;
// velCmd.y = cmd_velMsg->linear.y;
// velCmd.theta = cmd_velMsg->angular.z;
// } // end of callback

void ROSbegin()
{
    // create publishers
    createPublisher(battery, std_msgs__msg__Float32, "/rcm/battery");
    batteryMsg.data = 0;

    // add subscribers
    // addSub(cmd_vel, geometry_msgs__msg__Twist, "/cmd_vel");
}

void ROSrun()
{
    rosSpin(1);
    // you can add more publishers here
    batteryMsg.data = voltageComp.getSupplyVoltage();
    publish(battery);
}
#endif

#include "rcmutil.h"
