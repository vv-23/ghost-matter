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

#define BNO080_ADDRESS 0x4A

#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "ElegantOTA.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "Wire.h"
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
bool IMUStatus = false;

//const char* AP_NAME = "HAL"; const char* AP_PASSWORD = "booboo42";'

AsyncWebServer server(80);

unsigned long ota_progress_millis = 0;
unsigned long t_start,t_stop;
BNO08x myIMU;

struct Quarternion
{
    float quatI;
    float quatJ;
    float quatK;
    float quatReal;
    float quatRadianAccuracy;
    void print(byte decimals = 2)
    {
        Serial.print(quatI, decimals);
        Serial.print(F(","));
        Serial.print(quatJ, decimals);
        Serial.print(F(","));
        Serial.print(quatK, decimals);
        Serial.print(F(","));
        Serial.print(quatReal, decimals);
        Serial.print(F(","));
        Serial.print(quatRadianAccuracy, decimals);
        Serial.print(F("\n"));
    }
};
struct RPY
{
    float roll;
    float pitch;
    float yaw;
    void print(byte decimals = 2)
    {
        Serial.print(roll, decimals);
        Serial.print(F(","));
        Serial.print(pitch, decimals);
        Serial.print(F(","));
        Serial.print(yaw, decimals);
        Serial.print(F("\n"));
    }
};

static Quarternion robotQuart;
static RPY robotRPY;

float radsToDeg(float rads)
{
    return rads * 180.0 / PI;
}

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

void scan(){
    byte error, address;
    int nDevices;
    Serial.println("Scanning...");
    nDevices = 0;
    t_start = millis();
    for(address = 1; address < 127; address++ ){
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    if (error == 0){
        Serial.print("I2C device found at address 0x");
        if (address<16) 
        Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
        nDevices++;
    }else if (error==4){
        Serial.print("Unknow error at address 0x");
        if (address<16) 
        Serial.print("0");
        Serial.println(address,HEX);
    }    
    }
    if (nDevices == 0)
    Serial.println("No I2C devices found");
    else
    Serial.println("done");
    t_stop = millis();
    Serial.print("Time scanning: "); Serial.print((t_stop - t_start) / 1000.00); Serial.println(" sec.");
}

void i2cSetup()
{
    delay(100); //  Wait for BNO to boot
    //Start i2c and BNO080
    Wire1.flush();   // Reset I2C
    Wire1.begin(SDA1, SCL1);
}

bool imuSetReports()
{
    Serial.println("Setting desired reports");
    if (myIMU.enableRotationVector() == true)
    {
        Serial.println(F("Rotation vector enabled"));
        Serial.println(F("Output in form roll, pitch, yaw"));
        return true;
    }
    else
    {
        Serial.println("Could not enable rotation vector");
        return false;
    }
}

bool imuSetup(byte imuAddress)
{
    boolean imuStatus = myIMU.begin(BNO080_ADDRESS, Wire1);
    //Wire1.setClockStretchLimit(4000);

    if (imuStatus == false)
    {
        Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        return false;
        //while (1);
    }
    else
    {
        Wire.setClock(400000);
        return imuSetReports();
    }
}

bool getRobotOrientation(RPY& robotRPY, BNO08x& imu)
{
    if (myIMU.wasReset())
    {
        Serial.print("sensor was reset ");
        if (!imuSetReports())
        {
            return false;
        }
    }
    bool dataAvailable = imu.getSensorEvent();
    if (dataAvailable)
    {
        robotRPY.pitch = radsToDeg(imu.getPitch());
        robotRPY.roll = radsToDeg(imu.getRoll());
        robotRPY.yaw = radsToDeg(imu.getYaw());
        IMUStatus = true;
    }
    else
    {
        IMUStatus = false;
    }
    return dataAvailable;
}

void Enabled()
{
    // code to run while enabled, put your main code here
    if (getRobotOrientation(robotRPY, myIMU))
    {
        robotRPY.print(3);
    };
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
    delay(3000);
    i2cSetup();
    imuSetup(BNO080_ADDRESS);
    //scan();
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
    delay(10);
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
    //EWD::sendFl(servo1Val);
    EWD::sendFl(robotRPY.roll);
    EWD::sendFl(robotRPY.pitch);
    EWD::sendFl(robotRPY.yaw);
    //EWD::sendBl(IMUStatus);
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
    #if WIFI_MODE == WIFI_MODE_JOIN
    while (WiFi.status() != WL_CONNECTED) {
        delay(550);
        Serial.print(".");
    }
    #endif
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
