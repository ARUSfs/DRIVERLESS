#include <cstdint>
#include <cstring>
#include <canlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <nlohmann/json.hpp>
#include <mqtt/async_client.h>

// CAN IDs
#define CAN_ID_INVERTER_TX 0x181
#define CAN_ID_INVERTER_RX 0x201
#define CAN_ID_AS 0x182
#define CAN_ID_DASHBOARD 0x185
#define CAN_ID_BMS 0x186
#define CAN_ID_ACQUISITION 0x187
#define CAN_ID_POWERMGNT 0x188
#define CAN_ID_FL_1 0x310
#define CAN_ID_FL_2 0x311
#define CAN_ID_FR_1 0x330
#define CAN_ID_FR_2 0x331
#define CAN_ID_RL_1 0x350
#define CAN_ID_RL_2 0x351
#define CAN_ID_RR_1 0x370
#define CAN_ID_RR_2 0x371

// Global Byte Positions
#define ID_HEARTBEAT 0x00
#define BYTE_LSB_1 1
#define BYTE_MSB_1 2
#define BYTE_LSB_2 3
#define BYTE_MSB_2 4
#define BYTE_LSB_3 5
#define BYTE_MSB_3 6

// INVERTER TX
#define ID_TEMP_MOTOR 0x49
#define ID_TEMP_IGBT 0x4A
#define ID_CURRENT_LIMIT 0xC4
#define ID_STATUS_INVERTER 0x40
#define ID_ERROR_WARNING_BITMAP 0x8F
#define ID_ACTUAL_CURRENT 0x20
#define ID_COMMAND_CURRENT 0x26
#define ID_SPEED_ACTUAL 0x30
#define ID_FILTERED_ACTL_CURRENT 0x5F
#define ID_VOLTAGE_DC_BUS 0xEB

// INVERTER RX
#define ID_TORQUE_COMMAND 0x90

// DASHBOARD
#define ID_MISSION 0x01
#define ID_BUZZER 0x02

// AS
#define ID_STATUS_AS 0x01
#define ID_FAULT 0x02
#define ID_APPS 0x03
#define ID_BRAKE_PRESSURE 0x04
#define ID_PNEUMATIC_PRESSURE 0x05
#define ID_STATUS_VALVE 0x06

// BMS
#define ID_RELAYS 0x00
#define ID_VOLTAGES 0x01
#define ID_CURRENT 0x02
#define ID_TEMP 0x03

// ADQUISITION
#define ID_LOWVOLTAGE 0x00
#define ID_STEERING 0x01
#define ID_FRONT_EXT 0x02
#define ID_REAR_EXT 0x03
#define ID_FRONT_WHEELSPEED 0x04
#define ID_REAR_WHEELSPEED 0x05

// POWER MANAGEMENT
#define ID_STATUSPM 0x00
#define ID_TEMPREFRI 0x01

struct INVERTER
{
    unsigned int statusInverter;
    unsigned int actualCurrent;
    unsigned int commandCurrent;
    unsigned int errorBitmap;
    unsigned int warningBitmap;
    int torqueCommand;
    int currentLimit;
    int tempMotor;
    int tempIGBT;
    int16_t speedActual = 51;
    uint16_t filteredActualCurrent;
    uint16_t voltageDCBus;

} inverterCAN;

struct AS
{
    bool heartbeat;
    bool autonomous;
    int statusAS;
    int fault;
    int apps;
    int frontBrakePressure;
    int rearBrakePressure;
    int pneumaticPressure1;
    int pneumaticPressure2;
    int statusValve;
} asCAN;

struct PC
{
    bool heartbeat;
    int cpuTemp;
} pcCAN;

struct DASHBOARD
{
    bool heartbeat;
    int mission;
    int buzzer;
} dashboardCAN;
struct BMS
{
    int statusBMS;
    int totalVoltage;
    int tempCells;
    int globalMinVoltage;
    int lastMinVoltage;
    int currrent = 200;
} bmsCAN;

struct ACQUISITION
{
    int lowVoltage;
    int steering;
    int extfl;
    int extfr;
    int extrl;
    int extrr;
    int wsfl;
    int wsfr;
    int wsrl;
    int wsrr;
} acquisitionCAN;

struct POWERMGNT_CAN
{
    int statusPM;
    int tempRefri;
} powerMgntCAN;

struct TIRE_TEMP
{
    double FL1;
    double FL2;
    double FL3;
    double FL4;
    double FR1;
    double FR2;
    double FR3;
    double FR4;
    double RL1;
    double RL2;
    double RL3;
    double RL4;
    double RR1;
    double RR2;
    double RR3;
    double RR4;
} tireTempCAN;

canStatus stat;
canHandle hnd;

const std::string SERVER_ADDRESS("tcp://arus.westeurope.cloudapp.azure.com:1883");
const std::string CLIENT_ID("carrinho");
const std::string TOPIC("telemetry/ARTE");

mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);


void initCan()
{
    // Inicializamos la libreria
    canInitializeLibrary();

    // Openeamos el canalw
    hnd = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0)
    {
        printf("canOpenChannel failed, status=%d\n", hnd);
    }

    // Oneamos el bus
    stat = canBusOn(hnd);
    if (stat != canOK)
    {
        printf("canBusOn failed, status=%d\n", stat);
    }
}

void closeCan()
{
    stat = canBusOff(hnd);
    if (stat != canOK)
    {
        printf("canBusOff failed, status=%d\n", stat);
    }

    stat = canClose(hnd);
    if (stat != canOK)
    {
        printf("canClose failed, status=%d\n", stat);
    }

    canUnloadLibrary();
}

void processCANMessage(unsigned long id, unsigned char *buffer)
{

    switch (id)
    {

    case CAN_ID_INVERTER_TX:
        switch (buffer[0])
        {
        case ID_TEMP_MOTOR:
            inverterCAN.tempMotor = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        case ID_TEMP_IGBT:
            inverterCAN.tempIGBT = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        case ID_ERROR_WARNING_BITMAP:
            inverterCAN.errorBitmap = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            inverterCAN.warningBitmap = (int)(buffer[BYTE_MSB_2] << 8 | buffer[BYTE_LSB_2]);
            break;

        case ID_STATUS_INVERTER:
            inverterCAN.statusInverter = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        case ID_CURRENT_LIMIT:
            inverterCAN.currentLimit = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        case ID_ACTUAL_CURRENT:
            inverterCAN.actualCurrent = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        case ID_COMMAND_CURRENT:
            inverterCAN.commandCurrent = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        case ID_SPEED_ACTUAL:
            inverterCAN.speedActual = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        case ID_FILTERED_ACTL_CURRENT:
            inverterCAN.filteredActualCurrent = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        case ID_VOLTAGE_DC_BUS:
            inverterCAN.voltageDCBus = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        default:
            break;
        }
        break;

    case CAN_ID_INVERTER_RX:
        switch (buffer[0])
        {
        case ID_TORQUE_COMMAND:
            inverterCAN.torqueCommand = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        default:
            break;
        }
        break;

    case CAN_ID_AS:
        switch (buffer[0])
        {
        case ID_HEARTBEAT:
            asCAN.heartbeat = buffer[1];
            break;
        case ID_STATUS_AS:
            asCAN.autonomous = buffer[1];
            asCAN.statusAS = buffer[2];
            break;
        case ID_FAULT:
            asCAN.fault = buffer[1];
            break;
        case ID_APPS:
            asCAN.apps = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;
        case ID_BRAKE_PRESSURE:
            asCAN.frontBrakePressure = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            asCAN.rearBrakePressure = (int)(buffer[BYTE_MSB_2] << 8 | buffer[BYTE_LSB_2]);
            break;
        case ID_PNEUMATIC_PRESSURE:
            asCAN.pneumaticPressure1 = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            asCAN.pneumaticPressure2 = (int)(buffer[BYTE_MSB_2] << 8 | buffer[BYTE_LSB_2]);
            break;
        case ID_STATUS_VALVE:
            asCAN.statusValve = buffer[1];
            break;
        default:
            break;
        }
        break;

    case CAN_ID_DASHBOARD:
        switch (buffer[0])
        {
        case ID_HEARTBEAT:
            dashboardCAN.heartbeat = buffer[1];
            break;
        case ID_MISSION:
            dashboardCAN.mission = buffer[1];
            break;
        case ID_BUZZER:
            dashboardCAN.buzzer = buffer[1];
            break;
        default:
            break;
        }
        break;

    case CAN_ID_BMS:
        switch (buffer[0])
        {
        case ID_RELAYS:
            bmsCAN.statusBMS = buffer[1];
            break;

        case ID_VOLTAGES:
            bmsCAN.totalVoltage = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            bmsCAN.globalMinVoltage = buffer[3];
            bmsCAN.lastMinVoltage = buffer[4];
            break;

        case ID_CURRENT:
            bmsCAN.currrent = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        case ID_TEMP:
            bmsCAN.tempCells = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;

        default:
            break;
        }
        break;

    case CAN_ID_ACQUISITION:

        switch (buffer[0])
        {
        case ID_LOWVOLTAGE:
            acquisitionCAN.lowVoltage = buffer[1];
            break;
        case ID_STEERING:
            acquisitionCAN.steering = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;
        case ID_FRONT_EXT:
            acquisitionCAN.extfl = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            acquisitionCAN.extfr = (int)(buffer[BYTE_MSB_2] << 8 | buffer[BYTE_LSB_2]);
            break;
        case ID_REAR_EXT:
            acquisitionCAN.extrl = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            acquisitionCAN.extrr = (int)(buffer[BYTE_MSB_2] << 8 | buffer[BYTE_LSB_2]);
            break;
        case ID_FRONT_WHEELSPEED:
            acquisitionCAN.wsfl = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            acquisitionCAN.wsfr = (int)(buffer[BYTE_MSB_2] << 8 | buffer[BYTE_LSB_2]);
            break;
        case ID_REAR_WHEELSPEED:
            acquisitionCAN.wsrl = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            acquisitionCAN.wsrr = (int)(buffer[BYTE_MSB_2] << 8 | buffer[BYTE_LSB_2]);
            break;
        default:
            break;
        }
        break;

    case CAN_ID_POWERMGNT:
        switch (buffer[0])
        {
        case ID_STATUSPM:
            powerMgntCAN.statusPM = buffer[1];
            break;
        case ID_TEMPREFRI:
            powerMgntCAN.tempRefri = (int)(buffer[BYTE_MSB_1] << 8 | buffer[BYTE_LSB_1]);
            break;
        default:
            break;
        }
        break;

    case CAN_ID_FL_1:
        tireTempCAN.FL1 = (((buffer[0] << 8 | buffer[1]) + (buffer[2] << 8 | buffer[3])) / 2) * 10 - 2000;
        tireTempCAN.FL2 = (((buffer[4] << 8 | buffer[5]) + (buffer[6] << 8 | buffer[7])) / 2) * 10 - 2000;
        break;

    case CAN_ID_FL_2:
        tireTempCAN.FL3 = (((buffer[0] << 8 | buffer[1]) + (buffer[2] << 8 | buffer[3])) / 2) * 10 - 2000;
        tireTempCAN.FL4 = (((buffer[4] << 8 | buffer[5]) + (buffer[6] << 8 | buffer[7])) / 2) * 10 - 2000;
        break;

    case CAN_ID_FR_1:
        tireTempCAN.FR1 = (((buffer[0] << 8 | buffer[1]) + (buffer[2] << 8 | buffer[3])) / 2) * 10 - 2000;
        tireTempCAN.FR2 = (((buffer[4] << 8 | buffer[5]) + (buffer[6] << 8 | buffer[7])) / 2) * 10 - 2000;
        break;

    case CAN_ID_FR_2:
        tireTempCAN.FR3 = (((buffer[0] << 8 | buffer[1]) + (buffer[2] << 8 | buffer[3])) / 2) * 10 - 2000;
        tireTempCAN.FR4 = (((buffer[4] << 8 | buffer[5]) + (buffer[6] << 8 | buffer[7])) / 2) * 10 - 2000;
        break;

    case CAN_ID_RL_1:
        tireTempCAN.RL1 = (((buffer[0] << 8 | buffer[1]) + (buffer[2] << 8 | buffer[3])) / 2) * 10 - 2000;
        tireTempCAN.RL2 = (((buffer[4] << 8 | buffer[5]) + (buffer[6] << 8 | buffer[7])) / 2) * 10 - 2000;
        break;

    case CAN_ID_RL_2:
        tireTempCAN.RL3 = (((buffer[0] << 8 | buffer[1]) + (buffer[2] << 8 | buffer[3])) / 2) * 10 - 2000;
        tireTempCAN.RL4 = (((buffer[4] << 8 | buffer[5]) + (buffer[6] << 8 | buffer[7])) / 2) * 10 - 2000;
        break;

    case CAN_ID_RR_1:
        tireTempCAN.RR1 = (((buffer[0] << 8 | buffer[1]) + (buffer[2] << 8 | buffer[3])) / 2) * 10 - 2000;
        tireTempCAN.RR2 = (((buffer[4] << 8 | buffer[5]) + (buffer[6] << 8 | buffer[7])) / 2) * 10 - 2000;
        break;

    case CAN_ID_RR_2:
        tireTempCAN.RR3 = (((buffer[0] << 8 | buffer[1]) + (buffer[2] << 8 | buffer[3])) / 2) * 10 - 2000;
        tireTempCAN.RR4 = (((buffer[4] << 8 | buffer[5]) + (buffer[6] << 8 | buffer[7])) / 2) * 10 - 2000;
        break;

    default:
        break;
    }
}

nlohmann::json createCANJson()

{
    nlohmann::json data;
    data["time"] = 0; // TODO Comprobar

    data["speedActual"] = inverterCAN.speedActual;
    data["tempMotor"] = inverterCAN.tempMotor;
    data["tempIGBT"] = inverterCAN.tempIGBT;
    data["statusInverter"] = inverterCAN.statusInverter;
    data["errorBitmap"] = inverterCAN.errorBitmap;
    data["warningBitmap"] = inverterCAN.warningBitmap;
    data["currentLimit"] = inverterCAN.currentLimit;
    data["voltageDCBus"] = inverterCAN.voltageDCBus;

    data["torqueCommand"] = inverterCAN.torqueCommand;

    data["heartbeatAS"] = asCAN.heartbeat;
    data["autonomous"] = asCAN.autonomous;
    data["statusAS"] = asCAN.statusAS;
    data["fault"] = asCAN.fault;
    data["apps"] = asCAN.apps;
    data["frontBrakePressure"] = asCAN.frontBrakePressure;
    data["rearBrakePressure"] = asCAN.rearBrakePressure;
    data["pneumaticPressure1"] = asCAN.pneumaticPressure1;
    data["pneumaticPressure2"] = asCAN.pneumaticPressure2;
    data["statusValve"] = asCAN.statusValve;

    data["heartbeatPC"] = pcCAN.heartbeat;
    data["cpuTemp"] = pcCAN.cpuTemp;

    data["heartbeatDashboard"] = dashboardCAN.heartbeat;
    data["mission"] = dashboardCAN.mission;
    data["buzzer"] = dashboardCAN.buzzer;

    data["current"] = bmsCAN.currrent;
    data["totalVoltage"] = bmsCAN.totalVoltage;
    data["globalMinVoltage"] = bmsCAN.globalMinVoltage;
    data["lastMinVoltage"] = bmsCAN.lastMinVoltage;
    data["tempCells"] = bmsCAN.tempCells;
    data["statusBMS"] = bmsCAN.statusBMS;

    data["lowVoltage"] = acquisitionCAN.lowVoltage;
    data["steering"] = acquisitionCAN.steering;
    data["extfl"] = acquisitionCAN.extfl;
    data["extfr"] = acquisitionCAN.extfr;
    data["extrl"] = acquisitionCAN.extrl;
    data["extrr"] = acquisitionCAN.extrr;
    data["wsfl"] = acquisitionCAN.wsfl;
    data["wsfr"] = acquisitionCAN.wsfr;
    data["wsrl"] = acquisitionCAN.wsrl;
    data["wsrr"] = acquisitionCAN.wsrr;

    data["statusPM"] = powerMgntCAN.statusPM;
    data["tempRefri"] = powerMgntCAN.tempRefri;

    data["tempFL1"] = tireTempCAN.FL1;
    data["tempFL2"] = tireTempCAN.FL2;
    data["tempFL3"] = tireTempCAN.FL3;
    data["tempFL4"] = tireTempCAN.FL4;

    data["tempFR1"] = tireTempCAN.FR1;
    data["tempFR2"] = tireTempCAN.FR2;
    data["tempFR3"] = tireTempCAN.FR3;
    data["tempFR4"] = tireTempCAN.FR4;

    data["tempRL1"] = tireTempCAN.RL1;
    data["tempRL2"] = tireTempCAN.RL2;
    data["tempRL3"] = tireTempCAN.RL3;
    data["tempRL4"] = tireTempCAN.RL4;

    data["tempRR1"] = tireTempCAN.RR1;
    data["tempRR2"] = tireTempCAN.RR2;
    data["tempRR3"] = tireTempCAN.RR3;
    data["tempRR4"] = tireTempCAN.RR4;

    return data;
}

void initMQTT()
{
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    try
    {
        std::cout << "Conectando al MQTT broker en " << SERVER_ADDRESS << "..." << std::endl;
        client.connect(connOpts)->wait();
        std::cout << "Conectado." << std::endl;
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << "Error al conectar: " << exc.what() << std::endl;
        exit(1);
    }
}

void publishCANData(const nlohmann::json &jsonData)
{
    std::string payload = jsonData.dump();
    mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, payload);
    pubmsg->set_qos(1);
    client.publish(pubmsg)->wait_for(20);
}


int main(int argc, char **argv)
{
    initCan();
    initMQTT();

    while (true)
    {
        long id;
        uint8_t msg[8];
        unsigned int dlc;
        unsigned int flag;
        unsigned long time;

        stat = canRead(hnd, &id, &msg, &dlc, &flag, &time);

        if (stat == canOK)
        {
            processCANMessage(id, msg);
            nlohmann::json jsonData = createCANJson();
            publishCANData(jsonData);
        }

        //loop_rate.sleep()
    }

    closeCan();

    return 0;
}
