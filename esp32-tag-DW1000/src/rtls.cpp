/*

For ESP32 UWB or ESP32 UWB Pro

*/

#include <SPI.h>
#include <DW1000Ranging.h>
#include <WiFi.h>
#include "link.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34

const uint16_t anchor_1_addr = 0x7D00;
const uint16_t anchor_2_addr = 0x7D00;

struct MyLink *uwb_data;
int index_num = 0;
long runtime = 0;

void getPositioin(struct MyLink *p);

void newRange();

void newDevice(DW1000Device *device);

void inactiveDevice(DW1000Device *device);

void RTLS_Task(void *parameter)
{
    delay(1000);

    //init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, DW_CS, PIN_IRQ);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    //we start the module as a tag
    DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_SHORTDATA_FAST_ACCURACY);

    uwb_data = init_link();
    
    while (1)
    {
        DW1000Ranging.loop();
        if ((millis() - runtime) > 100)
        {
            getPositioin(uwb_data);
            runtime = millis(); 
        }
        vTaskDelay(1);
    }

}

void getPositioin(struct MyLink *p)
{
    struct MyLink *temp = p;

    while (temp->next != NULL)
    {
        Serial.print("short: ");
        Serial.print(temp->anchor_addr, HEX);
        Serial.print(" range: ");
        Serial.print(temp->range[0]);
        Serial.print(" m");
        Serial.print(" dbm: ");
        Serial.println(temp->dbm);
        temp = temp->next;
    }
}

void newRange()
{
    char c[30];

    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
    fresh_link(uwb_data, DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange(), DW1000Ranging.getDistantDevice()->getRXPower());
}

void newDevice(DW1000Device *device)
{
    Serial.print("ranging init; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);

    add_link(uwb_data, device->getShortAddress());
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);

    delete_link(uwb_data, device->getShortAddress());
}
