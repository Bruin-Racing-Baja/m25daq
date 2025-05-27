#ifndef CANBUS_H
#define CANBUS_H
#include <FlexCAN_T4.h>
#include <types.h>
class ODrive;
class Can_Bus
{
    public:
        Can_Bus();
        
        static u8 send_command(u32 func_id, u32 node_id, bool remote, u8 buf[]);
        static u8 send_command(CAN_message_t msg);
        static void can_parse(const CAN_message_t &msg);
        static void setup();
    
    private:
        static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
};
constexpr u8 ODRIVE_NODE_ID = 0x3;
constexpr u8 RASP_NODE_ID = 0x4;     
constexpr u32 FLEXCAN_BAUD_RATE = 1000000;  
constexpr u32 FLEXCAN_MAX_MAILBOX = 63;   
    
#endif