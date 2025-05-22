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
        static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> flexcan_bus;
};
  
    
#endif