#include "constants.h"
#include <FlexCAN_T4.h>
#include <can_bus.h>
#include <string>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>  Can_Bus::flexcan_bus;

Can_Bus::Can_Bus()
{
  
}
void Can_Bus::setup()
{
  flexcan_bus.begin();
  flexcan_bus.setBaudRate(1000000);
  flexcan_bus.setMaxMB(FLEXCAN_MAX_MAILBOX);
  flexcan_bus.enableFIFO();
  flexcan_bus.enableFIFOInterrupt();
  flexcan_bus.onReceive(can_parse);
}

/**
 * @brief Send a command
 * @param cmd_id ODrive CAN command ID number
 * @param remote RTR bit for CAN message (typically 0 for setters and 1 for
 * requesters)
 * @param buf 8-wide array of command data bytes
 * @return Status code of command send
 */
u8 Can_Bus::send_command(u32 func_id, u32 node_id, bool remote, u8 buf[]) {
  // TODO: Fix error mwessages
  CAN_message_t msg;
  if (func_id < 0x00 || 0x1f < func_id) {
    return -1;
  }
  
  buf[0] = static_cast<u8>(func_id);
  func_id = 0x1f;
  
  msg.len = 8;
  memcpy(&msg.buf, buf, 8);
  msg.flags.remote = remote;

  int write_code = flexcan_bus.write(msg);
 
  return write_code;
}
u8 Can_Bus::send_command(CAN_message_t msg) {
  return flexcan_bus.write(msg);
}

void Can_Bus::can_parse(const CAN_message_t &msg)
{
  u32 parsed_node_id = (msg.id >> 5) & 0x3F;
  u32 cmd_id = msg.id & 0x1F;
  /*
  switch (parsed_node_id) {
    case ODRIVE_ECVT_NODE_ID: //after figuring out cmd id change this later
      odrive_ecvt->parse_message(msg);
      break;
    case ODRIVE_ECENT_NODE_ID: //after figuring out cmd id change this later
      odrive_ecent->parse_message(msg);
      break;
    case CONTROLS_PCB_NODE_ID:
    ;

  }
    */
}