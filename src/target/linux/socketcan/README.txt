SLAP/CAN Transport for AllJoyn.
===============================

The implementation uses SLAP protocol which is also used to communicate by Serial (UART) port.

src/target/linux/socketcan/aj_socketcan.* and src/target/linux/socketcan/aj_target_serio.c
implement underlying layer for SLAP Protocol and expose following API:

int AJ_SocketCAN_Open(const char *ifname)
 - This opens and binds a CAN socket, returns result of socket(2) call.

int AJ_SocketCAN_Read(int fd, uint8_t *buffer, int len, canid_t can_id)
 - Blocking read from the CAN socket ('fd'). The 'len' parameter must be
the size of pre-allocated data 'buffer'. The 'can_id' parameter specifies
CAN address (11-bits) of the peer to receive data from.
 - Returns actual bytes read or -1 on error.

int AJ_SocketCAN_Write(int fd, uint8_t *buffer, int len, canid_t can_id)
 - Blocking write to CAN socket ('fd'). The can_id parameter specifies
CAN address of this device. This API blocks until all 'len' bytes will be
written. Returns number of bytes written or -1 on error.

SocketCAN protocol sends and receives data in for of sequences of CAN frames.
Each CAN frame in the sequence has CAN address passed in AJ_SocketCAN_Write
and up to 8 bytes of data:
 data[0] - data descriptor
 data[1-?] - SLAP data payload (up to 7 bytes, set in field can_dlc of the can_frame).

Data descriptor (data[0]) structure:
 bits 0-1: sequence number. rounds from 0 to 2. Specifies the current sequence ID being sent.
 bits 2-6: CAN packet number in the sequence (up to 32 packets)
 bit  7: Marks the last packet in the sequence (1 - for last frame, 0 - for others).

The protocol splits data to be sent onto sequences of CAN frames, up to 32 frames 
with up to 7 bytes payload in each frame. The receiver is capable to receive unordered frames
of two sequences at the same time. When all frames in the pending sequence are received the
AJ_SocketCAN_Read() moves bytes to the buffer, increments number of the pending sequence 
and returns. The aj_target_serial reading thread (runRx) has to call AJ_SocketCAN_Read() continuously 
to receive all frames from all sequences being sent.

The AJ_SocketCAN_Read() drops all data from the pending sequence if a frame from next-to-next sequence 
comes before pending sequence is finished as this is treated as some frames are lost and won't come.

All data checksuming is performed on SLAP transport layer.

