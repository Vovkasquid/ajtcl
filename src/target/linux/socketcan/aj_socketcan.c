#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <stdint.h>

#include "aj_socketcan.h"

#define MAX_SEQUENCES (3)
#define MAX_PAYLOAD (7)
#define MAX_CHUNKS (32)

//#define SCAN_DEBUG

#define logError(s, ...) printf("E %s:%d " s, __FILE__, __LINE__, ##__VA_ARGS__)
#define logInfo(s, ...) printf("I %s:%d " s, __FILE__, __LINE__, ##__VA_ARGS__)

typedef struct {
    int finished;
    int len;
    int pos; // end of upstreamed buffer part
    uint8_t final_chunk;
#ifdef SCAN_DEBUG
    uint8_t pending_chunk;
#endif
    uint8_t chunk_done[MAX_CHUNKS];
    uint8_t buf[MAX_PAYLOAD * MAX_CHUNKS]; // must be last field!
} SEQUENCE;

int active_sequence = -1;

static SEQUENCE sequences[MAX_SEQUENCES];

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif


static void clean_sequence(SEQUENCE *seq) {
    memset(seq, 0, ((uint8_t *)(&seq->buf)) - (uint8_t *)seq);
}

#ifdef SCAN_DEBUG
static void print_sequence(uint8_t seq_id) {
    int i;
    printf("seq %d: finished=%d len=%d pos=%d last=%d ",
        seq_id, sequences[seq_id].finished, sequences[seq_id].len,  sequences[seq_id].pos, sequences[seq_id].final_chunk);
    for (i = 0; i < MAX_CHUNKS; i++) {
        putchar(sequences[seq_id].chunk_done[i] ? '+' : '.');
    }
    printf("\n");
}
#endif

/**
 * @return binded socket fd or -1
 */
int AJ_SocketCAN_Open(const char *ifname) {
    int fd = -1;
    struct sockaddr_can addr;
    struct ifreq ifr;
    int i;

    if((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(fd, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex; 

    if (ifr.ifr_ifindex < 0) {
        printf("Error: ifr_index < 0\n");
        return -1;
    }

    if(bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -1;
    }

    for (i = 0; i < MAX_SEQUENCES; i++) {
        clean_sequence(sequences + i);
    }
    
    return fd;
}

static int process_frame(struct can_frame *frame) {
    uint8_t seq_id = frame->data[0] & 3;
    uint8_t chunk_num = (frame->data[0] >> 2) & 0x1F;
    uint8_t is_last = (frame->data[0] & 0x80);
    SEQUENCE *seq = sequences + seq_id;

//    printf("got frame seq %d, chunk %d sz %d, data[0] = 0x%X\n", seq_id, chunk_num, frame->can_dlc, frame->data[0]);

    if (is_last) {
        if (seq->final_chunk > 0) {
            logError("last chunk num is already received %d, seq = %d, new = %d\n",
                    seq->final_chunk, seq_id, chunk_num);
            return -1;
        }
        seq->final_chunk = chunk_num;
        seq->len = chunk_num * MAX_PAYLOAD + (frame->can_dlc - 1); // finalize len
    } else {
        if (frame->can_dlc != 8) {
            logError("wrong payload length in non-final chunk %d, seq = %d, dlc = %d\n",
                    chunk_num, seq_id, frame->can_dlc);
            return -1;
        }        
    }

    if (seq->chunk_done[chunk_num]) {
        logError("chunk is already received %d, seq = %d\n",
                chunk_num, seq_id);
        return -1;
    } else {
#ifdef SCAN_DEBUG
        if (seq->pending_chunk != chunk_num) {
            logInfo("seq %d: chunk %d is out-of-order, pending %d\n",
               seq_id, chunk_num, seq->pending_chunk);
            seq->pending_chunk = chunk_num + 1;
        } else {
            seq->pending_chunk++;
        }
#endif
        seq->chunk_done[chunk_num] = 1;
    }

    memcpy(seq->buf + (chunk_num * MAX_PAYLOAD), frame->data + 1, frame->can_dlc - 1);

    return 0;
}

static SEQUENCE *have_finished(void) {
    if (active_sequence == -1) {
        return NULL;
    } else {
        int i;
        SEQUENCE *active = sequences + active_sequence;

        if (active->finished) {
            return active;
        }

        // check if sequence is ready

        if (active->len == 0) {
            return NULL; // did not get last chunk
        }
        
        // check if all chunks up to last are received
        for (i = 0; i <= active->final_chunk; i++) {
            if (!active->chunk_done[i]) {
                return NULL;
            }
        }

        // sanity check if no chunk above last was received
        for (i = active->final_chunk + 1; i < MAX_CHUNKS; i++) {
            if (active->chunk_done[i]) {
                logError("FATAL: found chunk %i above last chunk %d in seq %d\n", i, active->final_chunk, active_sequence);
                active_sequence = ((active_sequence + 1) % MAX_SEQUENCES);
                clean_sequence(active);
                return NULL;
            }
        }

        // mark as finished and return
        logInfo("recvd sequence %d bytes\n", active->len);
        active->finished = 1;
        return active;
    }   
}

static int check_and_close(SEQUENCE *seq) {
    if (seq->pos == seq->len) {
        active_sequence = ((active_sequence + 1) % MAX_SEQUENCES);
        clean_sequence(seq);
        return 1;
    }
    return 0;
} 

int AJ_SocketCAN_Read(int fd, uint8_t *buffer, int len, canid_t can_id) {
    int overall = 0;
    struct can_frame frame;

    while (len) {
        uint8_t seq_id;
        SEQUENCE *finished = have_finished();

        // check if we already have sonthing to upstream
        if (finished) {
            int tocopy = MIN(len, (finished->len - finished->pos)); 
            memcpy(buffer, finished->buf + finished->pos, tocopy);
            finished->pos += tocopy;
            len -= tocopy;
            buffer += tocopy;
            overall += tocopy;
            if (check_and_close(finished)) {
                // sequence finished, break
                break;
            }
        }

        // read next frame with correct can_id
        do {
            if (read(fd, &frame, sizeof(struct can_frame)) < 0) {
                logError("read < 0, '%s'\n", strerror(errno));
                return -1;
            }
        } while (frame.can_id != can_id);

        seq_id = frame.data[0] & 3;

        if (active_sequence == -1) {
            active_sequence = seq_id;
        }

        if (((seq_id + 1) % MAX_SEQUENCES) == active_sequence) {
            logError("got frame from next-next sequence (%d), seems active sequence %d is stalled, drop it.\n",
                seq_id, active_sequence);
            //print_sequence(active_sequence);
            clean_sequence(sequences + active_sequence);
            active_sequence = ((active_sequence + 1) % MAX_SEQUENCES);
            //print_sequence(active_sequence);
        }

        if (process_frame(&frame) == -1) {
            logError("Drop sequence %d, sorry.\n", seq_id);
            clean_sequence(sequences + seq_id);
            if (seq_id == active_sequence) {
                active_sequence = ((active_sequence + 1) % MAX_SEQUENCES);
            }
        }
    }
    return overall; 
}

int AJ_SocketCAN_Write(int fd, uint8_t *buffer, int len, canid_t can_id) {
    static uint8_t write_sequence = -1;
    struct can_frame frame;
    int offset = 0;
    uint8_t chunk;

    if (len <= 0) {
        logError("wrong len %d\n", len);
        return -1;
    }

    len = MIN(len, MAX_PAYLOAD * MAX_CHUNKS);
    frame.can_id = can_id;
    
    write_sequence = (write_sequence + 1) % MAX_SEQUENCES;

    chunk = 0;
    while (len) {
        int nbytes;

        frame.can_dlc = 1 + MIN(len, MAX_PAYLOAD);
        frame.data[0] = write_sequence | (chunk << 2);

        memcpy(frame.data + 1, buffer + offset, frame.can_dlc - 1);
        offset += frame.can_dlc - 1;
        len -= frame.can_dlc - 1;

        if (len == 0) {
            frame.data[0] |= 0x80; // last chunk
        }   

        nbytes = write(fd, &frame, sizeof(struct can_frame));
        if (nbytes != sizeof(struct can_frame)) {
            logError("write failed %d\n", nbytes);
            return -1;
        }
        
        chunk++;
    }

    return offset; 
}

