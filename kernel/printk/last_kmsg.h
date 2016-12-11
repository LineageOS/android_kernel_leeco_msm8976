#ifndef __LAST_KMSG_HEAD__
#define __LAST_KMSG_HEAD__

/*================================================
  **  definition for last_debug module
  **  The 2 files should be nearly the same:
  **  letv_last_debug.h----XBL
  **  last_kmsg.h ---KERNEL
  **================================================
  */

typedef u8 uint8;
typedef u16 uint16;
typedef u32 uint32;
typedef u64 uint64;

#define BUILD_TIME_LEN (256)
#define CRC16_START_VAL (0x55aa)
#define CRC16_TO_64BIT_ZERO (0x28f)
#define CRC16_TO_32BIT_ZERO (0xc831)

/* the UFS paritition will be divided into several parts.
 * last_kmsg is the first.
 */
#define SEC_GAP (0x1000)
#define LAST_DBG_INFO_OFFSET (0x0)
#define LAST_DBG_INFO_SIZE (0x100000)
#define PERSIST_LOG_HEADER_OFFSET (LAST_DBG_INFO_OFFSET+LAST_DBG_INFO_SIZE+SEC_GAP)
#define PERSIST_LOG_HEADER_SIZE (0x1000)
#define PERSIST_LOG_OFFSET (PERSIST_LOG_HEADER_OFFSET+PERSIST_LOG_HEADER_SIZE+SEC_GAP)
#define PERSIST_LOG_SIZE (0x200000)

/*last debug offset info in debug parition*/
#define LAST_DBG_MEM_LEN LAST_DBG_INFO_SIZE
#define LAST_DBG_HEAD_MAX_LEN (0x1000)
#define LAST_DBG_HEAD_OFFSET (LAST_DBG_MEM_LEN - LAST_DBG_HEAD_MAX_LEN)
#define LAST_DBG_HEAD_ADDR_OFFSET (LAST_DBG_MEM_LEN-2*LAST_DBG_HEAD_MAX_LEN)
#define LAST_INFO_MAX_LEN LAST_DBG_HEAD_ADDR_OFFSET

#define LAST_FSM_UNINT (0x0)
#define LAST_FSM_KERN_FAIL (0x1)
#define LAST_FSM_KERN_SUCC (0x2)
#define LAST_FSM_XBL_FAIL (0x3)
#define LAST_FSM_XBL_SUCC (0x4)
#define LAST_FSM_LK_SUCC (0x5)

#define LAST_FSM_REG_OFFSET 0x8E //reg is QPNP_PON_DVDD_RB_SPARE(x)
#define LAST_FSM_MASK (0x0e)
#define LAST_FSM_SHIFT (1)

#define LAST_FSM_ADDR_INVALID (0x0)
#define LAST_FSM_ADDR_VALID (BIT(4))
#define LAST_FSM_REG_ADDR_STATUS 0x8E

typedef struct {
	/* align by 64bit?*/
	u64 log_first_seq;
	u64 log_next_seq;
	u32 log_first_idx;
	u32 log_next_idx;
	u16 log_first_seq_crc16;
	u16 log_first_idx_crc16;
	u16 log_next_seq_crc16;
	u16 log_next_idx_crc16;
}last_kmsg_info_t;

typedef struct {
	u64 log_first_seq_addr;
	u64 log_first_seq_crc16_addr;
	u64 log_first_idx_addr;
	u64 log_first_idx_crc16_addr;
	u64 log_next_seq_addr;
	u64 log_next_seq_crc16_addr;
	u64 log_next_idx_addr;
	u64 log_next_idx_crc16_addr;
	u64 log_buf_addr;
	u32 log_buf_len;
	u32 reserved;
}last_kmsg_addr_info_t;

typedef struct
{
	u8 pon_reason1;
	u8 pon_reason2;
	u8 warm_reset_reason1;
	u8 warm_reset_reason2;
	u8 poff_reason1;
	u8 poff_reason2;
	u8 soft_reset_reason1;
	u8 soft_reset_reason2;
}pon_pm_reason_status_t;

typedef struct {
	u64 pm0;
	u64 pm1;
	u32 reset_status_reg;
	u32 reserved0;
}pon_poff_info_t;

typedef struct {
	u32 panic_reason;
	u32 reserved;
}panic_reason_info_t;

typedef struct {
	u64 last_panic_reason_addr;
}panic_reason_addr_info_t;

typedef struct {
	last_kmsg_info_t last_kmsg;
	panic_reason_info_t panic_info;
	pon_poff_info_t pon_poff;
}last_dbg_info_t;

typedef struct {
	last_kmsg_addr_info_t last_kmsg_addr;
	panic_reason_addr_info_t panic_reason_addr;
	char buildtime[BUILD_TIME_LEN];
}last_dbg_addr_info_t;

void* get_mem_last_info(void);
#endif


