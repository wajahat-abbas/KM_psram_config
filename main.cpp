#include "mbed.h"


/** Macro generating the mask for a bitfield of \p n bits */
#define DRIVER_BIT_SET(n)                      (1u<<(n))

#define DRIVER_BITS_CLR(data, mask)            ((data) & ~(mask))
#define DRIVER_BITS_SET(data, bits)            ((data) |  (bits))

/** Macro generating the mask for a bitfield of \p n bits */
#define DRIVER_BIT_MASK(n)                      (DRIVER_BIT_SET(n) - 1)

/** Macro generating the mask for a bitfield defined as `name_OFFSET` and `name_SIZE` */
#define DRIVER_BITFIELD_MASK_(name)             (DRIVER_BIT_MASK(name##_SIZE) << (name##_OFFSET))
#define DRIVER_BITFIELD_MASK(name)              DRIVER_BITFIELD_MASK_(name)

/** Extract bitfield defined as `name_OFFSET` and `name_SIZE` from \p data */
#define DRIVER_BITFIELD_GET_(data, name)        (((data) >> name##_OFFSET) & DRIVER_BIT_MASK(name##_SIZE))
#define DRIVER_BITFIELD_GET(data, name)         DRIVER_BITFIELD_GET_(data, name)

/** Return \p data with bitfield defined as `name_OFFSET` and `name_SIZE` cleared */
#define DRIVER_BITFIELD_CLR(data, name)         ((data) & ~DRIVER_BITFIELD_MASK(name))

/** Return \p bitfield defined as `name_OFFSET` and `name_SIZE` set to \p value */
#define DRIVER_BITFIELD_VAL_(name, value)       (((value) & DRIVER_BIT_MASK(name##_SIZE)) << name##_OFFSET)
#define DRIVER_BITFIELD_VAL(name, value)        DRIVER_BITFIELD_VAL_(name, value)

/** Return \p data with bitfield defined as `name_OFFSET` and `name_SIZE` set to \p value */
#define DRIVER_BITFIELD_SET(data, name, value)  (DRIVER_BITFIELD_CLR(data, name) | DRIVER_BITFIELD_VAL(name, value))

/** Return \p bitfield defined as `name_OFFSET` and `name_SIZE` set to \p name_ENUM_value */
#define DRIVER_BITFIELD_ENUM_(name, enumValue)      DRIVER_BITFIELD_VAL(name, name##_ENUM_##enumValue)
#define DRIVER_BITFIELD_ENUM(name, enumValue)       DRIVER_BITFIELD_ENUM_(name, enumValue)

/** Return \p data with bitfield defined as `name_OFFSET` and `name_SIZE` set to \p name_ENUM_value */
#define DRIVER_BITFIELD_SET_ENUM(data, name, enumValue)  DRIVER_BITFIELD_SET(data, name,  DRIVER_BITFIELD_ENUM(name, enumValue))

int main() {

     struct psram_s *app_ss_psram_ctrl = (struct psram_s*)(0x49600000);
     struct kmgo_sysctrl_s *pwr_ctrl = (struct kmgo_sysctrl_s*)(0x49000000);

    // Disable AXI
      app_ss_psram_ctrl->axi_ctrl = 0x00000000;

#ifdef PSRAM_ASYNC_MODE_ENABLED

      // Set async mode read and write latencies
      app_ss_psram_ctrl->async_mode_latency = (DRIVER_BITFIELD_VAL(PSRAM_ASYNC_MODE_LATENCY_ASYNC_MODE_READ_LATENCY,  0x4) |
                                               DRIVER_BITFIELD_VAL(PSRAM_ASYNC_MODE_LATENCY_ASYNC_MODE_WRITE_LATENCY, 0x3) ); // 0x00000403

      // Set address width
      app_ss_psram_ctrl->psram_addr_width = ((1 << 24) - 1); //0x00FFFFFF

#else // Assume DDR mode enabled

      // Set sync latency + wait type
      app_ss_psram_ctrl->sync_mode_latency =(DRIVER_BITFIELD_VAL(PSRAM_SYNC_MODE_LATENCY_SYNC_MODE_READ_LATENCY,  0x2) |
                                             DRIVER_BITFIELD_VAL(PSRAM_SYNC_MODE_LATENCY_SYNC_MODE_WRITE_LATENCY, 0x2) ); //0x00000022

      // Set address width
      app_ss_psram_ctrl->psram_addr_width = ((1 << 24) - 1); //0x00FFFFFF

      // Set PHY CFG, AP memory DDR index in BCR
      app_ss_psram_ctrl->psram_controller_phy_cfg = (DRIVER_BITFIELD_VAL(PSRAM_PSRAM_CONTROLLER_PHY_CFG_PSRAM_CONTROLLER_PHY_DQS_SWITCH_ENABLE, 0x1) |
                                                     DRIVER_BITFIELD_VAL(PSRAM_PSRAM_CONTROLLER_PHY_CFG_PSRAM_CONTROLLER_PHY_BCR_DDR_INDEX,     0x9) |
                                                     DRIVER_BITFIELD_VAL(PSRAM_PSRAM_CONTROLLER_PHY_CFG_PSRAM_CONTROLLER_PHY_RX_WAIT_FEEDBACK,  0x1) |
                                                     DRIVER_BITFIELD_VAL(PSRAM_PSRAM_CONTROLLER_PHY_CFG_PSRAM_CONTROLLER_PHY_RX_TIMEOUT,        0xA) |
                                                     DRIVER_BITFIELD_VAL(PSRAM_PSRAM_CONTROLLER_PHY_CFG_PSRAM_CONTROLLER_PHY_RX_TIMEOUT_ENABLE, 0x1) ); //0x00005598

      // Set PHY read data capture DQs
      app_ss_psram_ctrl->phy_read_data_capture = 0x00000001;

      //Enable BCR write request
      app_ss_psram_ctrl->bcr = (DRIVER_BITFIELD_VAL(PSRAM_BCR_BURST_LENGTH,           0x7) |
                                DRIVER_BITFIELD_VAL(PSRAM_BCR_BURST_WRAP,             0x1) |
                                DRIVER_BITFIELD_VAL(PSRAM_BCR_DRIVE_STRENGTH,         0x1) |
                                DRIVER_BITFIELD_VAL(PSRAM_BCR_WAIT_CONFIGURATION,     0x1) |
                                DRIVER_BITFIELD_VAL(PSRAM_BCR_SDR_DDR_MODE_AP_MEMORY, 0x1) |
                                DRIVER_BITFIELD_VAL(PSRAM_BCR_WAIT_POLARITY,          0x1) |
                                DRIVER_BITFIELD_VAL(PSRAM_BCR_LATENCY_COUNTER,        0x2) |
                                DRIVER_BITFIELD_VAL(PSRAM_BCR_INITIAL_LATENCY,        0x1) ); //0x0000571F;

      app_ss_psram_ctrl->bcr_write_req = 0x1;

      while(((app_ss_psram_ctrl->bcr_write_req) & 0x1) == 0x1)
      {
        asm volatile("NOP");
      }

      if(((app_ss_psram_ctrl->psram_irq_and_fault_status_raw) & 0x1) == 0x1)
      {
        app_ss_psram_ctrl->psram_irq_and_fault_status_clr = 0x1;
      }

#endif // #ifdef PSRAM_ASYNC_MODE_ENABLED

      // Re-enable AXI
      app_ss_psram_ctrl->axi_ctrl = 0x00000001; //(1 << PSRAM_AXI_CTRL_AXI_ENABLE_OFFSET)

      printf("psram configured\r\n");

      //NVIC_SystemReset();
      pwr_ctrl->rst_sw_strobe_1 = (1<<KMGO_SYSCTRL_RST_SW_STROBE_1_CPU_APP_OFFSET);

      while(1);
}
