#include "pru_rpmsg.h" //for pru_rpmsg_transport
#include "sys_mailbox.h" //for CT_MBX
#include "pru_cfg.h" //for CT_CFG

void main()
{
  struct pru_rpmsg_transport transport;
  uint16_t src, dst, len;
  volatile uint8_t *status;

  CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
  
  status = &resourceTable.rpmsg_vdev.status;
  while(!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

  pru_virtqueue_init(&transport.virtqueue0, &resourceTable.rpmsg_vring0, &CT_MBX.MESSAGE[MB_TO_ARM_HOST], &CT_MBX.MESSAGE[MB_FROM_ARM_HOST]);

  pru_virtqueue_init(&transport.virtqueue1, &resourceTable.rpmsg_vring1, &CT_MBX.MESSAGE[MB_TO_ARM_HOST], &CT_MBX.MESSAGE[MB_FROM_ARM_HOST]);

}
