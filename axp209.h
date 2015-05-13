#ifndef _AXP209_H
#define _AXP209_H
#include <linux/ioctl.h>

#define SET_BIT(value, bit)     (value |= (0x1 << bit))
#define CLR_BIT(value, bit)     (value &= ~(0x1 << bit))

#define AXP209_DEV_NAME "axp209"
#define AXP209_IOC_MAGIC        't'

#define AXP209_GET_REG           _IOR(AXP209_IOC_MAGIC, 0xff, unsigned short)
#define AXP209_SET_REG           _IOW(AXP209_IOC_MAGIC, 0xff, unsigned short)
#define AXP209_GET_ELEC          _IOR(AXP209_IOC_MAGIC, 0xb9, unsigned char)
#define AXP209_SET_IRQ_MASK      _IOW(AXP209_IOC_MAGIC, 0xfe, unsigned int)
#define AXP209_GET_IRQ_MASK      _IOR(AXP209_IOC_MAGIC, 0xfe, unsigned int)
#define AXP209_CLEAR_IRQ_MASK    _IOW(AXP209_IOC_MAGIC, 0xfd, unsigned int)

/*
#define AXP209_GET_ACIN_STATUS   _IOR(AXP209_IOC_MAGIC, 0x00, char)
#define AXP209_GET_MODEL    _IOR(AXP209_IOC_MAGIC, 0x01, char)
#define AXP209_POWER_OFF    _IO(AXP209_IOC_MAGIC,0x32)
*/
#define AXP209_GET_ADDR          _IOR(AXP209_IOC_MAGIC, 0xfe, unsigned int)

#endif
