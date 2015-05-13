#ifndef __AXP209_EVENT_H__
#define __AXP209_EVENT_H__

#define AXP209_PEK_PRESS            (1 << 0)
#define AXP209_PEK_PRESS_LONG       (1 << 1)
#define AXP209_VBUS_INSERT          (1 << 2)
#define AXP209_VBUS_REMOVE          (1 << 3)
#define AXP209_CHARGING             (1 << 4)
#define AXP209_CHARG_FINISHED       (1 << 5)
#define AXP209_ACIN_INSERT          (1 << 6)
#define AXP209_ACIN_REMOVE          (1 << 7)
#define AXP209_SET_TIMER            (1 << 8)

enum{
    ELEC80_100 = 0x20,
    ELEC50_80,
    ELEC20_50,
    ELEC0_20,
    ELEC80_100_C,
    ELEC50_80_C,
    ELEC20_50_C,
    ELEC0_20_C,
};

#endif
