#ifndef POWER_SELF_TEST_H
#define POWER_SELF_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile float g_virtual_load_ohm;

void power_self_test_step(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_SELF_TEST_H
