/* AUTOGENERATED FILE. DO NOT EDIT. */

//=======Test Runner Used To Run Each Test Below=====
#define RUN_TEST(TestFunc, TestLineNum)            \
    \
{                                           \
        Unity.CurrentTestName = #TestFunc;         \
        Unity.CurrentTestLineNumber = TestLineNum; \
        Unity.NumberOfTests++;                     \
        if (TEST_PROTECT())                        \
        {                                          \
            setUp();                               \
            TestFunc();                            \
        }                                          \
        if (TEST_PROTECT() && !TEST_IS_IGNORED)    \
        {                                          \
            tearDown();                            \
        }                                          \
        UnityConcludeTest();                       \
    \
}

//=======Automagically Detected Files To Include=====
#include "unity.h"

//=======External Functions This Runner Calls=====
extern void setUp(void);
extern void tearDown(void);
extern void test_mma9553_init(void);
extern void test_mma9553_config(void);
extern void test_mma9553_deInit(void);
extern void test_mma9553_negative_1(void);
extern void test_mma9553_negative_2(void);
extern void test_mma9553_negative_3(void);
extern void test_mma9553_negative_4(void);

//=======Test Reset Option=====
void resetTest(void);
void resetTest(void)
{
    tearDown();
    setUp();
}

//=======MAIN=====
int main(void)
{
    UnityBegin("test_mma9553.c");

    RUN_TEST(test_mma9553_negative_1, 0);
    RUN_TEST(test_mma9553_negative_2, 0);
    RUN_TEST(test_mma9553_init, 0);
    RUN_TEST(test_mma9553_negative_3, 0);
    RUN_TEST(test_mma9553_config, 0);
    RUN_TEST(test_mma9553_negative_4, 0);
    RUN_TEST(test_mma9553_deInit, 0);

    return (UnityEnd());
}
