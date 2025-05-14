#include <Arduino.h>
#include <unity.h>
#include <Blinker.h>

Blinker testLed(2);

void test_initial_state() {
    TEST_ASSERT_EQUAL(false, testLed.getState());
}

void test_toggle_once() {
    testLed.toggle();
    TEST_ASSERT_EQUAL(true, testLed.getState());
}

void test_toggle_twice() {
    testLed.toggle();
    testLed.toggle();
    TEST_ASSERT_EQUAL(false, testLed.getState());
}

void setup() {
    delay(1000);
    testLed.begin();
    UNITY_BEGIN();
    RUN_TEST(test_initial_state);
    RUN_TEST(test_toggle_once);
    RUN_TEST(test_toggle_twice);
    UNITY_END();
}

void loop() {}
