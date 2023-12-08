
#include <limits.h>
#include "unity.h"
#include "sht31_driver.h"

#define countof(x) (sizeof(x)/sizeof(x[0]))

TEST_CASE("Known CRC", "[sht31_driver]")
{
    const uint8_t data[] = { 0x01, 0x02, 0x03 };
    TEST_ASSERT_EQUAL(0x17, SHT31_CRC(data) );
}
