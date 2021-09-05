 #pragma once

namespace Tools{
    // 判断一个double类型的数是否等于0
    bool isZero(double value);

    // 判断前一个double是否大于后一个double
    bool isLarge(double value_1, double value_2);

    // 判断前一个double是否小于后一个double
    bool isSmall(double value_1, double value_2);

    // 判断前一个double是否等于后一个double
    bool isEqual(double value_1, double value_2);
};
