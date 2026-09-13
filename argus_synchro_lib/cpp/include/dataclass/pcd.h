#pragma once

// *** クラス ***
class PcdData
{
  public:
    static constexpr unsigned int SIZE = 40000U;
};

enum class PCD : int
{
    X = 0,
    Y = 1,
    Z = 2,
    INTENSITY = 3, // 反射強度;
    TIME = 4,      // TTimeStamp;
    CH = 3,        // チャンネル数;
    // enum型の思想からするとイレギュラーな数値定義;
    XYZ = 3, // xyzに限定したいデータ処理用;
};