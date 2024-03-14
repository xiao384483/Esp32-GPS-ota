//下面注释代码用于识别整数、浮点、字符串对应的不同输出
/* if (!rxValue.empty()) {
    char *endptr;
    // 尝试转换为浮点数
    float floatValue = std::strtof(rxValue.c_str(), &endptr);
    if (*endptr == '\0') { // 如果转换后的指针指向字符串的末尾，表示转换成功
        if (floatValue > 0 && floatValue < 101) {
            // 浮点数在有效范围内
            display.clear();
            display.drawString(0, 20, String(floatValue) + "%");
            display.drawRect(0, 40, 60, 10);
            display.fillRect(0, 42, (60 * floatValue / 100), 8);
        } else {
            // 浮点数超出有效范围
            display.clear();
            display.drawString(0, 20, "FAILED");
        }
    } else {
        // 尝试转换为整数
        char *endptr;
        long intValue = std::strtol(rxValue.c_str(), &endptr, 10);
        if (*endptr == '\0') { // 如果转换后的指针指向字符串的末尾，表示转换成功
            if (intValue > 0 && intValue < 101) {
                // 整数在有效范围内
                display.clear();
                display.drawString(0, 20, String(intValue) + "%");
                display.drawRect(0, 40, 60, 10);
                display.fillRect(0, 42, (60 * intValue / 100), 8);
            } else {
                // 整数超出有效范围
                display.clear();
                display.drawString(0, 20, "FAILED");
            }
        } else {
            // 既不是浮点数也不是整数，视为字符串
            display.clear();
            display.drawString(0, 20, String(rxValue.c_str()) + " yyds!");
        }
    }
}*/