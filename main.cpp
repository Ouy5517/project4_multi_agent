#include <iostream>
#include <string>
#include <cstdlib> // 用于 atof, atoi

int main(int argc, char* argv[]) {
    // 默认参数
    std::string scenario = "default";
    float target_x = FIXED_PASS_TARGET.x;
    float target_y = FIXED_PASS_TARGET.y;
    int duration = 15;
    bool export_csv = false;

    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            scenario = argv[++i];
        } else if (arg == "--target-x" && i + 1 < argc) {
            target_x = std::atof(argv[++i]);
        } else if (arg == "--target-y" && i + 1 < argc) {
            target_y = std::atof(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::atoi(argv[++i]);
        } else if (arg == "--export-csv") {
            export_csv = true;
        }
    }

    std::cout << "启动场景: " << scenario << "\n"
              << "目标坐标: (" << target_x << ", " << target_y << ")\n"
              << "持续时间: " << duration << "s\n"
              << "导出CSV: " << (export_csv ? "是" : "否") << std::endl;

    // ... 接着启动你的状态机主循环 ...
    return 0;
}