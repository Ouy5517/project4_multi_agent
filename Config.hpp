#pragma once
#include <string>

struct Vec2 { float x; float y; };

// 集中管理固定场景参数
constexpr Vec2 FIXED_PASS_TARGET = {7.0f, 4.0f};
const std::string PASSER_ID = "R1";
const std::string RECEIVER_ID = "R2";
constexpr float FIELD_WIDTH = 12.0f;
constexpr float FIELD_HEIGHT = 8.0f;
constexpr float ARRIVAL_TOLERANCE = 0.12f;