/*
 * Smart HAMLOG app — StackChan のトップメニューから入る CAT(無線機制御)モード。
 * Phase 0: USB ホストを起動し、無線機(CP210x)が検出できるかを検証する。
 */
#pragma once
#include <mooncake.h>

class AppSmartHamlog : public mooncake::AppAbility {
public:
    AppSmartHamlog();

    // Override lifecycle callbacks
    void onCreate() override;
    void onOpen() override;
    void onRunning() override;
    void onClose() override;
};
