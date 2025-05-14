//! For Alerts testing

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "../src/util/alerts.h"

class AlertManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        alertManager = new AlertManager(50.0f, 30.0f, 0.5f);
    }

    void TearDown() override {
        delete alertManager;
    }

    AlertManager* alertManager;
};

TEST_F(AlertManagerTest, SpeedViolationTest) {
    cv::Rect box(100, 100, 50, 50);
    alertManager->checkSpeedViolation(1, "car", 60.0f, box);
    
    auto alerts = alertManager->getActiveAlerts();
    EXPECT_EQ(alerts.size(), 1);
    EXPECT_EQ(alerts[0].type, AlertType::SPEED_VIOLATION);
}

TEST_F(AlertManagerTest, RestrictedAreaTest) {
    cv::Rect box(0, 0, 100, 100);
    cv::Rect zone(0, 0, 100, 100);
    
    alertManager->checkRestrictedArea(1, "car", box, zone);
    auto alerts = alertManager->getActiveAlerts();
    EXPECT_EQ(alerts.size(), 1);
    EXPECT_EQ(alerts[0].type, AlertType::RESTRICTED_AREA);
}

TEST_F(AlertManagerTest, AlertCleanup) {
    cv::Rect box(100, 100, 50, 50);
    alertManager->checkSpeedViolation(1, "car", 60.0f, box);
    
    // Simulate time passing
    std::this_thread::sleep_for(std::chrono::seconds(2));
    alertManager->clearOldAlerts(1);  // Clear alerts older than 1 second
    
    auto alerts = alertManager->getActiveAlerts();
    EXPECT_EQ(alerts.size(), 0);
}
