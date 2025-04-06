// wifimonitor.cpp
#include "wifimonitor.h"
#include <vector>
#include "credentials.h"
#include "debuglog.h"

extern void stop_motors();

WiFiMonitor::WiFiMonitor() {
    publish_count = 0;
}

void WiFiMonitor::begin(rcl_node_t * node) {
    
    rclc_publisher_init_default(&wifi_publisher, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "wifi");
}

// connect using ssid and pwd
void WiFiMonitor::Connect(){
    WiFi.disconnect(true);   // Reset Wi-Fi
    WiFi.mode(WIFI_STA);     // Set to Station mode
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    LOG_INFO("Connected to Wi-Fi");
    LOG_INFO("IP Address: %s", WiFi.localIP().toString().c_str());
    LOG_INFO("SSID: %s",WiFi.SSID().c_str());
    LOG_INFO("BSSID: %s", WiFi.BSSIDstr().c_str());
}

// connect using ssid bssid and pwd
void WiFiMonitor::Connect(Hotspot hotspot){
    WiFi.disconnect(true);   // Reset Wi-Fi
    WiFi.mode(WIFI_STA);     // Set to Station mode
    WiFi.begin(ssid.c_str(), pass.c_str(), 0, hotspot.bssid,true);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    LOG_INFO("Connected to Wi-Fi");
    LOG_INFO("IP Address: %s", WiFi.localIP().toString().c_str());
    LOG_INFO("SSID: %s",WiFi.SSID().c_str());
    LOG_INFO("BSSID: %s", WiFi.BSSIDstr().c_str());
}


void WiFiMonitor::publishStatus() {
    if (WiFi.status() == WL_CONNECTED) {
        // Retrieve WiFi information
        String ssid = WiFi.SSID();
        String bssid = WiFi.BSSIDstr();
        int rssi = WiFi.RSSI();

        // Format WiFi information
        snprintf(wifi_status, sizeof(wifi_status), "SSID: %s, BSSID: %s, RSSI: %d dBm", ssid.c_str(), bssid.c_str(), rssi);
        // Serial.println(wifi_status);

        // Publish WiFi information
        wifi_msg.data.data = wifi_status;
        wifi_msg.data.size = strlen(wifi_status);
        wifi_msg.data.capacity = sizeof(wifi_status);
        rcl_publish(&wifi_publisher, &wifi_msg, NULL);
    } else {
        LOG_WARN("WiFi disconnected!");
    }


    // Check Wi-Fi strength and reconnect if needed every 10 calls.
    if (publish_count++ % 10 == 0) {
        checkAndReconnectWiFi();
    }
}

String WiFiMonitor::BssidToString(uint8_t * bssid){

    String bssid_str;
    char sbyte[3];
    for(int i=0;i<6;i++){
        sprintf(sbyte,"%x",bssid[i]);
        bssid_str +=String(sbyte);
        if(i<5)
            bssid_str += ":";
    }
    return bssid_str;
}

WiFiMonitor::Hotspot WiFiMonitor::findStrongestHotspot() {
        /*
         Scan for available Wi-Fi hotspots and return the strongest one with the same SSID.
        */
        WiFi.disconnect(); // Disconnect from current network to perform scan
        delay(100);
        int networks = WiFi.scanNetworks();
        std::vector<Hotspot> hotspots;

        for (int i = 0; i < networks; ++i) {
            if (WiFi.SSID(i) == ssid) {
                struct Hotspot hotspot;
                hotspot.bssidstr = WiFi.BSSIDstr(i);
                memcpy(hotspot.bssid, WiFi.BSSID(i),6);
                hotspot.ssid = WiFi.SSID(i);
                hotspot.strength = WiFi.RSSI(i);
                hotspots.push_back(hotspot);
                LOG_DEBUG("found: ssid=%s bssid=%s rssi=%ddBm",hotspot.ssid.c_str(), hotspot.bssidstr.c_str(), hotspot.strength );
            }
        }

        // Find the hotspot with the strongest signal.
        auto strongest = std::max_element(hotspots.begin(), hotspots.end(),
                                          [](const Hotspot &a, const Hotspot &b) {
                                              return a.strength < b.strength;
                                          });
        return *strongest;
}


void WiFiMonitor::checkAndReconnectWiFi() {
        /*
         Check Wi-Fi signal strength and reconnect to the strongest hotspot if needed.
        */
        unsigned long getwifi = millis();
        int current_strength = WiFi.RSSI();
        getwifi = millis() - getwifi;
        LOG_INFO("Current Wi-Fi strength: %d dBm  SSID:%s BSSID: %s in %lu millis", 
            current_strength, WiFi.SSID().c_str(),WiFi.BSSIDstr().c_str(), getwifi);

        if (current_strength < -75) {
            LOG_WARN("Signal strength is low. Searching for stronger hotspot...");
            stop_motors(); 
            Hotspot strongest_hotspot = findStrongestHotspot();

            if (strongest_hotspot.strength > current_strength) {
                LOG_INFO("Connecting to stronger hotspot: ssid=%s bssid=%s (%d dBm)",
                    strongest_hotspot.ssid.c_str(), strongest_hotspot.bssidstr.c_str(), strongest_hotspot.strength);
                Connect(strongest_hotspot);
            } else {
                LOG_INFO("No stronger hotspot available.");
            }
        }
}

