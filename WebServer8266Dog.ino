#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <uri/UriBraces.h>
#include <uri/UriRegex.h>
#include "evodog.h"

ESP8266WebServer server(80);

// Setup Access Point Credentials
const char* ssid = "ESP32-CAM Robot";
const char* password = "1234567890";

void setup(){
Serial.begin(115200);
  evoSetup();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("");

  // Wait for connection
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("IP address: ");
  Serial.println(IP);

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on(F("/"), []() {
    server.send(200, "text/plain", "go to /cmd");
  });

  server.on(UriBraces("/cmd/{}"), []() {
    String user = server.pathArg(0);

    if (user.equals("i")){
      server.send(200, "text/plain", getInfo());
    }
    else {
    server.send(200, "text/plain", "cmd: '" + user + "'");
    }
    Serial.println(user);
    command(String(user));
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void) {
  server.handleClient();
  apploop();
}

String getInfo(){
  String output = "info," + String( flags.isset( FLAG_POWER ) ? 1 : 0 ) + "," + String(( flags.isset( FLAG_PAUSED ) ? 1 : 0 )) + "," + String(ultrasonic_distance);
      for ( char l = 0; l < 4; l++ )
      {
        for ( char j = 0; j < 3; j++ )
        {
          float apos = body.legs[l].cur_pos[j];
          if ( j == 0 )
          {
            if ( l == 1 || l == 3 )
              apos = -apos;
          }
          else if ( leg_info[l].flags.isset(REVERSED) )
            apos = -apos;

          output += ",";
          output+=String(round(apos));
        }
      }
     return output;
}
