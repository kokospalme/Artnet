/*The MIT License (MIT)

Copyright (c) 2014 Nathanaël Lécaudé
https://github.com/natcl/Artnet, http://forum.pjrc.com/threads/24688-Artnet-to-OctoWS2811

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef ARTNET_H
#define ARTNET_H

#include <Arduino.h>
#include <AsyncUDP_ESP32_SC_Ethernet.hpp>

// UDP specific
#define ART_NET_PORT 6454
// Opcodes
#define ART_POLL 0x2000
#define ART_POLL_REPLY 0x2100
#define ART_DMX 0x5000
#define ART_SYNC 0x5200
#define ART_MAX_CHAR_LONG 64
#define ART_MAX_CHAR_SHORT 18
#define ART_MAX_UNIVERSES 16
// Buffers
#define MAX_BUFFER_ARTNET 530
// Packet
#define ART_NET_ID "Art-Net\0"
#define ART_DMX_START 18

#define SHORTNAME_DEFAULT "node1"
#define LONGNAME_DEFAUT "ArtnetNode1"
#define NODEREPORT_DEFAULT "tba..."
#define UNIVERSE1_DEFAULT 1
#define UNIVERSE2_DEFAULT -1
#define UNIVERSE3_DEFAULT -1
#define UNIVERSE4_DEFAULT -1
#define STATIC_IP_DEFAULT false

enum dmxinput_t { INPUT_ARTNET = 0, INPUT_DMX = 1, INPUT_NOW = 2};
enum dmxoutput_t { OUTPUT_NONE = -1, OUTPUT_ARTNET = 0, OUTPUT_DMX = 1, OUTPUT_NOW = 2};

struct artnet_config_s {
  uint16_t port = ART_NET_PORT;
  bool staticIp = STATIC_IP_DEFAULT;  //true if ip should be static
  String longname = LONGNAME_DEFAUT;  //max 64 character
  String shortname = SHORTNAME_DEFAULT;  //max. 18 characater
  String nodereport = NODEREPORT_DEFAULT;
  int inputuniverse[4] = {UNIVERSE1_DEFAULT, UNIVERSE2_DEFAULT, UNIVERSE3_DEFAULT, UNIVERSE4_DEFAULT};
};

struct artnet_reply_s {
  uint8_t  id[8];
  uint16_t opCode;
  uint8_t  ip[4];
  uint16_t port;
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  subH;
  uint8_t  sub;
  uint8_t  oemH;
  uint8_t  oem;
  uint8_t  ubea;
  uint8_t  status;
  uint8_t  etsaman[2];
  uint8_t  shortname[ART_MAX_CHAR_SHORT];
  uint8_t  longname[ART_MAX_CHAR_LONG];
  uint8_t  nodereport[ART_MAX_CHAR_LONG];
  uint8_t  numbportsH;
  uint8_t  numbports;
  uint8_t  porttypes[4];//max of 4 ports per node
  uint8_t  goodinput[4];
  uint8_t  goodoutput[4];
  uint8_t  swin[4];
  uint8_t  swout[4];
  uint8_t  swvideo;
  uint8_t  swmacro;
  uint8_t  swremote;
  uint8_t  sp1;
  uint8_t  sp2;
  uint8_t  sp3;
  uint8_t  style;
  uint8_t  mac[6];
  uint8_t  bindip[4];
  uint8_t  bindindex;
  uint8_t  status2;
  uint8_t  filler[26];
} __attribute__((packed));

class Artnet
{
public:
  static void begin(byte mac[], byte ip[]);
  static void parsePacket(AsyncUDPPacket packet);
  static bool setConfig(artnet_config_s conf);
  static void setLocalip(IPAddress ip);

  static void setBroadcastAuto(IPAddress ip, IPAddress sn);
  static void setBroadcast(byte bc[]);
  static void setBroadcast(IPAddress bc);
  static uint16_t read(AsyncUDPPacket *packet);
  static void printPacketHeader();
  static void printPacketContent();

  // Return a pointer to the start of the DMX data
  static uint8_t* getDmxFrame(void);
  static uint16_t getOpcode(void);
  static uint8_t getSequence(void);
  static uint16_t getUniverse(void);
  static uint16_t getLength(void);
  static IPAddress getRemoteIP(void);
  static void setArtDmxCallback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP));
  static void setUniverse1Callback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP));
  static void setUniverse2Callback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP));
  static void setUniverse3Callback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP));
  static void setUniverse4Callback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP));
  static void setArtSyncCallback(void (*fptr)(IPAddress remoteIP));

private:
  static uint8_t  node_ip_address[4];
  static uint8_t  id[8];

  static AsyncUDP udp; //udp instance
  static struct artnet_reply_s ArtPollReply;

  static artnet_config_s config;
  static uint8_t artnetPacket[MAX_BUFFER_ARTNET];
  static uint16_t packetSize;
  static IPAddress broadcast;
  static uint16_t opcode;
  static uint8_t sequence;
  static uint16_t incomingUniverse;
  static uint16_t dmxDataLength;
  static IPAddress remoteIP;
  static IPAddress myIP;
  static void (*artDmxCallback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
  static void (*universe1Callback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
  static void (*universe2Callback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
  static void (*universe3Callback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
  static void (*universe4Callback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
  static void (*artSyncCallback)(IPAddress remoteIP);
};

#endif
