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

#include <Artnet.h>

uint8_t Artnet::node_ip_address[4];
uint8_t Artnet::id[8];

AsyncUDP Artnet::udp;
struct artnet_reply_s Artnet::ArtPollReply;
// SemaphoreHandle_t* Artnet::mutex;

artnet_config_s Artnet::config;
uint8_t Artnet::artnetPacket[MAX_BUFFER_ARTNET];
uint16_t Artnet::packetSize;
IPAddress Artnet::broadcast;
uint16_t Artnet::opcode;
uint8_t Artnet::sequence;
uint16_t Artnet::incomingUniverse;
uint16_t Artnet::dmxDataLength;
IPAddress Artnet::remoteIP;
IPAddress Artnet::myIP;
void (*Artnet::artDmxCallback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
void (*Artnet::universe1Callback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
void (*Artnet::universe2Callback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
void (*Artnet::universe3Callback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
void (*Artnet::universe4Callback)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP);
void (*Artnet::artSyncCallback)(IPAddress remoteIP);


/**
 * @brief Initializes the Artnet library with the specified MAC and IP addresses.
 *
 * This function sets up the necessary configurations and prepares the Artnet
 * library for use. It initializes network settings with the provided MAC and IP
 * addresses, configures UDP communication, and registers a callback function for
 * handling incoming Art-Net data packets.
 *
 * The function must be called before any other Artnet operations are performed.
 *
 * @param mac A byte array containing the MAC address for the network interface.
 * @param ip A byte array containing the IP address for the device.
 * @return void
 * */
void Artnet::begin(byte mac[], byte ip[]){

  if (udp.listen(config.port)) {
      Serial.print("Listening for Art-Net on port: ");
      Serial.println(config.port);

      // Funktion zur Verarbeitung eingehender Pakete registrieren
      udp.onPacket([](AsyncUDPPacket packet) {
          parsePacket(packet);
      });
  } else {
      Serial.println("Failed to start UDP server");
  }
}


/**
 * @brief Parses an incoming Art-Net packet.
 *
 * This function is called when an Art-Net packet is received. It handles the
 * parsing of the packet by delegating the actual reading of the packet data
 * to the `read` function. This function provides optional debug output to the
 * Serial console, including the remote IP address, remote port, and the packet
 * length.
 *
 * @param packet The incoming Art-Net packet to be parsed. *
 * @return void
 */
void Artnet::parsePacket(AsyncUDPPacket packet) {
    // Serial.println("Art-Net Packet Received:");unter welch
    // Serial.print("From: ");
    // Serial.print(packet.remoteIP());
    // Serial.print(":");
    // Serial.println(packet.remotePort());
    // Serial.print("Length: ");
    // Serial.println(packet.length());

  read(&packet);

    // Serial.println();
}

/**
 * @brief Sets the Art-Net configuration for the node.
 *
 * This function configures the Art-Net node with the specified settings. It validates
 * the provided configuration parameters, such as the long name, short name, node report,
 * and input universes, ensuring they do not exceed the maximum allowed lengths and values.
 * If any parameter is invalid, the function prints an error message to the Serial console
 * and returns false. If all parameters are valid, the configuration is applied, and the
 * function returns true.
 *
 * @param conf The Art-Net configuration settings to be applied.
 * @return true if the configuration is valid and has been set successfully, false otherwise.
 */
bool Artnet::setConfig(artnet_config_s conf){
  if(conf.longname.length() > ART_MAX_CHAR_LONG){
    Serial.println("long name is too long (max 64 character).");
    return false;
  }
  if(conf.shortname.length() > ART_MAX_CHAR_SHORT){
    Serial.println("short name is too long (max 18 character).");
    return false;
  }
  if(conf.nodereport.length() > ART_MAX_CHAR_LONG){
    Serial.println("nodereport is too long (max 64 character).");
    return false;
  }
  for(int i = 0; i < 4; i++){
    if(conf.inputuniverse[i] > ART_MAX_UNIVERSES){
      Serial.println("invalid universe (0...16)");
      return false;
    }
  }
  config = conf;
  return true;
}

/**
 * @brief Sets the local IP address of the Art-Net node.
 *
 * This function sets the IP address of the Art-Net node to the specified IP address.
 * This IP address is used by the node for network communication.
 *
 * @param ip The IP address to be set as the local IP address of the node.
 * @return void
 */
void Artnet::setLocalip(IPAddress ip){
  myIP = ip;
}

/**
 * @brief Automatically sets the broadcast address based on the given IP address and subnet mask.
 *
 * This function calculates the broadcast address by performing a bitwise operation
 * on the provided IP address and subnet mask. The calculated broadcast address is
 * then set for the Art-Net node.
 *
 * @param ip The IP address of the node.
 * @param sn The subnet mask to be used for calculating the broadcast address.
 * @return void
 */
void Artnet::setBroadcastAuto(IPAddress ip, IPAddress sn){
  //Cast in uint 32 to use bitwise operation of DWORD
  uint32_t ip32 = ip;
  uint32_t sn32 = sn;

  //Find the broacast Address
  uint32_t bc = (ip32 & sn32) | (~sn32);

  //sets the broadcast address
  setBroadcast(IPAddress(bc));
}

/**
 * @brief Sets the broadcast address of the Art-Net node.
 *
 * This function sets the broadcast address of the Art-Net node to the specified
 * address provided as a byte array.
 *
 * @param bc The broadcast address to be set, given as a byte array.
 * @return void
 */
void Artnet::setBroadcast(byte bc[]){
  //sets the broadcast address
  broadcast = bc;
}

/**
 * @brief Sets the broadcast address of the Art-Net node.
 *
 * This function sets the broadcast address of the Art-Net node to the specified
 * address provided as an IPAddress object.
 *
 * @param bc The broadcast address to be set, given as an IPAddress object.
 * @return void
 */
void Artnet::setBroadcast(IPAddress bc){
  //sets the broadcast address
  broadcast = bc;
}

/**
 * @brief Reads and processes an incoming Art-Net packet.
 *
 * This function reads an incoming Art-Net packet, verifies its integrity, and
 * processes it based on its opcode. It supports handling DMX, Poll, and Sync
 * packets, invoking appropriate callbacks for each type.
 *
 * @param packet The incoming Art-Net packet to be read and processed.
 * @return The opcode of the processed packet, or 0 if the packet is invalid.
 */
uint16_t Artnet::read(AsyncUDPPacket *packet){
  packetSize = packet->length();

  // remoteIP = udp.remoteIP();
  remoteIP = packet->remoteIP();
  if (packetSize <= MAX_BUFFER_ARTNET && packetSize > 0)
  {
      memcpy(artnetPacket, packet->data(), MAX_BUFFER_ARTNET);
      // Check that packetID is "Art-Net" else ignore
      for (byte i = 0 ; i < 8 ; i++)
      {
        if (artnetPacket[i] != ART_NET_ID[i])
          return 0;
      }

      opcode = artnetPacket[8] | artnetPacket[9] << 8;

      if (opcode == ART_DMX)
      {
        sequence = artnetPacket[12];
        incomingUniverse = artnetPacket[14] | artnetPacket[15] << 8;
        dmxDataLength = artnetPacket[17] | artnetPacket[16] << 8;
    
        if (artDmxCallback) (*artDmxCallback)(incomingUniverse, dmxDataLength, sequence, artnetPacket + ART_DMX_START, remoteIP);

        if(universe1Callback){  //Universe 1
          for(int i = 0; i < 4; i++){
            if(incomingUniverse == config.inputuniverse[0])(*universe1Callback)(incomingUniverse, dmxDataLength, sequence, artnetPacket + ART_DMX_START, remoteIP);
          }
        }

        if(universe2Callback){  // Universe 2
          for(int i = 0; i < 4; i++){
            if(incomingUniverse == config.inputuniverse[1])(*universe2Callback)(incomingUniverse, dmxDataLength, sequence, artnetPacket + ART_DMX_START, remoteIP);
          }
        }

        if(universe3Callback){  // Universe 3
          for(int i = 0; i < 4; i++){
            if(incomingUniverse == config.inputuniverse[2])(*universe3Callback)(incomingUniverse, dmxDataLength, sequence, artnetPacket + ART_DMX_START, remoteIP);
          }
        }

        if(universe4Callback){
          for(int i = 0; i < 4; i++){
            if(incomingUniverse == config.inputuniverse[3])(*universe4Callback)(incomingUniverse, dmxDataLength, sequence, artnetPacket + ART_DMX_START, remoteIP);
          }
        }
        delay(5);
        return ART_DMX;
      }
      if (opcode == ART_POLL){
        //fill the reply struct, and then send it to the network's broadcast address
        Serial.print("POLL from ");
        Serial.print(remoteIP);
        Serial.print(" broadcast addr: ");
        Serial.println(broadcast);

        // Aufteilen des Strings an den Punkten und Konvertieren in Zahlen
        int startIndex = 0;
        int partIndex = 0;
        String ipString = myIP.toString();
        for (int i = 0; i < ipString.length(); i++) {
          if (ipString.charAt(i) == '.') {
            node_ip_address[partIndex] = ipString.substring(startIndex, i).toInt();
            startIndex = i + 1;
            partIndex++;
          }
        }
        // Letztes Oktett hinzufügen
        node_ip_address[3] = ipString.substring(startIndex).toInt();

        sprintf((char *)id, "Art-Net");
        memcpy(ArtPollReply.id, id, sizeof(ArtPollReply.id));
        memcpy(ArtPollReply.ip, node_ip_address, sizeof(ArtPollReply.ip));

        ArtPollReply.opCode = ART_POLL_REPLY;
        ArtPollReply.port =  ART_NET_PORT;

        memset(ArtPollReply.goodinput,  0x08, 4);
        memset(ArtPollReply.goodoutput,  0x80, 4);
        memset(ArtPollReply.porttypes,  0xc0, 4);

        uint8_t shortname [18];
        uint8_t longname [64];

        const char* _shortname = config.shortname.c_str();
        const char* _longname = config.longname.c_str();
        
        sprintf((char *)shortname, _shortname);
        sprintf((char *)longname, _longname);

        memcpy(ArtPollReply.shortname, shortname, sizeof(shortname));
        memcpy(ArtPollReply.longname, longname, sizeof(longname));

        ArtPollReply.etsaman[0] = 0;
        ArtPollReply.etsaman[1] = 0;
        ArtPollReply.verH       = 1;
        ArtPollReply.ver        = 0;
        ArtPollReply.subH       = 0;
        ArtPollReply.sub        = 0;
        ArtPollReply.oemH       = 0;
        ArtPollReply.oem        = 0xFF;
        ArtPollReply.ubea       = 0;
        ArtPollReply.status     = 0xd2;
        ArtPollReply.swvideo    = 0;
        ArtPollReply.swmacro    = 0;
        ArtPollReply.swremote   = 0;
        ArtPollReply.style      = 0;

        ArtPollReply.numbportsH = 0;
        ArtPollReply.numbports  = 4;
        ArtPollReply.status2    = 0x08;

        ArtPollReply.bindip[0] = node_ip_address[0];
        ArtPollReply.bindip[1] = node_ip_address[1];
        ArtPollReply.bindip[2] = node_ip_address[2];
        ArtPollReply.bindip[3] = node_ip_address[3];

        uint8_t swin[4]  = {0x01,0x02,0x03,0x04};
        uint8_t swout[4] = {0x01,0x02,0x03,0x04};
        for(uint8_t i = 0; i < 4; i++)
        {
            ArtPollReply.swout[i] = swout[i];
            ArtPollReply.swin[i] = swin[i];
        }
        sprintf((char *)ArtPollReply.nodereport, "%i DMX output universes active.", ArtPollReply.numbports);
        AsyncUDPMessage message(sizeof(ArtPollReply));  //new message for the reply
        message.write((uint8_t*)&ArtPollReply, sizeof(ArtPollReply)); //fill the message with data

        if (udp.broadcastTo(message, ART_NET_PORT, TCPIP_ADAPTER_IF_ETH)) { //send reply
          Serial.println("Packet sent successfully.");
        } else {
          Serial.println("Failed to send packet.");
        }
        

        return ART_POLL;
      }
      if (opcode == ART_SYNC)
      {
        if (artSyncCallback) (*artSyncCallback)(remoteIP);
        return ART_SYNC;
      }
  }
  else
  {
    return 0;
  }
  return 0;
}

/**
 * @brief Prints the header information of the last received Art-Net packet.
 *
 * This function outputs the header details of the most recently received Art-Net packet
 * to the Serial console. The information includes packet size, opcode, universe number,
 * data length, and sequence number.
 *
 * @return void
 */
void Artnet::printPacketHeader()
{
  Serial.print("packet size = ");
  Serial.print(packetSize);
  Serial.print("\topcode = ");
  Serial.print(opcode, HEX);
  Serial.print("\tuniverse number = ");
  Serial.print(incomingUniverse);
  Serial.print("\tdata length = ");
  Serial.print(dmxDataLength);
  Serial.print("\tsequence n0. = ");
  Serial.println(sequence);
}

/**
 * @brief Prints the content of the DMX data in the last received Art-Net packet.
 *
 * This function outputs the DMX data content of the most recently received Art-Net packet
 * to the Serial console. Each data byte is printed in decimal format.
 *
 * @return void
 */
void Artnet::printPacketContent()
{
  for (uint16_t i = ART_DMX_START ; i < dmxDataLength ; i++){
    Serial.print(artnetPacket[i], DEC);
    Serial.print("  ");
  }
  Serial.println('\n');
}

/**
 * @brief Sets the callback function for handling Art-Net DMX packets.
 *
 * This function registers a callback function that will be invoked whenever an
 * Art-Net DMX packet is received. The callback function receives the universe,
 * data length, sequence number, DMX data, and the remote IP address as parameters.
 *
 * @param fptr A pointer to the callback function to be set.
 * @return void
 */
void Artnet::setArtDmxCallback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP)) {
    artDmxCallback = fptr;
}

/**
 * @brief Sets the callback function for handling Art-Net packets for Universe 1.
 *
 * This function registers a callback function that will be invoked whenever an
 * Art-Net packet for Universe 1 is received. The callback function receives the
 * universe, data length, sequence number, DMX data, and the remote IP address as parameters.
 *
 * @param fptr A pointer to the callback function to be set.
 * @return void
 */
void Artnet::setUniverse1Callback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP)) {
    universe1Callback = fptr;
}

/**
 * @brief Sets the callback function for handling Art-Net packets for Universe 2.
 *
 * This function registers a callback function that will be invoked whenever an
 * Art-Net packet for Universe 2 is received. The callback function receives the
 * universe, data length, sequence number, DMX data, and the remote IP address as parameters.
 *
 * @param fptr A pointer to the callback function to be set.
 * @return void
 */
void Artnet::setUniverse2Callback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP)) {
    universe2Callback = fptr;
}

/**
 * @brief Sets the callback function for handling Art-Net packets for Universe 3.
 *
 * This function registers a callback function that will be invoked whenever an
 * Art-Net packet for Universe 3 is received. The callback function receives the
 * universe, data length, sequence number, DMX data, and the remote IP address as parameters.
 *
 * @param fptr A pointer to the callback function to be set.
 * @return void
 */
void Artnet::setUniverse3Callback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP)) {
    universe3Callback = fptr;
}
/**
 * @brief Sets the callback function for handling Art-Net packets for Universe 4.
 *
 * This function registers a callback function that will be invoked whenever an
 * Art-Net packet for Universe 4 is received. The callback function receives the
 * universe, data length, sequence number, DMX data, and the remote IP address as parameters.
 *
 * @param fptr A pointer to the callback function to be set.
 * @return void
 */
void Artnet::setUniverse4Callback(void (*fptr)(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP)) {
    universe4Callback = fptr;
}

/**
 * @brief Sets the callback function for handling Art-Net Sync packets.
 *
 * This function registers a callback function that will be invoked whenever an
 * Art-Net Sync packet is received. The callback function receives the remote IP
 * address as a parameter.
 *
 * @param fptr A pointer to the callback function to be set.
 * @return void
 */
void Artnet::setArtSyncCallback(void (*fptr)(IPAddress remoteIP)) {
    artSyncCallback = fptr;
}

/**
 * @brief Reads DMX data for a specific universe from the last received Art-Net packet.
 *
 * This function copies the DMX data for a specified universe from the most recently
 * received Art-Net packet into the provided data buffer.
 *
 * @param universe The universe number to read the DMX data from.
 * @param data A pointer to the buffer where the DMX data will be copied.
 * @return void
 */
void Artnet::readUniverse(uint8_t universe, uint8_t* data){
  memcpy(data, artnetPacket + ART_DMX_START, sizeof(data));
}