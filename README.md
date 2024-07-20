# Artnet

An Art-Net library for ESP32-C3 (ESP32 and W6100 muntested)

Note: this library assumes you are using [this library](https://github.com/khoih-prog/AsyncUDP_ESP32_SC_Ethernet) for Ethernet

## edits
* a [different library](https://github.com/khoih-prog/AsyncUDP_ESP32_SC_Ethernet) is used for UDP package handling (suitable for ESP32-C3)
* more callback-functions are implemented (see ```Artnet.h```)
* NOTE: To get it work on ESP32-C3 you have to edit the SPI_HOST variable in khoih's library manually:
    * ```WebServer_ESP32_SC_W5500/src/w5500/esp32_sc_w5500.h:68:100```: change ```SPI3_HOST```to ```SPI2_HOST```
    * ```WebServer_ESP32_SC_W6100/src/w6100/esp32_sc_w6100.h:68:100:``: change ```SPI3_HOST```to ```SPI2_HOST```

## Acknowledgements

Many thanks to Nathanaël Lécaudé for the [original development of this library.](https://github.com/natcl/Artnet)

Many thanks to virtualdave and [mortonkopf](http://orchardelica.com/wp/artnet-multiple-universe-with-teensy-3-and-octows2811) on the pjrc [forums](http://forum.pjrc.com/threads/24688-Artnet-to-OctoWS2811?highlight=artnet) for the original sketches !

Avec la très aimable participation de Nicolas "Magic" Plourde
