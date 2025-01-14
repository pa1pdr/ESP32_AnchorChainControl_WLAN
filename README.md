# ESP32 Anchor Chain Remote Contol / Counter via WLAN

This repo is copied from the original author AK-Homberger, whom deserves all credits for the original design, hard work etc.
I merely enhanced it here and there and made it suitable for my own purposes.

This repository shows how to remotely control the achor chain relay via WLAN from phone/tablet.
Anchor chain events from the chain sensor are measured and presented on the phone/tablet.

# Please have a look to the other repository with a port from this ESP32 based layout to a ESP8266 (D1 Mini) board. For the D1 mini a PCB is available also [Link](https://github.com/AK-Homberger/ESP8266_AnchorChainContol_WLAN).

Just set the WLAN ssid and password according to your needs. 
Select WLAN type with setting WiFiMode_AP_STA to "0" means Acess Point, or "1" means Client with DHCP.

Also set Chain_Calibration_Value according to your sensor (e.g. 0.33. meter per event).

If working as Access Point, connect the phone/tablet to the defined AP and start "192.168.4.1" in the browser.
If working as WLAN client, check the DHCP IP address with Serial Monitor of IDE and start browser with the shown IP address.

![Picture1](https://github.com/AK-Homberger/ESP32_ChainCounter_WLAN/blob/master/IMG_1254.PNG)

To control the anchor chain relay just press:
- "Down" for anchor down
- "Up" for anchor up
- "Stop" for Stop
- "Reset" to reset the chain counter to zero
- "T-5" to decrease the target length  with 5m
- "T+5" to increase the target length  with 5m


Features:
- Saftey stop to stop "anchor up" two events before reaching zero (can be changed in code with SAFETY_STOP).
- Safety stop if maximum chain lenght is reached (standard 40 meters, can be changed with MAX_CHAIN_LENGTH)
- Watchdog timer to stop power after 1 second inactivity of client (e.g. due to connection problems).
- Watchdog timer to detect blocking chain. Engine stops if no events are detected within 1 second for up/down command.
- Current Chain Counter is stored in nonvolatile memory. ESP32 can be switched off after anchoring (counter is restored after new start).
- Demo mode to check functionality without having a windlass / chain counter connected to ESP32 (set ENABLE_DEMO to 1).
- Enhanced demo mode will use a fake pulse, simulating a rotating gypsy (set ENABLE_DEMO to 2).

![Picture2](https://github.com/AK-Homberger/ESP32_ChainCounter_WLAN/blob/master/ESP32ChainCounterWLAN_OC_Relais.png)

The current design should work for a Quick or Lofrans anchor chain relay and chain sensor (which looks like a simple reed relay triggerd from a magnet). Connection details  for a Quick windlass/counter can be found here: https://www.quickitaly.com/resources/downloads_qne-prod/1/CHC1203_IT-EN-FR_REV001A.pdf

The resistors R4, R5 and the transistors Q3, Q4 are currently not necessary. They shall support a manual override detection in the futere (currently not yet imlemented in the code).
# Partlist:

- ESP32 [Link](https://www.amazon.de/AZDelivery-NodeMCU-Development-Nachfolgermodell-ESP8266/dp/B071P98VTG/ref=sxts_sxwds-bia-wc-drs3_0?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&cv_ct_cx=ESP32&dchild=1&keywords=ESP32) 
- D24V10F5 [Link](https://eckstein-shop.de/Pololu-5V-1A-Step-Down-Spannungsregler-D24V10F5)
- Resistor 1 KOhm [Link](https://www.reichelt.de/de/en/carbon-film-resistor-1-4-w-5-1-0-kilo-ohms-1-4w-1-0k-p1315.html?&trstct=pos_2&nbc=1) Other resistors are the same type! Click on "5% Carbon film resistors" then two times "+ more filter" to select values.
- Diode 1N4148 [Link](https://www.reichelt.de/schalt-diode-100-v-150-ma-do-35-1n-4148-p1730.html?search=1n4148)
- H11-L1 [Link](https://www.reichelt.de/optokoppler-1-mbit-s-dil-6-h11l1m-p219351.html?search=H11-l1)
- Relay [Link](https://www.reichelt.de/de/en/printrelais-2x-no-5-a-5-v-dc-g6b-2s-5vdc-p168487.html?CCOUNTRY=445&LANGUAGE=de&GROUPID=7621&START=0&OFFSET=16&SID=96Xk5YJngRlij1C8dm7WFa8cc43c9fd0145a715a7ea5bf81fdb75&LANGUAGE=EN&&r=1)
- PCB universal set [Link](https://www.amazon.de/70Stk-Doppelseitig-Lochrasterplatte-Kit-Lochrasterplatine/dp/B07BDKG68Q/ref=sr_1_6?adgrpid=70589021505&dchild=1&gclid=EAIaIQobChMI07qXtuaN7AIVjentCh3xPg80EAAYASAAEgK_-_D_BwE&hvadid=352809599274&hvdev=c&hvlocphy=9043858&hvnetw=g&hvqmt=e&hvrand=11402952735368332074&hvtargid=kwd-300896600841&hydadcr=26892_1772693&keywords=lochrasterplatine&qid=1601363175&sr=8-6&tag=googhydr08-21)


# Updates:
07.03.2023 - Version 2.0: lots of changes to make it useful on my boat

14.02.2020 - Version 1.0: Changed web page layout.

13.02.2020 - Version 0.6: Added maximum chain lenght function.

13.02.2020 - Version 0.5: Added WLAN selection (AP / Client) and WLAN reconnect.

12.02.2020 - Version 0.4: Deleted delay() in loop() to improve responsiveness of web server.

12.02.2020 - Version 0.3: Added watchdog timer, safety stop and demo mode.

10.02.2020 - Version 0.1: Initial version.

