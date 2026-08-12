# esphome-husqvarna-automower-220ac

Beta ESPhome firmware for husqvarna 220 ac (and probably 230 and some other from the same generation)

En ESP32 kopplas till serviceporten på robotens moderkort och talar Husqvarnas
G2-protokoll över UART, 9600 8N1. Sensorer, knappar och timerschema exponeras
mot Home Assistant.

## Hårdvara

| Del              | Val                                                       |
| ---------------- | --------------------------------------------------------- |
| Kort             | ESP32-DevKitC V4 med WROOM-32U, extern u.FL-antenn        |
| Matning          | 5 V från serviceporten, in på kortets 5V-stift            |
| Säkring          | PTC 0,75 A (RXEF075) i serie på 5V-ledaren                |
| Avkoppling 5 V   | 470 µF låg ESR + 100 nF parallellt över 5V och GND        |
| Avkoppling 3,3 V | 10 µF + 100 nF parallellt över 3V3 och GND vid modulen    |
| Kontakt          | 10-polig Molex KK, färdigkrympt kabel, oanvända avklippta |
| UART             | UART2, GPIO16 = RX, GPIO17 = TX, 9600 8N1                 |
| Låda             | Ventilationshål på två motstående sidor                   |

Serviceporten bär Rx, Tx, GND, 5V, 3V3 och 18V, verifierat med multimeter.

Stiftnumreringen är inte dokumenterad här. Ordningen i schemat nedan är
grupperad efter funktion och är **inte** stiftordning. Läs av stift 1 på
kortets serigrafi — fyrkantig lödö, alternativt etta eller triangel — och
bekräfta varje ledare med multimeter innan inkoppling. 18V-ledaren ligger på
samma kontakt och förstör kortet om den hamnar fel.

## Kopplingsschema

```
 Moderkort, serviceport                          ESP32-DevKitC V4
 (10-polig Molex KK)
                        PTC 0,75 A
      5V  ────────────────[ ~~~ ]───────────┬───────────  5V
                                            │
                                     470 µF ┴ 100 nF
                                            │
      GND ──────────────────────────────────┴───────────  GND
                                                    │
                                            10 µF ──┴── 100 nF
                                            (över 3V3–GND vid modulen)

      Tx  ──────────────────────────────────────────────  GPIO16  (RX)

      Rx  ◄─────────────────────────────────────────────  GPIO17  (TX)

      3V3 ──  används ej
      18V ──  avklippt och isolerad
```

## Varför komponenterna sitter där

**5 V istället för 3,3 V.** Kortets LDO behöver spänningsmarginal för att
reglera. Robotens 3,3V-lina är dessutom svagt dimensionerad.

**PTC-säkring på 5V-ledaren.** Återställande säkring som skyddar robotens
matning om något kortsluter i ESP-änden. 0,75 A ligger över kortets normala
förbrukning inklusive WiFi-toppar, men under vad ledaren tål.

**470 µF + 100 nF på 5 V.** Elektrolyten tar sändningstopparna när WiFi-radion
går igång, så matningen inte dippar. Keramen tar det snabba högfrekventa
innehållet som elektrolyten är för långsam för. Kabeln från moderkortet är lång
nog att ha egen induktans, vilket gör buffringen nödvändig.

**10 µF + 100 nF på 3,3 V vid modulen.** Samma princip nära WROOM-modulen, där
strömrycken uppstår.

**18V-ledaren avklippt.** Den behövs inte och skulle förstöra kortet vid
felkoppling.

**Ventilationshål.** Kortet sitter i en sluten låda i en maskin som står i sol.

**UART2, inte UART0.** GPIO1/GPIO3 delas med USB-seriellkretsen. Bootloadern
skriver text på TX0 vid varje uppstart, vilket skulle gå rakt in i robotens
diagnostikport.

**Extern u.FL-antenn.** Roboten är i praktiken en jordad plåt- och plastlåda
som dämpar en kortmonterad antenn.

## Konfiguration

Se [`hqam-esphome.yaml`](hqam-esphome.yaml). Enhetens egen yaml hämtar
komponent och paket från det här repot, så bara den filen behövs.

`logger:` måste deklareras explicit — `components/confs/button.yaml` använder
`logger.log`, och utan den failar valideringen med
`Couldn't find any component that can be used for 'logger::Logger'`.

`wifi.output_power: 12dB` används i den fungerande konfigurationen. Sänkt
sändareffekt drar ner strömtopparna, vilket är relevant på en säkrad matning.

`packages.files` är en explicit lista. Nya filer i repot hämtas inte förrän de
läggs till där.

Kräver ESPHome 2024.6 eller senare för `datetime`-plattformen och
`ota:`-listsyntaxen.

## Home Assistant

[`dashboard-automower.yaml`](dashboard-automower.yaml) är en kontrollpanels-
replika som klistras in via Dashboard > Redigera > Raw configuration editor.
Kräver `button-card`, `stack-in-card` och `card-mod` från HACS.

## Status

Beta. Flera register är avlästa men inte kalibrerade, och är märkta som
diagnostik. Statuskoderna i `publishStatus()` är ofullständiga — okända koder
visas som `STATUS_xxxx` med rå hexkod.
