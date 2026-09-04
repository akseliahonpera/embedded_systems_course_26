# TODO:

## TYPES.H

- Siistimistä
- Mietitään mitä dataa fuusiotaski tarvitsee

## GPS:

- NMEA parsetus -> mäppäys tietorakenteeksi ja lähetys jonoon
- Toiminnallisuuden miettiminen
  - Pinnejä käyttämättä: TM, FIX, HW_R, HW_S
    - FIX-tieto tulee jo NMEA-muodossa, onko laitteistofiksille käyttöä?
    - TM on PPS-signaali synkronointia varten. Ei liene tarvetta niin tarkkaan ajastukseen?
  - Miten GPS:ää on järkevää lukea? Kannattaako datanlähetystaajuutta laskea?

## IMU:

- mäppäys ja lähetys jonoon
- virkistystaajuus?

## BARO:

- mäppäys ja lähetys jonoon
- Toiminnallisuuden miettiminen
  - Onko barometrissä jotain pinnejä/keskeytyksiä mitä hyödyntää?
  - Onko virkistystaajuus hyvä?
  - Pollaako liikaa?

## FUSION:

- Fuusioalgoritmin toteutus, voi olla aluksi simppeli datan eteenpäinlähetys.
- Koostetaan tietorakenteeseen fusion_state_t -> fusion_msg_t, lähetys jonoon telemetriataskille

## TELEMETRY:

- Wifin toteutus
- Fuusioidun datan lähetys

## NOTE:

Nykynen jonototeutus on vähän hasardi, koska se on FIFO ilman päällekirjotusta

- voi käydä niin, että esim. IMU saturoi jonon kokonaan, jos anturidataa tuotetaan nopeammin kuin sitä kulutetaan.
  - tällöin barometri ja gps eivät välttämättä saa puskettua ollenkaan dataa jonoon.

Jos tämä on ongelma, niin voi korjata niin, että muuttaa jonon pituuden yhdeksi,
ja käyttää xQueueOverwriteä, jolloin ainoastaan uusin viesti menee jonoon.