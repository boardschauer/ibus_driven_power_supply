# ibus_driven_power_supply
Dieses Programm steuert ein Netzteil (geschaltet über
ein Transistor und Relais am PB0 so dass
nach einer längeren Inaktivität am IBUS das Netzteil
abgeschaltet wird.
Erfolgt wieder Kommunikation auf dem IBUS
wird das Netzteil augenblicklich eingeschaltet.
Drückt der Benutzer die vier Lenkrad-Tasten
in richtiger Reihenfolge hintereinander,
wird das Netzteil für eine kurze Zeit abgeschaltet
(Reset).
