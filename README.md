# Jumeau numérique du T1000-A (validation avant flash)

Le T1000-A n'a ni USB ni SWD accessibles. Chaque essai sur le boîtier passe donc par un DFU BLE, et une
erreur au démarrage ne se rattrape que par la récupération « bouton + câble ». Le jumeau numérique
exécute **le firmware réel (l'ELF compilé)** sur un modèle du GR5515 et de ses composants. On y fait
passer les tests avant tout flash, et `t1000_ble.py flash` refuse un firmware qui échoue.

Dépôt autonome, monté en sous-module `emulator/twin` du dépôt racine
[T1000A](https://github.com/Artemia76/T1000A). Il y trouve le SDK Goodix (symboles de la ROM) et les firmwares
construits (voir `twin/paths.py` ; hors de T1000A : `T1000_ROOT=…` ou `GR551X_SDK=…`).

```
pip install -r requirements.txt                      # unicorn, pyelftools, pytest, pytest-xdist
python -m pytest tests -q -n auto --dist loadfile    # suite complète en parallèle (≈ 30 s ; ≈ 2 min sur un cœur)
python -m pytest tests -q --firmware X.elf           # un seul firmware (ce que fait le garde-fou)
python -m pytest tests -q --endurance-h 24           # endurance de 24 h virtuelles
TWIN_TURBO=0 python -m pytest tests -q -n auto       # sans accélération (exécution brute, pour comparer)
python run_twin.py firmware.elf -n 2                 # jumeaux en temps réel, joignables en TCP
meshcore-cli -t 127.0.0.1 -p 5000 infos              # le vrai client MeshCore sur le jumeau
nc 127.0.0.1 5100                                    # console BLE de secours du nœud 0
```

## Ce qui est modélisé

| Composant | Modèle (`twin/`) | Fidélité |
|---|---|---|
| CPU Cortex-M4, RAM, Flash XIP | `core.py` (Unicorn) | Code machine réel. Flash NOR (écriture = ET, effacement par 4 Ko). Mémoire d'échange BLE |
| Temps | `core.py` | Horloge unique à 64 MHz pour DWT, SysTick, minuteur AON et calendrier. Veille qui saute au prochain événement. Attentes actives accélérées (voir plus bas) |
| Interruptions | `core.py` | Rappels injectés entre deux instructions quelconques, si PRIMASK le permet, sur une pile d'interruption. `seed` fait varier les points d'injection |
| GPIO / AON / MSIO | `periph.py` `Gpio` | Niveaux, tirages, fronts et niveaux d'IRQ, composants qui imposent des niveaux |
| app_timer, UART, SPI, ADC, PWM | `periph.py` | Au niveau des pilotes `app_*` du SDK. UART au débit réel, morceaux livrés en interruption |
| Chien de garde AON | `periph.py` `Watchdog` | Redémarre vraiment le MCU s'il n'est pas rafraîchi |
| LR1110 | `lr1110.py` | Fidèle au manuel utilisateur LR1110 Rev 2.3 et aux mesures faites sur le boîtier (sonde `lr`, 2026-11-16, firmware 03.07) : chaque comportement a sa source dans l'en-tête du fichier. Protocole SPI (Stat1/Stat2/IRQ, réponse en 2ᵉ transaction, CMD_PERR / CMD_FAIL), commande perdue si BUSY est haut ou si elle réveille la puce, durées de BUSY mesurées (TCXO 5 ms, calibrations), sommeil chaud (640 µs) et froid (13,3 ms, contexte perdu), mode de repli, écoute cyclique (fenêtre / sommeil + 5,2 ms, arrêt sur RX_DONE ou NSS). Temps d'antenne exact, DIO inversée vers AON1, RNG, bruit propre à chaque radio |
| AG3335 (GNSS) | `ag3335.py` | NMEA RMC/GGA/GSA chaque seconde, seulement si GPS_EN et RESET sont hauts. Sans fix, avec fix ou trajet. Acquitte les `$PAIR` |
| Pile BLE (ROM) | `ble.py` `BleStack` | API `ble_gap/gatts/sec` : base GATT construite depuis les tables du firmware, permissions (ATT 0x05 sans appairage), appairage par code, liaisons persistantes à travers les redémarrages, files de notifications limitées |
| Téléphone | `ble.py` `Phone`, `companion.py` | Central qui se connecte, s'appaire, écrit et s'abonne. Client du protocole compagnon MeshCore |
| Éther LoRa | `world.py` | Plusieurs jumeaux avancent en pas d'1 ms. RSSI/SNR par lien, pertes, collisions sans effet de capture, injection de paquets |
| Carte T1000-A | `board.py` | Bouton AON0, LED AON3, batterie MSIO4 via pont ÷2 (actif si GPIO14=1), buzzer PWM0 |

Les structures et énumérations du SDK (`ble_evt_t`, `app_io_evt_t`…) sont lues dans le **DWARF de l'ELF**
(`layouts.py`, avec un cache dans `~/.cache/t1000-twin`). Les modèles suivent donc le SDK réellement compilé,
sans aucun décalage codé en dur.

## Tests (`tests/`)

* `test_safety.py` couvre chaque firmware destiné au boîtier (MeshCore, blink) : garde de démarrage, DFU au
  3ᵉ démarrage court, bouton maintenu 6 s, garde effacée après 2 min ou à la connexion, chien de garde de 8 s,
  écritures Flash bornées. Deux mutations sont aussi testées : un firmware bloqué dans `loop()` et un LR1110
  muet doivent finir **en DFU d'eux-mêmes**, jamais en brique.
* `test_twin_core.py` vérifie le cœur du jumeau : durée des attentes accélérées, événement servi à l'heure
  pendant une attente.
* `test_lr1110.py` vérifie le modèle LR1110 seul, au niveau SPI et sans firmware : une mesure ou une règle du
  manuel par test (≈ 0,1 s).
* `test_console.py` vérifie la console BLE de secours : `info`, `dfu`, `reboot`, deux lignes reçues d'affilée.
* `test_meshcore.py` couvre le compagnon :
  * radio (aucune commande inconnue, BUSY respecté, réception continue) ;
  * sécurité BLE et `SELF_INFO` ;
  * batterie et GNSS ;
  * validité des adverts émis ;
  * veille ;
  * endurance ;
  * réseau à deux nœuds (contacts, message + ACK), persistance après redémarrage, collision.

## Garde-fou avant flash

`t1000_ble.py flash X_dfu.bin` (outil de la plateforme gr551x) cherche l'ELF voisin (`X.elf`) et lance la suite avec `--firmware X.elf`.
Le flash est refusé si un test échoue, ou si le jumeau ou l'ELF sont introuvables. `--skip-twin` force le
flash ; avec PlatformIO, c'est `custom_upload_skip_twin = yes`. Si le Python courant n'a pas unicorn,
pyelftools et pytest, `T1000_TWIN_PYTHON` désigne celui qui les a, et `T1000_TWIN_DIR` indique le dossier du
jumeau hors du dépôt.

## Constats faits avec le jumeau

* Après l'ajout d'un contact, MeshCore garde `hasPendingWork()` vrai pendant 5 s (écriture différée des
  contacts). Le MCU tourne alors en continu, sans veille : c'est un coût d'énergie réel sur le boîtier.
* GPIO14 (pont de mesure batterie, peut-être aussi l'alimentation RF) reste à 1 en permanence. Le test
  correspondant est marqué `xfail` strict : il passera en échec quand ce sera corrigé, pour qu'on retire la
  marque.
* `g_nvds_buf` pointe dans la mémoire d'échange BLE (0xB0000000). Le modèle de mémoire doit l'inclure, sinon
  `nvds_put` faute au 2ᵉ démarrage.

## Limites : un succès est nécessaire, pas suffisant

* **La ROM n'est pas exécutée.** La pile BLE, la gestion d'énergie et les pilotes HAL en ROM sont remplacés
  par des modèles. Un bogue de la ROM, ou une mauvaise configuration qu'elle seule détecterait (horloge LF,
  alimentation, comme pour hello-0.1), n'est **pas** vu. Les réglages de démarrage restent à comparer au
  firmware Seeed.
* Aucun comportement électrique ni radiofréquence, pas de consommation. 1 instruction ≈ 1 cycle.
* Le LR1110 n'implémente que la partie LoRa utilisée par RadioLib. GNSS et Wi-Fi internes ne sont pas
  modélisés. Les commandes inconnues sont relevées (`unknown_cmds`).
* **Accélérations** (désactivables par `T1000A(..., turbo=False)` ou `TWIN_TURBO=0`) :
  * une boucle principale qui tourne à vide 8 fois de suite sans appeler aucun modèle fait sauter le temps
    au prochain événement (1 ms au plus) ;
  * une attente active (`delay()`, `delayMicroseconds()`, `delay_ms` du SDK, toute boucle sur `millis()`)
    qui relit l'horloge 16 fois sans appeler aucun modèle fait sauter le temps à chaque lecture, d'au plus
    1/16 de la durée déjà attendue (4 ms au plus). Une attente finit donc au plus ≈ 6 % trop tard ;
  * une lecture de broche qui renvoie la même valeur que la précédente ne compte pas comme activité : une
    boucle qui attend le relâchement du bouton reste une attente ;
  * une instruction `b .` saute directement au prochain événement.

  Les échéances restent exactes, car aucun saut ne dépasse le prochain événement. Seul le nombre de tours de
  boucle change.

## Ajouter un modèle

Un modèle s'attache au jumeau par `tw.on('nom_de_fonction', fn)`. `fn` reçoit les arguments AAPCS et
renvoie r0 (ou `PASS` pour exécuter la vraie fonction). Si la fonction est en ROM, le repli est
automatique. Le modèle programme ses échéances par `tw.schedule(cycles, fn)` et déclenche les rappels du
firmware par `tw.irq(nom, adresse, args, blobs)`. Son état est remis à zéro au reset par
`tw.reset_listeners`.

## Ancien simulateur

`gr5515_sim.py`, `test_hello.py` et `test_arduino.py` restent en place, et leurs 45 + 34 vérifications sont
toujours vertes. Ils valident le firmware de test `hello`, désormais figé. `rom_check.py` compare les appels
ROM du firmware Seeed à notre table de symboles.

## Licence

[MIT](LICENSE), © 2026 Artemia76. Les dépendances installées par `requirements.txt` gardent leur licence
(Unicorn : GPL-2.0, pyelftools : domaine public, pytest : MIT) ; elles ne sont pas incluses dans ce dépôt.
Projet indépendant, sans lien avec Seeed Studio ni Goodix.
