# Émulateur GR5515 (validation avant flash)

Valide nos firmwares T1000-A sans risquer le boîtier. Basé sur Unicorn (Cortex-M4) + pyelftools.

| Fichier | Rôle |
|---|---|
| `rom_check.py` | Vérifie que le firmware Seeed (qui fonctionne sur le boîtier) appelle la ROM aux adresses de la table de symboles de notre SDK → même ROM. Résultat : 73/78 fonctions, 96 % des appels. |
| `gr5515_sim.py` | Émulateur : RAM (double vue 0x30000000/0x00800000), Flash NOR simulée, état laissé par la ROM, quelques registres matériels modélisés (XIP, RNG, compteur AON), appels ROM remplacés par des bouchons. |
| `run_boot.py` | Démarre une image jusqu'à la boucle principale, avec trace en cas d'échec. |
| `test_hello.py` | 12 scénarios / 40 vérifications du firmware de test : compteur de démarrages, DFU par bouton, par console, et automatique si le BLE reste muet (≈ 10 min), chien de garde, zones Flash écrites. |

```
.claude-work/.venv/bin/python -I tools/sim/test_hello.py firmware/hello/build/t1000a_hello.elf
```

**Limites** : la ROM de la puce n'est pas disponible. La pile BLE, la gestion d'énergie et les pilotes ROM ne sont
pas exécutés (bouchons). Le comportement électrique et les durées réelles ne sont pas modélisés.
L'émulateur valide notre code et l'enchaînement du démarrage du SDK, pas le matériel.
