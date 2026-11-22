# Návod na spuštění emulatoru pomoci Renode

Renode je open-source software, dostupný na https://github.com/renode/renode

Použité soubory:
* octopus_pro.repl - definice některých parametrů specifických pro naší boardu stm32f4
* run_marlin.resc - načtení .elf z buildu, spuštění emulátoru,  spouštění scriptu specifického pro náš usecase

## Step-by-step

* Instalace renode dle návodu na githubu
* Kompilace Marlinu pomocí platformia pro Renode - nutno přeponout build enviroment v Platformiu
* Zapnout Renode
* Do okna Monitor napsat "i @C:\\\\\ROBOT-MANIPULATOR\renode\run_marlin.resc" - nutné upravit cestu k souboru run_marlin.resc
* Teď můžu posílat příkazy ve stejném okně pomocí např.: sysbus.usart1 WriteLine "M115"
* Výstup z robota vidím ve vedlejším okně které se mi také spustilo

## Příkazy Renode

Posílá libovolý G-code do simulovaného robota

    sysbus.usart1 WriteLine "M114"

## Příkazy definované v run_marlin.resc python sekci

Uloží všechny provedené step-dir pulzy do `pulses.csv` vedle `run_marlin.resc`.

    save_pulses

Smaže záznam všech step-dir půlzů

    clear_pulses
