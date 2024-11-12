# Dat160 Prosjekt

Inneholder et par scripts for å gjøre ting litt enklere.

`setup_symlinks` scriptet lager symlinks for hver package i denne repository til din lokale ros2 workspace. Dette vil gjøre at alle endringer fra git automatisk havner i workspace, og vice versa. 


`new_package *package_name*` scriptet lager en ny pakke, bruk `new_package -s *package_name*` så lager den automatisk en symlink i default ros2 workspace


`build` scriptet bytter mappe til ros2 workspace, og kjører build kommandoen. Kan også bruke `build -s` for å bruke `--symlink-install` option til `colcon build`


`source` scriptet sourcer setup filen etter en `build`. For at terminalen skal riktig oppdateres, så kjører en `. source` i repo folder


Merk! Etter en har kjørt `setup_symlinks` scriptet vil alt fungere som vanlig, og packages havner der de vanligvis er. De andre skriptene er kun laget for å slippe å "hoppe" mellom git repo mappen og workspace mappen hver gang en skal gjøre ting.
Eneste er at evt. nye packages havner ikke i git repository automatisk, da anbefaler jeg å bruke `./new_package` fra git repository mappen.

## Setup

Først velg et sted å ha din lokale repository, jeg anbefaler å bruke `mkdir -p ~/git`
Naviger til nye mappen: `cd ~/git`
Deretter clone denne repository til VM-en: `git clone https://github.com/h598062/dat160-prosjekt.git`

Set så opp symlinks for packages som er i repo til din lokale workspace: `./setup_symplinks`

for å kjøre scriptene så må du navigere til git mappen (f.eks. `cd ~/git/dat160-prosjekt`) og deretter bruke `./` før script navnene. Kan også kjøre dem via `bash *script navn*`
