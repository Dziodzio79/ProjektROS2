# ProjektROS2 — Model robota RRR `|-*`

Projekt zawiera model robota typu **RRR** przygotowany w ROS2.  
Konfiguracja osi robota odpowiada układowi:

```text
|-*
```

czyli:

- pierwszy przegub obraca się wokół osi pionowej,
- drugi przegub jest ustawiony prostopadle względem pierwszego,
- trzeci przegub jest ustawiony zgodnie z przyjętą konfiguracją `*`,
- model zawiera również efektor końcowy.

---

## Zawartość projektu

W projekcie znajdują się między innymi:

```text
launch/
resource/
robotModel/
projektDM/
setup.py
setup.cfg
package.xml
test/
```

Najważniejsze elementy:

- model robota w formacie URDF,
- pliki uruchomieniowe ROS2,
- skrypty Python do sterowania przegubami,
- skrypt do wizualizacji przestrzeni roboczej,
- konfiguracja pakietu ROS2.

---

## Model robota

Robot składa się z:

- czarnej podstawy,
- trzech niebieskich przegubów,
- czterech srebrnych prętów konstrukcyjnych,
- efektora końcowego.

W modelu przyjęto:

```text
ROD_LENGTH = 0.30 m
JOINT_LENGTH = 0.34 m
BASE_HEIGHT = 0.45 m
```

Pozycja startowa została ustawiona tak, aby robot po uruchomieniu miał poprawną orientację względem konfiguracji `|-*`.

---

## Sterowanie przegubami

Do zadawania kątów wykorzystywany jest publisher `JointState`.

Kąty podawane są w stopniach, a następnie konwertowane na radiany.

Przykładowe przeguby:

```text
arm_1_joint
arm_2_joint
arm_3_joint
```

Przykładowe konfiguracje testowe:

```text
0 0 0
90 90 90
180 180 180
```

---

## Workspace

Projekt zawiera również skrypt do generowania przestrzeni roboczej robota.

Workspace uwzględnia:

- zakresy ruchu przegubów,
- efektor końcowy,
- dolny zakres ruchu,
- konfiguracje krytyczne takie jak `90 90 90` oraz `180 180 180`.

Workspace jest publikowany jako marker w RViz.

---

## Budowanie projektu

W katalogu workspace ROS2:

```bash
colcon build
```

Następnie:

```bash
source install/setup.bash
```

---

## Uruchamianie

Przykładowe uruchomienie launch file:

```bash
ros2 launch projektDM <nazwa_pliku_launch>.py
```

Uruchomienie sterowania przegubami:

```bash
ros2 run projektDM <nazwa_skryptu>
```

Uruchomienie workspace:

```bash
ros2 run projektDM <nazwa_skryptu_workspace>
```

Nazwy plików launch oraz skryptów należy dopasować do aktualnych nazw w pakiecie.

---

## Docker

Projekt był przygotowywany w środowisku Docker/ROS2.

Przykładowe wejście do działającego kontenera:

```bash
docker exec -it <nazwa_kontenera> bash
```

Jeśli kontener jest zatrzymany:

```bash
docker start <nazwa_kontenera>
docker exec -it <nazwa_kontenera> bash
```

---

## Git

Repozytorium projektu:

```text
https://github.com/Dziodzio79/ProjektROS2
```

Podstawowe komendy:

```bash
git add .
git commit -m "Update project"
git push
```

---

## Autor

Projekt wykonany jako model robota RRR w ROS2.
