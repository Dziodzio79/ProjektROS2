# ProjektROS2

Model robota RRR w ROS2 dla konfiguracji osi:

```text
|-|
```

Repozytorium zawiera pakiet `projektDM` z modelem URDF, plikiem launch oraz skryptami do sterowania przegubami i wizualizacji workspace.

## Struktura projektu

```text
launch/
  robotDM.launch.py

robotModel/
  model robota URDF

projektDM/
  simulate_joints.py
  workspace.py

resource/
test/
package.xml
setup.py
setup.cfg
```

## Najważniejsze elementy

### `robotDM.launch.py`

Plik launch uruchamia model robota w ROS2/RViz.

Uruchomienie:

```bash
ros2 launch projektDM robotDM.launch.py
```

### `simulate_joints.py`

Skrypt publikuje wiadomości `JointState` na temat `/joint_states`.

Służy do ręcznego zadawania kątów trzech przegubów:

```text
arm_1_joint
arm_2_joint
arm_3_joint
```

Kąty są wpisywane w stopniach i konwertowane na radiany.

Uruchomienie:

```bash
ros2 run projektDM simulate_joints
```

### `workspace.py`

Skrypt generuje i publikuje marker workspace robota.

Workspace uwzględnia:
- zakresy ruchu przegubów,
- efektor końcowy,
- dolny obszar pracy,
- konfiguracje krytyczne, między innymi `90 90 90` oraz `180 180 180`.

Marker publikowany jest na temat:

```text
/workspace_marker
```

Uruchomienie:

```bash
ros2 run projektDM workspace
```

## Budowanie

Z poziomu workspace ROS2:

```bash
colcon build
source install/setup.bash
```

Po przebudowaniu pakietu można uruchomić launch oraz skrypty:

```bash
ros2 launch projektDM robotDM.launch.py
ros2 run projektDM simulate_joints
ros2 run projektDM workspace
```

## Uwagi

Projekt zakłada konfigurację robota RRR `|-|`.

Model, sterowanie przegubami oraz workspace są przygotowane pod wizualizację w RViz.
