# Drone Pose Control

Programma che permette di controllare un drone SJTU in ambiente di simulazione Gazebo attraverso i movimenti stimati attraverso la webcam.

**Configurazione**:

Compilare il progetto:

```bash
cd ros2_ws/
colcon build
```

Lanciare i nodi di stima della posa, interpretazione dei movimenti e il simulatore Gazebo:

```bash
ros2 launch launch_pkg  pose_control.launch.py
```

Attendere il caricamento del simulatore, poi, in un altro terminale, inizializzare il volo del drone con:

```bash
ros2 run gesture_controller drone_initializer
```

Ora è possibile comandare il drone nell'ambiente di simulazione attraverso i seguenti gesti:
- Salita: alzare la mano destra
- Discesa: alzare la mano sinistra
- Hover: tenere entrambe le mani lungo i fianchi
- Avanti: portare la mano destra sulla spalla destra
- Indietro: portare la mano sinistra sulla spalla sinistra
- Rotazione a destra: distendere il braccio destro parallelo al terreno
- Rotazione a sinistra: distendere il braccio sinistro parallelo al terreno
