# Équipe 106 x Agence Spatiale Polymtl

Ce projet est l'implémentation de la demande de l'Agence Spatiale de Polytechnique Montréal par l'équipe 106 du cours de Projet intégrateur 3.

Il consiste en un remplacement d'un gros robot explorateur cher qui peut seulement explorer un terrain tout seul et celà de manière limitée et lente. À la place, plusieurs petits robots seront mis en place et pourront simulatanément explorer un terrain et ce à petit coût.


## Demo

Quelques démos du projet.

PDR R.F.1 (Vidéo Youtube) :
[![PDR Demo R.F.1](https://img.youtube.com/vi/DeXg1fxnp5c/0.jpg)](https://youtu.be/DeXg1fxnp5c "PDR Demo R.F.1")

PDR R.F.2 (Vidéo Youtube) :
[![PDR Demo R.F.2](https://img.youtube.com/vi/9uRGYGRVol4/0.jpg)](https://youtu.be/9uRGYGRVol4 "PDR Demo R.F.2")

CDR Démo Simulation (Vidéo Youtube) :
[![CDR Démo Simulation](https://img.youtube.com/vi/KJSRqRf6pac/0.jpg)](https://youtu.be/KJSRqRf6pac "CDR Démo Simulation")

CDR Démo Robots Physiques (Vidéo Youtube) :
[![CDR Démo Robots Physiques](https://img.youtube.com/vi/zJLV-Qb_sj0/0.jpg)](https://youtu.be/zJLV-Qb_sj0 "CDR Démo Robots Physiques")


## Installation

Si vous voulez lancer le programme dans votre machine directement, il faudra faire ces étapes à l'avance. Et être sous Ubuntu Jammy (22.04).

### ROS2 et Gazebo
Téléchargez le fichier suivant : https://github.com/Sh3mm/INF3995-Templates/blob/main/Installation/Ubuntu22.04/install.sh

```sh
sudo ./install.sh
```

### npm v18.19.0
```sh
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.3/install.sh | bash
source ~/.nvm/nvm.sh
nvm install 18.19.0
nvm use 18.19.0
```

### dépôt git
```sh
git clone https://github.com/OAXP/ProjetIntegrateur-3.git
```

### dépendances ROS
```sh
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
```

### packages python
```sh
sudo pip install setuptools==58.2.0
sudo pip install playsound
```

### YLidar-sdk
```sh
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
cd ../../
rm -r YDLidar-SDK
```
## Lancer localement

### Démarrage du frontend
```sh
cd ./frontend/robot-commander
npm i
npm start
```

### Démarrage du backend
Il faut avoir fait colcon build dans la simulation_ws ou dans robot_physique_ws puis faire un
```sh
source install/setup.bash
```
Puis revenir dans le répertoire `./backend_server` avec `cd`.
```sh
npm i
npx generate-ros-messages
npm start
```

### Démarrage du robot physique
```sh
cd ./robot_physique_ws
colcon build
source install/setup.bash
ros2 launch robot_bringup robot.launch.py
```

### Démarrage de la simulation
```sh
cd ./simulation_ws
colcon build
source install/setup.bash
ros2 launch simulation_bringup diff_drive.launch.py
```

### Problèmes connus
En cas d'erreur de tf2_geometry_msgs, exécutez ces commandes :
```sh
cd /usr/local/include/
sudo ln -s /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs .
```
## Docker

Profils disponibles : station, backend, frontend, robot, simulation.

- station   : lance les serveurs de backend et frontend
- backend   : lance le serveur de backend
- frontend  : lance le serveur de frontend
- robot     : lance le code du robot physique
- simulation    : lance le code du robot simulé avec Gazebo

### lancer un profil Docker
```sh
docker compose --profile <profil> up -d
```

### arrêter et supprimer le Docker
```sh
docker compose down
```
