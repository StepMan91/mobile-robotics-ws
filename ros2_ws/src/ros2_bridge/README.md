# Pont UDP -> ROS2

Ce dossier contient le code à exécuter côté **Robot/WSL** (dans votre projet `mobile-robotics-ws`).

## Installation

1.  Copiez `human_bridge_node.py` dans un package ROS2 (ex: `my_robot_control/scripts/`).
2.  Assurez-vous qu'il est exécutable sur Linux : `chmod +x human_bridge_node.py`.
3.  Lancez le : `python3 human_bridge_node.py` (ou via `ros2 run`).

## Fonctionnement

*   Ce nœud écoute le port UDP **8888** par défaut.
*   Il reçoit les JSON envoyés par l'application Windows.
*   Il convertit les squelettes en `visualization_msgs/MarkerArray`.
*   Il publie sur le topic `/human_skeleton`.

## Visualisation

1.  Ouvrez Rviz2.
2.  Ajoutez un affichage "MarkerArray".
3.  Abonnez-vous au topic `/human_skeleton`.
4.  Assurez-vous que le "Fixed Frame" est `camera_link` (ou ajoutez une TF statique).

## Transformation de Coordonnées

Le script applique une conversion simple pour passer du repère Caméra (Z=Profondeur) au repère Robot (X=Devant).
*   `ROS_X` = `Cam_Z`
*   `ROS_Y` = `-Cam_X`
*   `ROS_Z` = `-Cam_Y`
