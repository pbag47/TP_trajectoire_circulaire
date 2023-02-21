# TP_trajectoire_circulaire - Branche "Windows" de déploiement / exploitation
___

TP_trajectoire_circulaire est un projet Python destiné à l'enseignement du module d'automatique à des étudiants en école d'ingénieur.
Il permet de mettre en oeuvre un drone Bitcraze Crazyflie avec un robot terrestre, et sert de support au développement de lois de commandes pour asservir 
l'attitude du drone selon un objectif de position à atteindre.

Ce fichier détaille la configuration et l'installation des différents modules sur un PC équipé d'un système d'exploitation Windows 10 et d'une connexion Internet fiable.

## 1. Pré-requis
Ce projet a été développé en Python 3.10 avec le logiciel Pycharm Community Edition sur une machine Ubuntu 22.04 "Jammy Jellyfish" LTS (branche "master"). Cette branche permet le déploiement de ce projet sur une machine Windows.

### 1.1. Installation et configuration de Python
- Installer Python 3.10 et son gestionnaire de paquets "pip"  partir du site officiel de Python.
- Ouvrir un nouvel invité de commande et mettre à jour pip :
  + C:\Users\ [nom d'utilisateur] > ```pip install --upgrade pip```

### 1.2. Instalation de PyCharm
- Installer PyCharm Community Edition via le site officiel de JetBrains.

### 1.3. Installation de JupyterLab
- Installer JupyterLab : 
  + C:\Users\ [nom d'utilisateur] > ```pip install jupyterlab```

___

## 2. Import du projet

Cette partie décrit comment importer ce projet de GitHub vers le PC utilisé.

- Ouvrir Pycharm.
- Dans l'écran d'accueil, cliquer sur "Get from VCS"
- Sélectionner "Repository URL"
- Copier-coller le lien suivant : https://github.com/pbag47/TP_trajectoire_circulaire.git
- Spécifier le répertoire de destination
- Cliquer sur "Clone"

Les fichiers sont alors téléchargés sur le PC, et Pycharm ouvre le projet.

- Sur Pycharm, en bas à droite, cliquer sur la mention "No interpreter", ou "Python 3.10"
- Cliquer sur "Interpreter settings"
- Cliquer sur "Add Interpreter" -> "Add local interpreter" -> "Virtualenv Environment" -> "New"
- Laisser les paramètres par défaut, puis cliquer sur OK.

Pycharm crée alors un environnement virtuel Python dédié à ce projet et le gère automatiquement dans le dossier "venv".

___

## 3. Installation des librairies Python

### 3.1 Pour JupyterLab

Deux Notebooks sont contenus dans ce projet. 
Documentation.ipynb contient la documentation et fonctionne avec les libraires fournies par l'installation de JupyterLab.
En revanche, Flight_logs_dashboard.ipynb, qui sert à tracer les enregistrements du vol précédent, requiert des librairies Python supplémentaires.
- Installer la librairie Python "ipywidgets" :
  + C:\Users\ [nom d'utilisateur] > ```pip install ipywidgets```
- Installer la librairie Python "plotly" :
  + C:\Users\ [nom d'utilisateur] > ```pip install plotly```

### 2.2. Pour le programme Python principal

Ce projet est livré avec un fichier "requirements.txt" contenant le nom de toutes les librairies nécessaires. Les fonctions intégrées de PyCharm permettent d'interpréter ce fichier et d'installer automatiquement ces librairies dans l'environnement virtuel Python dédié au projet (à condition que le PC soit connecté à Internet).
- Ouvrir le fichier "requirements.txt" sur Pycharm.
- Repérer la notification qui indique que des packages sont manquants.
- Cliquer sur "Install Packages".
- Attendre la fin de l'installation

___

Les installations sont alors terminées et tous les programmes Python du projet peuvent être exécutés à partir de PyCharm.
JupyterLab peut être démarré à partir de l'invité de commande avec la commande suivante :
  + C:\Users\ [nom d'utilisateur] > ```jupyter-lab```

Ouvrir le Notebook "Documentation.ipynb" avec JupyterLab pour obtenir des informations supplémentaires et avoir accès au mode d'emploi.

