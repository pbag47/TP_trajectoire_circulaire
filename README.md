# TP_trajectoire_circulaire
___

TP_trajectoire_circulaire est un projet Python destiné à l'enseignement du module d'automatique à des étudiants en école d'ingénieur.
Il permet de mettre en oeuvre un drone Bitcraze Crazyflie avec un robot terrestre, et sert de support au développement de lois de commandes pour asservir 
l'attitude du drone selon un objectif de position à atteindre.

Ce fichier détaille la configuration et l'installation des différents modules en partant de "zéro" (PC vierge sans système d'exploitation et connexion Internet fiable).

## 1. Pré-requis
Ce projet a été conçu pour fonctionner en Python 3.10 avec le logiciel Pycharm Community Edition sur une machine Ubuntu 22.04 "Jammy Jellyfish" LTS.


### 1.1. Installation d'Ubuntu
- Créer une clé USB bootable contenant Ubuntu 22.04 (voir documentation sur le site officiel Linux)
- Après avoir branché cette clé bootable sur le PC dédié au projet, démarrer le PC et ouvrir le menu de démarrage ou le menu du BIOS
- Configurer le périphérique démarrage pour que l'ordinateur démarre avec l'OS contenu dans la clé USB, puis enregistrer et rebooter le PC
- Une fois qu'Ubuntu est démarré depuis la clé USB, suivre les instructions et installer Ubuntu 22.04 sur le PC en configuration minimale (en simple ou dual-boot selon l'utilisation souhaitée)
- A la fin de l'installation, connecter le PC à Internet, puis lancer l'application "Software Updater"
- Installer les mises à jour les plus récentes et redémarrer le PC
- Dernière étape : ouvrir un terminal de commande et entrer les commandes suivantes : 
  + ~$ ```sudo apt-get update```
  + ~$ ```sudo apt-get upgrade```

### 1.2. Configuration de Python
Une version de Python 3.10 est livrée et installée par défaut sur Ubuntu 22.04, aucune autre installation de Python n'est nécessaire.
En revanche, le gestionnaire de paquets pip n'est pas installé par défaut.
- Installer pip pour Python3 :
  + ~$ ```sudo apt install python3-pip```

Pour séparer les projets Python dans des environnements distancts (et afin d'éviter les problèmes de compatibilité entre les librairies utilisées par différents projets),
il est préférable d'isoler chaque projet dans un environnement virtuel qui contient uniquement les librairies Python nécessaires au projet.
Le module "venv" permet de créer et gérer les environnements virtuels.
- Installer venv :
  + ~$ ```sudo apt install python3.10-venv```

### 1.3. Installation du cfclient
Le cfclient est l'application officielle de Bitcraze pour configurer et faire voler les drones Crazyflie. 
Pour ce projet, il ne sera utilisé que pour configurer les drones.
La procédure d'installation du cfclient est rappelée ici, bien qu'elle soit disponible sur le site officiel de bitcraze : 
https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/
- Créer un environnement virtuel spécifique au cfclient :
  + ~$ ```python3 -m venv cfclient``` 

Cette commande crée un dossier "cfclient" dans le répertoire ```~/``` (Home), dans lequel un nouvel interpréteur Python 3.10 est ajouté.
- Activer l'environnement virtuel :
  + ~$ ```source cfclient/bin/activate```
- Installer les pré-requis du cfclient dans son environnement virtuel :
  + (cfclient) ~$ ```sudo apt install git```
  + (cfclient) ~$ ```sudo apt insall python3-pip```
  + (cfclient) ~$ ```sudo apt install libxcb-xinerama0```
  + (cfclient) ~$ ```sudo pip3 install --upgrade pip```

- Autoriser l'utilisation des Dongle USB "CrazyRadioPA" sans les permissions d'administrateur :
  + (cfclient) ~$ ```sudo groupadd plugdev```
  + (cfclient) ~$ ```sudo usermod -a -G plugdev $USER```
  + Copier-coller le texte suivant en tant que commande : 
```  
cat <<EOF | sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie (over USB)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
```

- Installer le cfclient : 
  + (cfclient) ~$ ```pip3 install cfclient```
- Une fois installé, le cfclient peut être exécuté à partir de son environnement virtuel avec la commande suivante :
  + (cfclient) ~$ ```cfclient```
- Quitter l'environnement virtuel :
  + (cfclient) ~$ ```deactivate```

### 1.4. Instalation de PyCharm
- Installer PyCharm Community Edition via le gestionnaire de paquets "snap" :
  + ~$ ```sudo snap install pycharm-community --classic```

### 1.5. Installation de JupyterLab
- Installer JupyterLab : 
  + ~$ ```pip3 install jupyterlab```

### 1.6. Création des raccourcis
La procédure de lancement du cfclient n'est pas très "user-friendly" actuellement : il faut ouvrir un terminal de commande, activer le bon environnement 
virtuel, puis taper la commande (cfclient) ~$ ```cfclient```.
Il en va de même pour JupyterLab. C'est pourquoi je propose d'automatiser ces procédures en créant un raccourci dans la liste des applications 
pour lancer ces programmes en un clic.
- Créer un nouveau fichier sur le bureau à l'aide de l'éditeur de texte. Le nommer "cfclient.desktop".
- Télécharger le logo de Bitcraze et l'enregistrer sur le PC.
- Copier-coller le texte suivant dans le fichier "cfclient.desktop" et l'enregistrer (modifier les informations et chemins d'accès si besoin).
```
[Desktop Entry]
Version=2022.12
Name=cfclient
Comment=Official Bitcraze client to fly the Crazyflie.
Exec=/home/eae/cfclient/bin/cfclient
Icon=/home/eae/cfclient/Bitcraze_logo.png
Terminal=true
Type=Application
Categories=Application
```

- Sur le bureau, faire un clic-droit sur ce fichier, puis "Properties", "Permissions", et "Allow executing file as a program".
- Faire un deuxième clic-droit sur ce fichier, puis "Allow launching".
Le fichier ainsi créé agit comme un raccourci sur le bureau : un double clic lance le cfclient.
Pour intégrer ce raccourci à la liste des applications, il est nécessaire de lancer l'explorateur de fichiers en tant qu'administrateur avec la commande
  + ~$ ```sudo nautilus```
- A l'aide de l'explorateur de fichiers lancé en tant qu'administrateur, faire un copier-coller du raccourci "cfclient.desktop" dans le répertoire "/usr/share/applications/" (remonter le chemin d'accès jusqu'à la racine).
Le raccourci apparaît alors dans le menu des applications (touche Windows sur le bureau), et il est possible de l'ajouter à la barre des favoris 
(clic-droit, puis "add to favourites").

- Répéter la procédure avec JupyterLab en créant le fichier "jupyterlab.desktop" contenant les informations suivantes (adaptées si besoin) :
```
[Desktop Entry]
Version=3.5.3
Name=JupyterLab
Comment=Server-based interactive Notebooks service.
Exec=/home/eae/.local/bin/jupyter-lab
Icon=/home/eae/cfclient/jupyter_logo.png
Terminal=true
Type=Application
Categories=Application
```

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

Pycharm crée alors un environnement virtuel dédié à ce projet et le gère automatiquement dans le dossier "venv".

___

## 3. Installation des librairies Python

### 3.1 Pour JupyterLab

Deux Notebooks sont contenus dans ce projet. Documentation.ipynb contient la documentation et fonctionne avec les libraires fournies par l'installation de JupyterLab.
En revanche, Flight_logs_dashboard.ipynb, qui sert à tracer les enregistrements du vol précédent, requiert des librairies Python supplémentaires.
- Installer la librairie Python "ipywidgets" :
  + ~$ ```pip3 install ipywidgets```
- Installer la librairie Python "plotly" :
  + ~$ ```pip3 install plotly```

### 2.2. Pour le programme Python principal

Ce projet est livré avec un fichier "requirements.txt" contenant le nom de toutes les librairies nécessaires.
- A l'aide d'un terminal de commande, se placer dans le dossier source du projet "TP_trajectoire_circulaire" installé sur le PC (commandes ```ls``` et ```cd```)
- Activer l'environnement virtuel du projet :
  + ~/PycharmProjects/TP_trajectoire_circulaire$ ```source venv/bin/activate```
- Installer toutes les libraries listées dans le fichier "requirements.txt" en respectant les versions spécifiées :
  + (venv) ~/PycharmProjects/TP_trajectoire_circulaire$ ```python3 -m pip install -r requirements.txt```
- Quitter l'environnement virtuel :
  + (venv) ~/PycharmProjects/TP_trajectoire_circulaire$ ```deactivate```

___

Les installations sont terminées et tous les programmes Python du projet peuvent être exécutés à partir de PyCharm.
Ouvrir le Notebook "Documentation.ipynb" avec JupyterLab pour obtenir des informations supplémentaires et avoir accès au mode d'emploi.

