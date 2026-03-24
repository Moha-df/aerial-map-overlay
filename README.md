# Drone Image Overlay on Satellite View

Application OpenGL/C++ qui affiche une vue satellite (ArcGIS World Imagery) et superpose des images prises par drone à leur position géographique correcte.

## Fonctionnalités

- **Vue satellite** : tuiles téléchargées depuis ArcGIS World Imagery, avec navigation (pan/zoom)
- **Superposition drone** : chaque image est placée sur la carte selon ses métadonnées GPS (latitude, longitude, altitude) et orientée selon le yaw (heading)
- **Calcul d'emprise au sol** : modèle sténopé utilisant la focale normalisée (issue de `cameras.json` ODM) et l'altitude au-dessus du sol
- **Filtrage des images obliques** : les images dont le pitch dépasse un seuil réglable sont exclues (seules les vues nadir sont pertinentes)
- **Interface ImGui** avec sliders pour ajuster en temps réel :
  - Nombre d'images affichées
  - Opacité de l'overlay
  - Élévation du terrain (pour le calcul d'altitude AGL)
  - Échelle et offset de rotation
  - Seuil de pitch max
  - Paramètres par image (visibilité, rotation)
- **Chargement asynchrone** : tuiles téléchargées en multi-thread, textures drone chargées progressivement en arrière-plan

## Dépendances

- OpenGL, GLFW, GLEW
- libcurl
- Dear ImGui (cloné automatiquement)
- stb_image, stb_image_resize2 (headers téléchargés)
- exiftool (pour la lecture EXIF, optionnel si ODM `images.json` est présent)

### Installation (Arch Linux)

```bash
sudo pacman -S glfw glew curl
```

## Compilation et exécution

```bash
# Cloner Dear ImGui
git clone --depth 1 https://github.com/ocornut/imgui.git imgui_lib

# Télécharger les headers stb
curl -sL https://raw.githubusercontent.com/nothings/stb/master/stb_image.h -o stb_image.h
curl -sL https://raw.githubusercontent.com/nothings/stb/master/stb_image_resize2.h -o stb_image_resize2.h

# Compiler et lancer
make
./drone_overlay
```

## Structure des données

Le programme attend :
- Un dossier `images/` contenant les photos drone (JPG)
- Un dossier ODM (auto-détecté, ex: `odm-aukerman-OqlZ1jka/`) contenant :
  - `images.json` : métadonnées par image (GPS, yaw, pitch, roll, omega, phi, kappa)
  - `cameras.json` : calibration caméra (focale normalisée, point principal, distorsion)

## Approche actuelle

Le positionnement repose sur les métadonnées EXIF/ODM brutes :
- **Position** : coordonnées GPS du drone (latitude, longitude)
- **Taille** : emprise au sol calculée par le modèle sténopé : `ground_width = altitude_AGL / focal_x`
- **Rotation** : yaw (heading géographique du drone)

Cette approche est approximative : le GPS embarqué a une précision de quelques mètres, et le yaw du drone ne correspond pas exactement à l'orientation de l'image au sol.

## Piste d'amélioration : recalage par SIFT

Pour obtenir un alignement précis des images drone sur la vue satellite, une approche par **mise en correspondance de points d'intérêt (SIFT)** permettrait de corriger automatiquement la position et la rotation de chaque image :

1. **Extraction de descripteurs SIFT** sur chaque image drone et sur la zone satellite correspondante
2. **Matching** des descripteurs entre image drone et tuile satellite (ratio test de Lowe pour filtrer les faux positifs)
3. **Estimation d'une homographie** (matrice 3x3) par RANSAC à partir des correspondances, qui encode simultanément la translation, rotation et échelle correctes
4. **Application de la transformation** : au lieu de plaquer un rectangle tourné, on déforme l'image drone par l'homographie pour qu'elle s'aligne pixel à pixel sur le satellite

Cette méthode, vue en cours, permettrait de s'affranchir des imprécisions du GPS et du yaw, et d'obtenir un mosaïquage quasi parfait comparable à l'orthophoto produite par ODM.

## Contrôles

| Action | Contrôle |
|--------|----------|
| Déplacer la carte | Clic gauche + glisser |
| Zoom | Molette |
| Quitter | Echap |
| Réglages | Panneaux ImGui (Controls, Images) |
