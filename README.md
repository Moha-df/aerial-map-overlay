# Drone Image Overlay on Satellite View

Application OpenGL/C++ qui affiche une vue satellite (ArcGIS World Imagery) et superpose des images prises par drone à leur position géographique correcte.

---

## Rendu final

Au-delà du placement GPS brut, chaque image drone est désormais **recalée par rapport aux images précédentes** pour reconstruire progressivement une mosaïque cohérente.

Nouveautés :

- **Recalage ORB + RANSAC** : chaque nouvelle image est rapprochée de ses 4 voisines GPS les plus proches via les features ORB. Une transformation similarity (translation + rotation + échelle) est estimée par `cv::estimateAffinePartial2D` et appliquée par-dessus la position GPS
- **Image 0 = ancre** placée par GPS, puis chaque image suivante s'ajuste finement par rapport aux features visuelles communes avec les images déjà placées.
- **Repli automatique** : si le matching échoue (zone uniforme, peu de chevauchement), l'image retombe sur sa position GPS pure.
- **Bordures colorées** indiquant l'état d'alignement : bleu = ancre, vert = recalé visuellement, rouge = repli GPS.
- **Toggle ImGui** "Use ORB refinement" pour comparer en direct rendu GPS brut vs rendu recalé.

---

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
- **OpenCV** (modules `core`, `imgproc`, `features2d`, `calib3d` — utilisés pour ORB + RANSAC)
- Dear ImGui (cloné depuis GitHub)
- stb_image, stb_image_resize2 (headers téléchargés)
- exiftool (pour la lecture EXIF, optionnel si ODM `images.json` est présent)

### Installation (Arch Linux)

```bash
sudo pacman -S glfw glew curl opencv pkgconf
```

### Installation (Windows / MSYS2 UCRT64)

```bash
pacman -S --needed mingw-w64-ucrt-x86_64-gcc \
                   mingw-w64-ucrt-x86_64-make \
                   mingw-w64-ucrt-x86_64-pkgconf \
                   mingw-w64-ucrt-x86_64-glfw \
                   mingw-w64-ucrt-x86_64-glew \
                   mingw-w64-ucrt-x86_64-curl \
                   mingw-w64-ucrt-x86_64-opencv \
                   git
```

## Compilation et exécution

```bash
# Cloner Dear ImGui
git clone --depth 1 https://github.com/ocornut/imgui.git imgui_lib

# Télécharger les headers stb
curl -sL https://raw.githubusercontent.com/nothings/stb/master/stb_image.h -o stb_image.h
curl -sL https://raw.githubusercontent.com/nothings/stb/master/stb_image_resize2.h -o stb_image_resize2.h

# Compiler (Linux)
make
./drone_overlay

# Compiler (Windows MSYS2 UCRT64)
mingw32-make
./drone_overlay.exe
```

Le `Makefile` détecte automatiquement la plateforme et utilise les bons flags de link OpenGL (`-lGL` sur Linux, `-lopengl32` sur Windows). Pour OpenCV, il essaie d'abord `pkg-config`, puis fallback sur les chemins standards `/ucrt64/include/opencv4`, `/mingw64/include/opencv4`, `/usr/include/opencv4`.

## Structure des données

Le programme attend :
- Un dossier `images/` contenant les photos drone (JPG)
- Un dossier ODM (auto-détecté, ex: `odm-aukerman-OqlZ1jka/`) contenant :
  - `images.json` : métadonnées par image (GPS, yaw, pitch, roll, omega, phi, kappa)
  - `cameras.json` : calibration caméra (focale normalisée, point principal, distorsion)

## Approche

### 1. Placement initial (GPS + sténopé)

Pour chaque image, on calcule une pose prédite à partir des métadonnées EXIF/ODM :
- **Position** : coordonnées GPS du drone (latitude, longitude)
- **Taille** : emprise au sol par le modèle sténopé — `ground_width = altitude_AGL / focal_x`
- **Rotation** : yaw (heading géographique du drone)

Cette approche seule est approximative : le GPS embarqué a une précision de quelques mètres, et le yaw du drone ne correspond pas exactement à l'orientation de l'image au sol.

### 2. Recalage progressif par ORB + RANSAC

Pour corriger ces erreurs résiduelles, chaque image (sauf la première, qui sert d'**ancre**) est recalée vis-à-vis des images précédentes :

1. Extraction de ~3000 features **ORB** sur la version 512 px de l'image (worker thread, en parallèle du chargement texture).
2. Sélection des **4 voisines GPS les plus proches** parmi les images déjà chargées.
3. Matching descripteurs avec **BFMatcher Hamming** + **ratio test de Lowe** (0.80).
4. Les keypoints des deux côtés sont projetés en coordonnées mosaïque (mosaic-pixels au zoom 19), via la pose prédite de l'image courante et la pose corrigée des voisines.
5. **`cv::estimateAffinePartial2D`** (RANSAC, seuil ≈ 2 m) pour estimer la similarity (translation + rotation + échelle) qui fait passer la prédiction à la position correcte.
6. Si ≥ 12 inliers : on applique la correction (nouvelle lat/lon/yaw/scale). Sinon : repli sur la pose GPS pure.

Matcher contre plusieurs voisines (et pas juste l'image i-1) limite la dérive : les erreurs ne s'accumulent pas en chaîne.

## Contrôles

| Action | Contrôle |
|--------|----------|
| Déplacer la carte | Clic gauche + glisser |
| Zoom | Molette |
| Quitter | Echap |
| Réglages | Panneaux ImGui (Controls, Images) |

Dans le panneau **Controls** :
- **Use ORB refinement** : toggle pour comparer placement GPS brut vs recalé en temps réel.
- **Show status borders** : affiche les bordures colorées (bleu / vert / rouge) selon l'état d'alignement de chaque image.
- **Aligned / GPS fallback** : compteurs de la qualité globale du calage.
